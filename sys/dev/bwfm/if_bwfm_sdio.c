/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2017 Patrick Wildt <patrick@blueri.se>
 * Copyright (c) 2024 The FreeBSD Foundation
 *
 * Portions of this software were developed with reference to OpenBSD's
 * if_bwfm_sdio.c driver.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Broadcom FullMAC SDIO WiFi driver
 *
 * This driver supports SDIO-attached Broadcom/Cypress WiFi chips including:
 * - BCM43430 (Raspberry Pi Zero W, 3B)
 * - BCM43455/CYW43455 (Raspberry Pi 3B+, 4, 5)
 */

#include <sys/cdefs.h>
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/socket.h>
#include <sys/taskqueue.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/ethernet.h>

#include <net80211/ieee80211_var.h>

#include <dev/sdio/sdiob.h>
#include <dev/sdio/sdio_subr.h>

#include <dev/bwfm/bwfmreg.h>
#include <dev/bwfm/bwfmvar.h>

#include "sdio_if.h"

/* SDIO function numbers */
#define BWFM_SDIO_FUNC_BACKPLANE	1
#define BWFM_SDIO_FUNC_FRAME		2
#define BWFM_SDIO_NUM_FUNCS		3

/* SDIO register offsets for function 1 (backplane) */
#define BWFM_SDIO_REG_BACKPLANE_ADDR_LOW	0x1000a
#define BWFM_SDIO_REG_BACKPLANE_ADDR_MID	0x1000b
#define BWFM_SDIO_REG_BACKPLANE_ADDR_HIGH	0x1000c

/* SBSDIO registers */
#define BWFM_SBSDIO_FUNC1_SBADDRLOW		0x0001000a
#define BWFM_SBSDIO_FUNC1_SBADDRMID		0x0001000b
#define BWFM_SBSDIO_FUNC1_SBADDRHIGH		0x0001000c
#define BWFM_SBSDIO_FUNC1_CHIPCLKCSR		0x0001000e
#define  BWFM_SBSDIO_FUNC1_CHIPCLKCSR_FORCE_ALP	0x01
#define  BWFM_SBSDIO_FUNC1_CHIPCLKCSR_FORCE_HT	0x02
#define  BWFM_SBSDIO_FUNC1_CHIPCLKCSR_ALP_AVAIL	0x40
#define  BWFM_SBSDIO_FUNC1_CHIPCLKCSR_HT_AVAIL	0x80
#define BWFM_SBSDIO_FUNC1_SDIOPULLUP		0x0001000f
#define BWFM_SBSDIO_FUNC1_WFRAMEBCLO		0x00010019
#define BWFM_SBSDIO_FUNC1_WFRAMEBCHI		0x0001001a
#define BWFM_SBSDIO_FUNC1_RFRAMEBCLO		0x0001001b
#define BWFM_SBSDIO_FUNC1_RFRAMEBCHI		0x0001001c
#define BWFM_SBSDIO_FUNC1_MESBUSYCTRL		0x0001001d
#define BWFM_SBSDIO_DEVICE_CTL			0x00010000
#define  BWFM_SBSDIO_DEVICE_CTL_SETBUSY		0x01
#define  BWFM_SBSDIO_DEVICE_CTL_SPI_INTR_SYNC	0x02
#define  BWFM_SBSDIO_DEVICE_CTL_CA_INT_ONLY	0x04
#define  BWFM_SBSDIO_DEVICE_CTL_PADS_ISO	0x08
#define  BWFM_SBSDIO_DEVICE_CTL_SB_RST_CTL	0x30
#define  BWFM_SBSDIO_DEVICE_CTL_RST_CORECTL	0x00
#define  BWFM_SBSDIO_DEVICE_CTL_RST_BPRESET	0x10
#define  BWFM_SBSDIO_DEVICE_CTL_RST_NOBPRESET	0x20

/* SDIO protocol frame header */
#define BWFM_SDIO_SEQ_NUM_MAX			256
struct bwfm_sdio_hdr {
	uint16_t	len;
	uint16_t	checksum;
	uint8_t		seq;
	uint8_t		channel;
#define BWFM_SDIO_CHANNEL_CONTROL	0
#define BWFM_SDIO_CHANNEL_EVENT		1
#define BWFM_SDIO_CHANNEL_DATA		2
#define BWFM_SDIO_CHANNEL_GLOM		3
	uint8_t		next_len;
	uint8_t		data_offset;
	uint8_t		flow;
	uint8_t		credit;
	uint8_t		reserved[2];
} __packed;

#define BWFM_SDIO_FRAME_HEADER_LEN	12

/* SDIO specific softc */
struct bwfm_sdio_softc {
	struct bwfm_softc	sc_sc;
	device_t		sc_dev;

	/* SDIO functions */
	struct sdio_func	*sc_func[BWFM_SDIO_NUM_FUNCS];
	device_t		sc_sdiob;	/* Parent sdiob device */

	/* Backplane window */
	uint32_t		sc_sbwad;

	/* Transmit state */
	uint8_t			sc_tx_seq;
	uint8_t			sc_tx_max_seq;

	/* Control response */
	char			*sc_rxctl_buf;
	size_t			sc_rxctl_len;
	int			sc_rxctl_done;

	/* Console buffer */
	char			*sc_console_buf;
	size_t			sc_console_bufsize;

	/* Firmware */
	const struct firmware	*sc_fw_code;
	const struct firmware	*sc_fw_nvram;
	const struct firmware	*sc_fw_clm;

	/* Taskqueue */
	struct taskqueue	*sc_tq;
	struct task		sc_task_rx;
};

/* Memory allocation tag */
static MALLOC_DEFINE(M_BWFM_SDIO, "bwfm_sdio", "Broadcom SDIO WiFi driver");

/* Bus operations for bwfm core */
static int	bwfm_sdio_preinit(struct bwfm_softc *);
static void	bwfm_sdio_stop(struct bwfm_softc *);
static int	bwfm_sdio_txcheck(struct bwfm_softc *);
static int	bwfm_sdio_txdata(struct bwfm_softc *, struct mbuf *);
static int	bwfm_sdio_txctl(struct bwfm_softc *, void *, size_t);
static int	bwfm_sdio_rxctl(struct bwfm_softc *, void *, size_t *);

/* SDIO access functions */
static uint8_t	bwfm_sdio_read_1(struct bwfm_sdio_softc *, uint32_t);
static void	bwfm_sdio_write_1(struct bwfm_sdio_softc *, uint32_t, uint8_t);
static uint32_t	bwfm_sdio_read_4(struct bwfm_sdio_softc *, uint32_t);
static void	bwfm_sdio_write_4(struct bwfm_sdio_softc *, uint32_t, uint32_t);
static int	bwfm_sdio_buf_read(struct bwfm_sdio_softc *, uint8_t, uint32_t,
		    void *, size_t);
static int	bwfm_sdio_buf_write(struct bwfm_sdio_softc *, uint8_t, uint32_t,
		    void *, size_t);

/* Backplane access */
static void	bwfm_sdio_backplane_window(struct bwfm_sdio_softc *, uint32_t);
static uint32_t	bwfm_sdio_buscore_read(struct bwfm_softc *, uint32_t);
static void	bwfm_sdio_buscore_write(struct bwfm_softc *, uint32_t, uint32_t);

/* Chip operations */
static int	bwfm_sdio_buscore_prepare(struct bwfm_softc *);
static int	bwfm_sdio_buscore_reset(struct bwfm_softc *);
static void	bwfm_sdio_buscore_setup(struct bwfm_softc *);
static void	bwfm_sdio_buscore_activate(struct bwfm_softc *, uint32_t);

/* Firmware loading */
static int	bwfm_sdio_load_firmware(struct bwfm_sdio_softc *);
static int	bwfm_sdio_load_microcode(struct bwfm_sdio_softc *,
		    const uint8_t *, size_t, const uint8_t *, size_t);

/* Device methods */
static int	bwfm_sdio_probe(device_t);
static int	bwfm_sdio_attach(device_t);
static int	bwfm_sdio_detach(device_t);

/* Bus ops structure */
static struct bwfm_bus_ops bwfm_sdio_bus_ops = {
	.bs_preinit = bwfm_sdio_preinit,
	.bs_stop = bwfm_sdio_stop,
	.bs_txcheck = bwfm_sdio_txcheck,
	.bs_txdata = bwfm_sdio_txdata,
	.bs_txctl = bwfm_sdio_txctl,
	.bs_rxctl = bwfm_sdio_rxctl,
};

/* Buscore ops structure */
static struct bwfm_buscore_ops bwfm_sdio_buscore_ops = {
	.bc_read = bwfm_sdio_buscore_read,
	.bc_write = bwfm_sdio_buscore_write,
	.bc_prepare = bwfm_sdio_buscore_prepare,
	.bc_reset = bwfm_sdio_buscore_reset,
	.bc_setup = bwfm_sdio_buscore_setup,
	.bc_activate = bwfm_sdio_buscore_activate,
};

/* Supported devices */
static const struct {
	uint16_t	vendor;
	uint16_t	device;
	const char	*desc;
} bwfm_sdio_devices[] = {
	{ 0x02d0, 0x4329, "Broadcom BCM4329 SDIO WiFi" },
	{ 0x02d0, 0x4330, "Broadcom BCM4330 SDIO WiFi" },
	{ 0x02d0, 0x4334, "Broadcom BCM4334 SDIO WiFi" },
	{ 0x02d0, 0x4335, "Broadcom BCM4335 SDIO WiFi" },
	{ 0x02d0, 0x4339, "Broadcom BCM4339 SDIO WiFi" },
	{ 0x02d0, 0x4345, "Broadcom BCM43455 SDIO WiFi" },
	{ 0x02d0, 0xa94c, "Broadcom BCM43430 SDIO WiFi" },
	{ 0x02d0, 0xa94d, "Broadcom BCM43430 SDIO WiFi" },
	{ 0x02d0, 0xa9a6, "Broadcom BCM43455 SDIO WiFi" },
	{ 0x02d0, 0x4354, "Broadcom BCM4354 SDIO WiFi" },
	{ 0, 0, NULL }
};

/*
 * SDIO register access functions
 */
static uint8_t
bwfm_sdio_read_1(struct bwfm_sdio_softc *sc, uint32_t addr)
{
	struct sdio_func *func;
	uint8_t val;
	int error;

	/* Determine which function to use based on address */
	if (addr & 0x10000)
		func = sc->sc_func[1];	/* Backplane */
	else
		func = sc->sc_func[0];	/* F0 */

	val = sdio_read_1(func, addr & 0xffff, &error);
	if (error != 0)
		device_printf(sc->sc_dev,
		    "sdio_read_1 failed: addr=%#x error=%d\n", addr, error);
	return val;
}

static void
bwfm_sdio_write_1(struct bwfm_sdio_softc *sc, uint32_t addr, uint8_t val)
{
	struct sdio_func *func;
	int error;

	if (addr & 0x10000)
		func = sc->sc_func[1];
	else
		func = sc->sc_func[0];

	sdio_write_1(func, addr & 0xffff, val, &error);
	if (error != 0)
		device_printf(sc->sc_dev,
		    "sdio_write_1 failed: addr=%#x val=%#x error=%d\n",
		    addr, val, error);
}

static uint32_t
bwfm_sdio_read_4(struct bwfm_sdio_softc *sc, uint32_t addr)
{
	struct sdio_func *func = sc->sc_func[1];
	uint32_t val;
	int error;

	val = sdio_read_4(func, addr & 0xffff, &error);
	if (error != 0)
		device_printf(sc->sc_dev,
		    "sdio_read_4 failed: addr=%#x error=%d\n", addr, error);
	return val;
}

static void
bwfm_sdio_write_4(struct bwfm_sdio_softc *sc, uint32_t addr, uint32_t val)
{
	struct sdio_func *func = sc->sc_func[1];
	int error;

	sdio_write_4(func, addr & 0xffff, val, &error);
	if (error != 0)
		device_printf(sc->sc_dev,
		    "sdio_write_4 failed: addr=%#x val=%#x error=%d\n",
		    addr, val, error);
}

/*
 * Backplane window management
 */
static void
bwfm_sdio_backplane_window(struct bwfm_sdio_softc *sc, uint32_t addr)
{
	uint32_t bar0 = addr & ~(BWFM_SBSDIO_SB_OFT_ADDR_MASK);

	if (sc->sc_sbwad == bar0)
		return;

	bwfm_sdio_write_1(sc, BWFM_SBSDIO_FUNC1_SBADDRLOW,
	    (bar0 >> 8) & 0xff);
	bwfm_sdio_write_1(sc, BWFM_SBSDIO_FUNC1_SBADDRMID,
	    (bar0 >> 16) & 0xff);
	bwfm_sdio_write_1(sc, BWFM_SBSDIO_FUNC1_SBADDRHIGH,
	    (bar0 >> 24) & 0xff);

	sc->sc_sbwad = bar0;
}

#define BWFM_SBSDIO_SB_OFT_ADDR_MASK	0x7fff

static uint32_t
bwfm_sdio_buscore_read(struct bwfm_softc *bwfm, uint32_t addr)
{
	struct bwfm_sdio_softc *sc = (struct bwfm_sdio_softc *)bwfm;
	uint32_t val;

	bwfm_sdio_backplane_window(sc, addr);

	addr &= BWFM_SBSDIO_SB_OFT_ADDR_MASK;
	addr |= BWFM_SBSDIO_SB_ACCESS_2_4B_FLAG;

	val = bwfm_sdio_read_4(sc, addr);

	return val;
}

#define BWFM_SBSDIO_SB_ACCESS_2_4B_FLAG	0x8000

static void
bwfm_sdio_buscore_write(struct bwfm_softc *bwfm, uint32_t addr, uint32_t val)
{
	struct bwfm_sdio_softc *sc = (struct bwfm_sdio_softc *)bwfm;

	bwfm_sdio_backplane_window(sc, addr);

	addr &= BWFM_SBSDIO_SB_OFT_ADDR_MASK;
	addr |= BWFM_SBSDIO_SB_ACCESS_2_4B_FLAG;

	bwfm_sdio_write_4(sc, addr, val);
}

/*
 * Chip preparation
 */
static int
bwfm_sdio_buscore_prepare(struct bwfm_softc *bwfm)
{
	struct bwfm_sdio_softc *sc = (struct bwfm_sdio_softc *)bwfm;
	uint8_t val;
	int i;

	/* Enable function 1 (backplane) */
	if (sdio_enable_func(sc->sc_func[1]) != 0) {
		device_printf(sc->sc_dev,
		    "failed to enable function 1\n");
		return ENXIO;
	}

	/* Force ALP clock */
	bwfm_sdio_write_1(sc, BWFM_SBSDIO_FUNC1_CHIPCLKCSR,
	    BWFM_SBSDIO_FUNC1_CHIPCLKCSR_FORCE_ALP);

	/* Wait for ALP to be available */
	for (i = 0; i < 100; i++) {
		val = bwfm_sdio_read_1(sc, BWFM_SBSDIO_FUNC1_CHIPCLKCSR);
		if (val & BWFM_SBSDIO_FUNC1_CHIPCLKCSR_ALP_AVAIL)
			break;
		DELAY(10000);
	}
	if (i == 100) {
		device_printf(sc->sc_dev,
		    "timeout waiting for ALP clock\n");
		return ETIMEDOUT;
	}

	/* Clear clock request */
	bwfm_sdio_write_1(sc, BWFM_SBSDIO_FUNC1_CHIPCLKCSR, 0);

	/* Disable pull-ups */
	bwfm_sdio_write_1(sc, BWFM_SBSDIO_FUNC1_SDIOPULLUP, 0);

	return 0;
}

static int
bwfm_sdio_buscore_reset(struct bwfm_softc *bwfm)
{
	return 0;
}

static void
bwfm_sdio_buscore_setup(struct bwfm_softc *bwfm)
{
	/* Nothing special needed */
}

static void
bwfm_sdio_buscore_activate(struct bwfm_softc *bwfm, uint32_t rambase)
{
	struct bwfm_sdio_softc *sc = (struct bwfm_sdio_softc *)bwfm;

	/* Request HT clock */
	bwfm_sdio_write_1(sc, BWFM_SBSDIO_FUNC1_CHIPCLKCSR,
	    BWFM_SBSDIO_FUNC1_CHIPCLKCSR_FORCE_HT);
}

/*
 * Firmware loading
 */
static int
bwfm_sdio_load_microcode(struct bwfm_sdio_softc *sc,
    const uint8_t *code, size_t codelen,
    const uint8_t *nvram, size_t nvramlen)
{
	struct bwfm_softc *bwfm = &sc->sc_sc;
	struct bwfm_core *core;
	uint32_t rambase, ramsize;
	int error;

	/* Get ARM core */
	core = bwfm_chip_get_core(bwfm, BWFM_AGENT_CORE_ARM_CM3);
	if (core == NULL)
		core = bwfm_chip_get_core(bwfm, BWFM_AGENT_CORE_ARM_CR4);
	if (core == NULL) {
		device_printf(sc->sc_dev, "no ARM core found\n");
		return ENODEV;
	}

	rambase = bwfm->sc_chip.ch_rambase;
	ramsize = bwfm->sc_chip.ch_ramsize;

	/* TODO: Write firmware to RAM via SDIO */
	/* This requires multi-byte SDIO writes which need more infrastructure */

	device_printf(sc->sc_dev,
	    "firmware loading not yet implemented\n");

	return ENOTSUP;
}

static int
bwfm_sdio_load_firmware(struct bwfm_sdio_softc *sc)
{
	struct bwfm_softc *bwfm = &sc->sc_sc;
	char fwname[64];
	int error;

	/* Determine firmware name based on chip */
	switch (bwfm->sc_chip.ch_chip) {
	case BWFM_CC_43430_CHIP_ID:
		snprintf(fwname, sizeof(fwname),
		    "brcmfmac43430-sdio.bin");
		break;
	case BWFM_CC_43455_CHIP_ID:
		snprintf(fwname, sizeof(fwname),
		    "brcmfmac43455-sdio.bin");
		break;
	default:
		device_printf(sc->sc_dev,
		    "no firmware for chip %#x\n", bwfm->sc_chip.ch_chip);
		return ENODEV;
	}

	/* Request firmware */
	sc->sc_fw_code = firmware_get(fwname);
	if (sc->sc_fw_code == NULL) {
		device_printf(sc->sc_dev,
		    "failed to load firmware: %s\n", fwname);
		return ENOENT;
	}

	/* Load microcode */
	error = bwfm_sdio_load_microcode(sc,
	    sc->sc_fw_code->data, sc->sc_fw_code->datasize,
	    NULL, 0);

	if (error != 0 && sc->sc_fw_code != NULL) {
		firmware_put(sc->sc_fw_code, FIRMWARE_UNLOAD);
		sc->sc_fw_code = NULL;
	}

	return error;
}

/*
 * Bus operations for bwfm core
 */
static int
bwfm_sdio_preinit(struct bwfm_softc *bwfm)
{
	struct bwfm_sdio_softc *sc = (struct bwfm_sdio_softc *)bwfm;
	int error;

	/* Enable function 2 (data) */
	if (sdio_enable_func(sc->sc_func[2]) != 0) {
		device_printf(sc->sc_dev,
		    "failed to enable function 2\n");
		return ENXIO;
	}

	/* Set block size for data function */
	error = sdio_set_block_size(sc->sc_func[2], 512);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "failed to set block size: %d\n", error);
		return error;
	}

	return 0;
}

static void
bwfm_sdio_stop(struct bwfm_softc *bwfm)
{
	/* Nothing special needed */
}

static int
bwfm_sdio_txcheck(struct bwfm_softc *bwfm)
{
	struct bwfm_sdio_softc *sc = (struct bwfm_sdio_softc *)bwfm;

	/* Check if we have transmit credits */
	if (sc->sc_tx_seq == sc->sc_tx_max_seq)
		return ENOBUFS;

	return 0;
}

static int
bwfm_sdio_txdata(struct bwfm_softc *bwfm, struct mbuf *m)
{
	struct bwfm_sdio_softc *sc = (struct bwfm_sdio_softc *)bwfm;
	struct bwfm_sdio_hdr *hdr;
	int error;

	/* TODO: Implement data transmission */
	m_freem(m);
	return ENOTSUP;
}

static int
bwfm_sdio_txctl(struct bwfm_softc *bwfm, void *buf, size_t len)
{
	struct bwfm_sdio_softc *sc = (struct bwfm_sdio_softc *)bwfm;

	/* TODO: Implement control transmission */
	return ENOTSUP;
}

static int
bwfm_sdio_rxctl(struct bwfm_softc *bwfm, void *buf, size_t *len)
{
	struct bwfm_sdio_softc *sc = (struct bwfm_sdio_softc *)bwfm;
	int timeout = 100;	/* 1 second timeout */

	/* Wait for control response */
	while (!sc->sc_rxctl_done && timeout-- > 0)
		DELAY(10000);

	if (!sc->sc_rxctl_done)
		return ETIMEDOUT;

	/* Copy response */
	*len = min(*len, sc->sc_rxctl_len);
	memcpy(buf, sc->sc_rxctl_buf, *len);

	sc->sc_rxctl_done = 0;
	return 0;
}

/*
 * Device methods
 */
static int
bwfm_sdio_probe(device_t dev)
{
	uint16_t vendor, device;
	int i;

	vendor = sdio_get_vendor(dev);
	device = sdio_get_device(dev);

	for (i = 0; bwfm_sdio_devices[i].vendor != 0; i++) {
		if (vendor == bwfm_sdio_devices[i].vendor &&
		    device == bwfm_sdio_devices[i].device) {
			device_set_desc(dev, bwfm_sdio_devices[i].desc);
			return BUS_PROBE_DEFAULT;
		}
	}

	return ENXIO;
}

static int
bwfm_sdio_attach(device_t dev)
{
	struct bwfm_sdio_softc *sc;
	struct bwfm_softc *bwfm;
	int error;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	bwfm = &sc->sc_sc;
	bwfm->sc_dev = dev;

	/* Get SDIO function from parent */
	sc->sc_func[1] = sdio_get_function(dev);
	if (sc->sc_func[1] == NULL) {
		device_printf(dev, "failed to get SDIO function\n");
		return ENXIO;
	}

	/* Set up bus operations */
	bwfm->sc_bus_ops = &bwfm_sdio_bus_ops;
	bwfm->sc_proto_ops = &bwfm_proto_bcdc_ops;

	/* Initialize chip */
	error = bwfm_chip_attach(bwfm, &bwfm_sdio_buscore_ops, "brcm");
	if (error != 0) {
		device_printf(dev, "failed to attach chip: %d\n", error);
		return error;
	}

	device_printf(dev, "chip %s rev %u\n",
	    bwfm->sc_chip.ch_name, bwfm->sc_chip.ch_chiprev);

	/* Load firmware */
	error = bwfm_sdio_load_firmware(sc);
	if (error != 0) {
		device_printf(dev, "failed to load firmware: %d\n", error);
		bwfm_chip_detach(bwfm);
		return error;
	}

	/* Attach to ieee80211 */
	bwfm_attach(bwfm);

	return 0;
}

static int
bwfm_sdio_detach(device_t dev)
{
	struct bwfm_sdio_softc *sc = device_get_softc(dev);
	struct bwfm_softc *bwfm = &sc->sc_sc;

	bwfm_detach(bwfm);
	bwfm_chip_detach(bwfm);

	if (sc->sc_fw_code != NULL)
		firmware_put(sc->sc_fw_code, FIRMWARE_UNLOAD);
	if (sc->sc_fw_nvram != NULL)
		firmware_put(sc->sc_fw_nvram, FIRMWARE_UNLOAD);
	if (sc->sc_fw_clm != NULL)
		firmware_put(sc->sc_fw_clm, FIRMWARE_UNLOAD);

	return 0;
}

static device_method_t bwfm_sdio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bwfm_sdio_probe),
	DEVMETHOD(device_attach,	bwfm_sdio_attach),
	DEVMETHOD(device_detach,	bwfm_sdio_detach),

	DEVMETHOD_END
};

static driver_t bwfm_sdio_driver = {
	"bwfm",
	bwfm_sdio_methods,
	sizeof(struct bwfm_sdio_softc)
};

DRIVER_MODULE(bwfm_sdio, sdiob, bwfm_sdio_driver, NULL, NULL);
MODULE_VERSION(bwfm_sdio, 1);
MODULE_DEPEND(bwfm_sdio, bwfm, 1, 1, 1);
MODULE_DEPEND(bwfm_sdio, sdhci, 1, 1, 1);
MODULE_DEPEND(bwfm_sdio, wlan, 1, 1, 1);
MODULE_DEPEND(bwfm_sdio, firmware, 1, 1, 1);
