/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2010-2016 Broadcom Corporation
 * Copyright (c) 2017 Patrick Wildt <patrick@blueri.se>
 * Copyright (c) 2024 The FreeBSD Foundation
 *
 * Portions of this software were developed with reference to OpenBSD's
 * if_bwfm_pci.c driver.
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
 * WORK IN PROGRESS - Broadcom FullMAC PCIe WiFi driver
 *
 * This driver is a skeleton for PCIe-attached Broadcom WiFi chips.
 * The PCI driver uses a more complex message-buffer protocol (msgbuf)
 * with ring buffers and DMA, as opposed to the simpler BCDC protocol
 * used by USB and SDIO drivers.
 *
 * Supported chips (not yet functional):
 * - BCM4350
 * - BCM4356
 * - BCM43602
 * - BCM4371
 * - BCM4378
 * - BCM4387
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
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_media.h>
#include <net/ethernet.h>

#include <net80211/ieee80211_var.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bwfm/bwfmreg.h>
#include <dev/bwfm/bwfmvar.h>
#include <dev/bwfm/if_bwfm_pci.h>

#ifdef BWFM_DEBUG
#define DPRINTF(x)	do { printf x; } while (0)
#define DPRINTFN(n, x)	do { printf x; } while (0)
#else
#define DPRINTF(x)	do { ; } while (0)
#define DPRINTFN(n, x)	do { ; } while (0)
#endif

/* DMA buffer sizes */
#define BWFM_DMA_D2H_SCRATCH_BUF_LEN		8
#define BWFM_DMA_D2H_RINGUPD_BUF_LEN		1024
#define BWFM_DMA_H2D_IOCTL_BUF_LEN		ETHER_MAX_LEN

/* Ring counts */
#define BWFM_NUM_TX_MSGRINGS			2
#define BWFM_NUM_RX_MSGRINGS			3

/* Packet ID counts */
#define BWFM_NUM_IOCTL_PKTIDS			8
#define BWFM_NUM_TX_PKTIDS			2048
#define BWFM_NUM_RX_PKTIDS			1024

/* Chip IDs */
#define BRCM_CC_4350_CHIP_ID	0x4350
#define BRCM_CC_4355_CHIP_ID	0x4355
#define BRCM_CC_4356_CHIP_ID	0x4356
#define BRCM_CC_4364_CHIP_ID	0x4364
#define BRCM_CC_43602_CHIP_ID	43602
#define BRCM_CC_4371_CHIP_ID	0x4371
#define BRCM_CC_4377_CHIP_ID	0x4377
#define BRCM_CC_4378_CHIP_ID	0x4378
#define BRCM_CC_4387_CHIP_ID	0x4387

/* Ring status */
enum ring_status {
	RING_CLOSED,
	RING_CLOSING,
	RING_OPEN,
	RING_OPENING,
};

/* DMA memory descriptor */
struct bwfm_pci_dmamem {
	bus_dma_tag_t		bdm_tag;
	bus_dmamap_t		bdm_map;
	bus_dma_segment_t	bdm_seg;
	size_t			bdm_size;
	caddr_t			bdm_kva;
};

#define BWFM_PCI_DMA_LEN(_bdm)	((_bdm)->bdm_size)
#define BWFM_PCI_DMA_DVA(_bdm)	((uint64_t)(_bdm)->bdm_seg.ds_addr)
#define BWFM_PCI_DMA_KVA(_bdm)	((void *)(_bdm)->bdm_kva)

/* Message ring */
struct bwfm_pci_msgring {
	uint32_t		w_idx_addr;
	uint32_t		r_idx_addr;
	uint32_t		w_ptr;
	uint32_t		r_ptr;
	int			nitem;
	int			itemsz;
	enum ring_status	status;
	struct bwfm_pci_dmamem	*ring;
	struct mbuf		*m;

	int			fifo;
	uint8_t			mac[ETHER_ADDR_LEN];
};

/* PCI softc */
struct bwfm_pci_softc {
	struct bwfm_softc	sc_sc;
	device_t		sc_dev;

	struct resource		*sc_reg_res;
	int			sc_reg_rid;
	bus_space_tag_t		sc_reg_iot;
	bus_space_handle_t	sc_reg_ioh;

	struct resource		*sc_tcm_res;
	int			sc_tcm_rid;
	bus_space_tag_t		sc_tcm_iot;
	bus_space_handle_t	sc_tcm_ioh;

	struct resource		*sc_irq_res;
	int			sc_irq_rid;
	void			*sc_irq_hdl;

	bus_dma_tag_t		sc_dmat;

	int			sc_initialized;

	uint32_t		sc_shared_address;
	uint32_t		sc_shared_flags;
	uint8_t			sc_shared_version;

	uint8_t			sc_dma_idx_sz;
	struct bwfm_pci_dmamem	*sc_dma_idx_buf;
	size_t			sc_dma_idx_bufsz;

	uint16_t		sc_max_rxbufpost;
	uint32_t		sc_rx_dataoffset;
	uint32_t		sc_htod_mb_data_addr;
	uint32_t		sc_dtoh_mb_data_addr;
	uint32_t		sc_ring_info_addr;

	uint16_t		sc_max_flowrings;
	uint16_t		sc_max_submissionrings;
	uint16_t		sc_max_completionrings;

	struct bwfm_pci_msgring	sc_ctrl_submit;
	struct bwfm_pci_msgring	sc_rxpost_submit;
	struct bwfm_pci_msgring	sc_ctrl_complete;
	struct bwfm_pci_msgring	sc_tx_complete;
	struct bwfm_pci_msgring	sc_rx_complete;
	struct bwfm_pci_msgring	*sc_flowrings;

	struct bwfm_pci_dmamem	*sc_scratch_buf;
	struct bwfm_pci_dmamem	*sc_ringupd_buf;

	uint16_t		sc_ioctl_transid;

	uint8_t			sc_mbdata_done;
	uint8_t			sc_pcireg64;
	uint8_t			sc_mb_via_ctl;
};

static MALLOC_DEFINE(M_BWFM_PCI, "bwfm_pci", "Broadcom PCI WiFi driver");

/* Function prototypes */
static int	bwfm_pci_probe(device_t);
static int	bwfm_pci_attach(device_t);
static int	bwfm_pci_detach(device_t);

static int	bwfm_pci_intr(void *);

static int	bwfm_pci_preinit(struct bwfm_softc *);
static void	bwfm_pci_stop(struct bwfm_softc *);
static int	bwfm_pci_txcheck(struct bwfm_softc *);
static int	bwfm_pci_txdata(struct bwfm_softc *, struct mbuf *);

static int	bwfm_pci_msgbuf_query_dcmd(struct bwfm_softc *, int,
		    int, void *, size_t *);
static int	bwfm_pci_msgbuf_set_dcmd(struct bwfm_softc *, int,
		    int, void *, size_t);

/* Bus ops */
static struct bwfm_bus_ops bwfm_pci_bus_ops = {
	.bs_preinit = bwfm_pci_preinit,
	.bs_stop = bwfm_pci_stop,
	.bs_txcheck = bwfm_pci_txcheck,
	.bs_txdata = bwfm_pci_txdata,
	.bs_txctl = NULL,
	.bs_rxctl = NULL,
};

/* Protocol ops for PCI (uses msgbuf instead of BCDC) */
static struct bwfm_proto_ops bwfm_pci_msgbuf_ops = {
	.proto_query_dcmd = bwfm_pci_msgbuf_query_dcmd,
	.proto_set_dcmd = bwfm_pci_msgbuf_set_dcmd,
	.proto_rx = NULL,
	.proto_rxctl = NULL,
};

/* Device IDs */
static const struct {
	uint16_t	vendor;
	uint16_t	device;
	const char	*desc;
} bwfm_pci_devices[] = {
	{ 0x14e4, 0x4350, "Broadcom BCM4350 PCIe WiFi" },
	{ 0x14e4, 0x4356, "Broadcom BCM4356 PCIe WiFi" },
	{ 0x14e4, 0x43a3, "Broadcom BCM43602 PCIe WiFi" },
	{ 0x14e4, 0x4371, "Broadcom BCM4371 PCIe WiFi" },
	{ 0x14e4, 0x4378, "Broadcom BCM4378 PCIe WiFi" },
	{ 0x14e4, 0x4387, "Broadcom BCM4387 PCIe WiFi" },
	{ 0, 0, NULL }
};

static int
bwfm_pci_probe(device_t dev)
{
	uint16_t vendor, device;
	int i;

	vendor = pci_get_vendor(dev);
	device = pci_get_device(dev);

	for (i = 0; bwfm_pci_devices[i].vendor != 0; i++) {
		if (vendor == bwfm_pci_devices[i].vendor &&
		    device == bwfm_pci_devices[i].device) {
			device_set_desc(dev, bwfm_pci_devices[i].desc);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
bwfm_pci_attach(device_t dev)
{
	struct bwfm_pci_softc *sc;
	struct bwfm_softc *bwfm;
	int error;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	bwfm = &sc->sc_sc;
	bwfm->sc_dev = dev;

	/* Enable bus mastering */
	pci_enable_busmaster(dev);

	/* Map BAR0 (registers) */
	sc->sc_reg_rid = PCIR_BAR(0);
	sc->sc_reg_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->sc_reg_rid, RF_ACTIVE);
	if (sc->sc_reg_res == NULL) {
		device_printf(dev, "cannot map BAR0\n");
		return (ENXIO);
	}
	sc->sc_reg_iot = rman_get_bustag(sc->sc_reg_res);
	sc->sc_reg_ioh = rman_get_bushandle(sc->sc_reg_res);

	/* Map BAR1 (TCM - Tightly Coupled Memory) */
	sc->sc_tcm_rid = PCIR_BAR(2);
	sc->sc_tcm_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->sc_tcm_rid, RF_ACTIVE);
	if (sc->sc_tcm_res == NULL) {
		device_printf(dev, "cannot map BAR1 (TCM)\n");
		error = ENXIO;
		goto fail_bar0;
	}
	sc->sc_tcm_iot = rman_get_bustag(sc->sc_tcm_res);
	sc->sc_tcm_ioh = rman_get_bushandle(sc->sc_tcm_res);

	/* Allocate interrupt */
	sc->sc_irq_rid = 0;
	if (pci_alloc_msi(dev, &sc->sc_irq_rid) == 0)
		sc->sc_irq_rid = 1;
	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->sc_irq_rid, RF_ACTIVE | RF_SHAREABLE);
	if (sc->sc_irq_res == NULL) {
		device_printf(dev, "cannot allocate interrupt\n");
		error = ENXIO;
		goto fail_bar1;
	}

	error = bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, bwfm_pci_intr, sc, &sc->sc_irq_hdl);
	if (error != 0) {
		device_printf(dev, "cannot setup interrupt\n");
		goto fail_irq;
	}

	/* Create DMA tag */
	error = bus_dma_tag_create(bus_get_dma_tag(dev),
	    1, 0,
	    BUS_SPACE_MAXADDR_32BIT,
	    BUS_SPACE_MAXADDR,
	    NULL, NULL,
	    MSGBUF_MAX_CTL_PKT_SIZE,
	    1,
	    MSGBUF_MAX_CTL_PKT_SIZE,
	    0, NULL, NULL,
	    &sc->sc_dmat);
	if (error != 0) {
		device_printf(dev, "cannot create DMA tag\n");
		goto fail_intr;
	}

	/* Set bus operations */
	bwfm->sc_bus_ops = &bwfm_pci_bus_ops;
	bwfm->sc_proto_ops = &bwfm_pci_msgbuf_ops;

	/* TODO: Initialize chip, load firmware, setup rings */
	device_printf(dev,
	    "PCI driver is a skeleton - msgbuf protocol not yet implemented\n");

	/* Attach to ieee80211 stack (will fail until preinit works) */
	/* bwfm_attach(bwfm); */

	return (0);

fail_intr:
	bus_teardown_intr(dev, sc->sc_irq_res, sc->sc_irq_hdl);
fail_irq:
	bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid, sc->sc_irq_res);
	pci_release_msi(dev);
fail_bar1:
	bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_tcm_rid, sc->sc_tcm_res);
fail_bar0:
	bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_reg_rid, sc->sc_reg_res);
	return (error);
}

static int
bwfm_pci_detach(device_t dev)
{
	struct bwfm_pci_softc *sc = device_get_softc(dev);
	struct bwfm_softc *bwfm = &sc->sc_sc;

	if (sc->sc_initialized)
		bwfm_detach(bwfm);

	if (sc->sc_dmat != NULL)
		bus_dma_tag_destroy(sc->sc_dmat);

	if (sc->sc_irq_hdl != NULL)
		bus_teardown_intr(dev, sc->sc_irq_res, sc->sc_irq_hdl);

	if (sc->sc_irq_res != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		    sc->sc_irq_res);
		pci_release_msi(dev);
	}

	if (sc->sc_tcm_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_tcm_rid,
		    sc->sc_tcm_res);

	if (sc->sc_reg_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_reg_rid,
		    sc->sc_reg_res);

	return (0);
}

static int
bwfm_pci_intr(void *arg)
{
	struct bwfm_pci_softc *sc = arg;

	(void)sc;
	/* TODO: Handle interrupt, process completion rings */
	return (FILTER_HANDLED);
}

static int
bwfm_pci_preinit(struct bwfm_softc *bwfm)
{
	struct bwfm_pci_softc *sc = (struct bwfm_pci_softc *)bwfm;

	(void)sc;

	/* TODO: Initialize chip, load firmware, setup message rings */
	device_printf(sc->sc_dev,
	    "preinit not implemented (msgbuf protocol required)\n");

	return (ENOTSUP);
}

static void
bwfm_pci_stop(struct bwfm_softc *bwfm)
{
	struct bwfm_pci_softc *sc = (struct bwfm_pci_softc *)bwfm;

	(void)sc;
	/* TODO: Stop rings, disable interrupts */
}

static int
bwfm_pci_txcheck(struct bwfm_softc *bwfm)
{
	struct bwfm_pci_softc *sc = (struct bwfm_pci_softc *)bwfm;

	(void)sc;
	/* TODO: Check if flow rings have space */
	return (ENOTSUP);
}

static int
bwfm_pci_txdata(struct bwfm_softc *bwfm, struct mbuf *m)
{
	struct bwfm_pci_softc *sc = (struct bwfm_pci_softc *)bwfm;

	(void)sc;
	/* TODO: Submit TX via flow ring */
	m_freem(m);
	return (ENOTSUP);
}

static int
bwfm_pci_msgbuf_query_dcmd(struct bwfm_softc *bwfm, int ifidx,
    int cmd, void *buf, size_t *len)
{
	struct bwfm_pci_softc *sc = (struct bwfm_pci_softc *)bwfm;

	(void)sc;
	(void)ifidx;
	(void)cmd;
	(void)buf;
	(void)len;

	/* TODO: Send IOCTL via control ring, wait for response */
	return (ENOTSUP);
}

static int
bwfm_pci_msgbuf_set_dcmd(struct bwfm_softc *bwfm, int ifidx,
    int cmd, void *buf, size_t len)
{
	struct bwfm_pci_softc *sc = (struct bwfm_pci_softc *)bwfm;

	(void)sc;
	(void)ifidx;
	(void)cmd;
	(void)buf;
	(void)len;

	/* TODO: Send IOCTL via control ring */
	return (ENOTSUP);
}

static device_method_t bwfm_pci_methods[] = {
	DEVMETHOD(device_probe,		bwfm_pci_probe),
	DEVMETHOD(device_attach,	bwfm_pci_attach),
	DEVMETHOD(device_detach,	bwfm_pci_detach),
	DEVMETHOD_END
};

static driver_t bwfm_pci_driver = {
	"bwfm",
	bwfm_pci_methods,
	sizeof(struct bwfm_pci_softc)
};

DRIVER_MODULE(bwfm_pci, pci, bwfm_pci_driver, NULL, NULL);
MODULE_VERSION(bwfm_pci, 1);
MODULE_DEPEND(bwfm_pci, bwfm, 1, 1, 1);
MODULE_DEPEND(bwfm_pci, pci, 1, 1, 1);
MODULE_DEPEND(bwfm_pci, wlan, 1, 1, 1);
MODULE_DEPEND(bwfm_pci, firmware, 1, 1, 1);
MODULE_PNP_INFO("U16:vendor;U16:device", pci, bwfm_pci, bwfm_pci_devices,
    nitems(bwfm_pci_devices) - 1);
