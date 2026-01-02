/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2010-2016 Broadcom Corporation
 * Copyright (c) 2016,2017 Patrick Wildt <patrick@blueri.se>
 * Copyright (c) 2024 The FreeBSD Foundation
 *
 * Portions of this software were developed with reference to OpenBSD's
 * if_bwfm_usb.c driver.
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
 * Broadcom FullMAC USB WiFi driver
 *
 * This driver supports USB-attached Broadcom WiFi chips including:
 * - BCM43143
 * - BCM43236/43238
 * - BCM43242
 * - BCM43566/43569
 */

#include <sys/cdefs.h>
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/condvar.h>
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
#include <net/if_media.h>
#include <net/ethernet.h>

#include <net80211/ieee80211_var.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdevs.h>

#include <dev/bwfm/bwfmreg.h>
#include <dev/bwfm/bwfmvar.h>

#ifdef BWFM_DEBUG
#define DPRINTF(x)	do { printf x; } while (0)
#define DPRINTFN(n, x)	do { printf x; } while (0)
#else
#define DPRINTF(x)	do { ; } while (0)
#define DPRINTFN(n, x)	do { ; } while (0)
#endif

/* USB device IDs */
static const STRUCT_USB_HOST_ID bwfm_usbdevs[] = {
	{ USB_VPI(USB_VENDOR_BROADCOM, 0x43143, 0) },	/* BCM43143 */
	{ USB_VPI(USB_VENDOR_BROADCOM, 0x43236, 0) },	/* BCM43236 */
	{ USB_VPI(USB_VENDOR_BROADCOM, 0x43242, 0) },	/* BCM43242 */
	{ USB_VPI(USB_VENDOR_BROADCOM, 0x43566, 0) },	/* BCM43566 */
	{ USB_VPI(USB_VENDOR_BROADCOM, 0x43569, 0) },	/* BCM43569 */
	{ USB_VPI(USB_VENDOR_BROADCOM, 0xbd17, 0) },	/* BCM43236 (firmware) */
};

#define BRCMF_POSTBOOT_ID	0xA123	/* ID to detect if dongle has booted */

/* TRX firmware header */
#define TRX_MAGIC		0x30524448	/* "HDR0" */
#define TRX_MAX_OFFSET		3
#define TRX_UNCOMP_IMAGE	0x20
#define TRX_RDL_CHUNK		1500

/* Control messages: bRequest values */
#define DL_GETSTATE	0
#define DL_CHECK_CRC	1
#define DL_GO		2
#define DL_START	3
#define DL_REBOOT	4
#define DL_GETVER	5
#define DL_GO_PROTECTED	6
#define DL_EXEC		7
#define DL_RESETCFG	8

/* States */
#define DL_WAITING	0
#define DL_READY	1
#define DL_BAD_HDR	2
#define DL_BAD_CRC	3
#define DL_RUNNABLE	4

struct trx_header {
	uint32_t	magic;
	uint32_t	len;
	uint32_t	crc32;
	uint32_t	flag_version;
	uint32_t	offsets[TRX_MAX_OFFSET];
};

struct rdl_state {
	uint32_t	state;
	uint32_t	bytes;
};

struct bootrom_id {
	uint32_t	chip;
	uint32_t	chiprev;
	uint32_t	ramsize;
	uint32_t	remapbase;
	uint32_t	boardtype;
	uint32_t	boardrev;
};

/* Chip IDs (Broadcom) */
#define BRCM_CC_43143_CHIP_ID	43143
#define BRCM_CC_43235_CHIP_ID	43235
#define BRCM_CC_43236_CHIP_ID	43236
#define BRCM_CC_43238_CHIP_ID	43238
#define BRCM_CC_43242_CHIP_ID	43242
#define BRCM_CC_43566_CHIP_ID	43566
#define BRCM_CC_43569_CHIP_ID	43569

/* USB transfer buffers */
#define BWFM_RX_LIST_COUNT	50
#define BWFM_TX_LIST_COUNT	50
#define BWFM_RXBUFSZ		1600
#define BWFM_TXBUFSZ		1600

/* USB endpoint indices */
enum {
	BWFM_USB_BULK_RX,
	BWFM_USB_BULK_TX,
	BWFM_USB_N_TRANSFER
};

struct bwfm_usb_rx_data {
	struct bwfm_usb_softc	*sc;
	uint8_t			*buf;
};

struct bwfm_usb_tx_data {
	struct bwfm_usb_softc	*sc;
	struct mbuf		*mbuf;
	uint8_t			*buf;
	STAILQ_ENTRY(bwfm_usb_tx_data) next;
};

struct bwfm_usb_softc {
	struct bwfm_softc	sc_sc;
	device_t		sc_dev;
	struct usb_device	*sc_udev;
	struct usb_xfer		*sc_xfer[BWFM_USB_N_TRANSFER];

	int			sc_initialized;

	uint32_t		sc_chip;
	uint32_t		sc_chiprev;

	struct bwfm_usb_rx_data	sc_rx_data[BWFM_RX_LIST_COUNT];
	struct bwfm_usb_tx_data	sc_tx_data[BWFM_TX_LIST_COUNT];
	STAILQ_HEAD(, bwfm_usb_tx_data) sc_tx_free_list;
	STAILQ_HEAD(, bwfm_usb_tx_data) sc_tx_pending_list;

	struct mtx		sc_usb_mtx;

	/* Control transfer state */
	char			*sc_ctl_buf;
	size_t			sc_ctl_len;
	int			sc_ctl_done;
	struct cv		sc_ctl_cv;
};

static MALLOC_DEFINE(M_BWFM_USB, "bwfm_usb", "Broadcom USB WiFi driver");

/* Function prototypes */
static device_probe_t	bwfm_usb_match;
static device_attach_t	bwfm_usb_attach;
static device_detach_t	bwfm_usb_detach;

static usb_callback_t	bwfm_usb_bulk_rx_callback;
static usb_callback_t	bwfm_usb_bulk_tx_callback;

static int	bwfm_usb_dl_cmd(struct bwfm_usb_softc *, uint8_t, void *, int);
static int	bwfm_usb_load_microcode(struct bwfm_usb_softc *,
		    const uint8_t *, size_t);

static int	bwfm_usb_alloc_rx_list(struct bwfm_usb_softc *);
static void	bwfm_usb_free_rx_list(struct bwfm_usb_softc *);
static int	bwfm_usb_alloc_tx_list(struct bwfm_usb_softc *);
static void	bwfm_usb_free_tx_list(struct bwfm_usb_softc *);

static int	bwfm_usb_preinit(struct bwfm_softc *);
static void	bwfm_usb_stop(struct bwfm_softc *);
static int	bwfm_usb_txcheck(struct bwfm_softc *);
static int	bwfm_usb_txdata(struct bwfm_softc *, struct mbuf *);
static int	bwfm_usb_txctl(struct bwfm_softc *, void *, size_t);
static int	bwfm_usb_rxctl(struct bwfm_softc *, void *, size_t *);

/* USB transfer configuration */
static const struct usb_config bwfm_usb_config[BWFM_USB_N_TRANSFER] = {
	[BWFM_USB_BULK_RX] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.bufsize = BWFM_RXBUFSZ,
		.flags = { .pipe_bof = 1, .short_xfer_ok = 1 },
		.callback = bwfm_usb_bulk_rx_callback,
	},
	[BWFM_USB_BULK_TX] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.bufsize = BWFM_TXBUFSZ,
		.flags = { .pipe_bof = 1, .force_short_xfer = 1 },
		.callback = bwfm_usb_bulk_tx_callback,
	},
};

/* Bus operations */
static struct bwfm_bus_ops bwfm_usb_bus_ops = {
	.bs_preinit = bwfm_usb_preinit,
	.bs_stop = bwfm_usb_stop,
	.bs_txcheck = bwfm_usb_txcheck,
	.bs_txdata = bwfm_usb_txdata,
	.bs_txctl = bwfm_usb_txctl,
	.bs_rxctl = bwfm_usb_rxctl,
};

static int
bwfm_usb_match(device_t self)
{
	struct usb_attach_arg *uaa = device_get_ivars(self);

	if (uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);
	if (uaa->info.bConfigIndex != 0)
		return (ENXIO);
	if (uaa->info.bIfaceIndex != 0)
		return (ENXIO);

	return (usbd_lookup_id_by_uaa(bwfm_usbdevs, sizeof(bwfm_usbdevs), uaa));
}

static int
bwfm_usb_attach(device_t self)
{
	struct bwfm_usb_softc *sc = device_get_softc(self);
	struct usb_attach_arg *uaa = device_get_ivars(self);
	struct bwfm_softc *bwfm = &sc->sc_sc;
	int error;

	device_set_usb_desc(self);
	sc->sc_dev = self;
	sc->sc_udev = uaa->device;
	bwfm->sc_dev = self;

	mtx_init(&sc->sc_usb_mtx, "bwfm_usb", NULL, MTX_DEF);
	cv_init(&sc->sc_ctl_cv, "bwfm_ctl");

	STAILQ_INIT(&sc->sc_tx_free_list);
	STAILQ_INIT(&sc->sc_tx_pending_list);

	/* Setup USB transfers */
	error = usbd_transfer_setup(sc->sc_udev, &uaa->info.bIfaceIndex,
	    sc->sc_xfer, bwfm_usb_config, BWFM_USB_N_TRANSFER, sc,
	    &sc->sc_usb_mtx);
	if (error != 0) {
		device_printf(self, "could not setup USB transfers: %s\n",
		    usbd_errstr(error));
		goto fail;
	}

	/* Set bus operations */
	bwfm->sc_bus_ops = &bwfm_usb_bus_ops;
	bwfm->sc_proto_ops = &bwfm_proto_bcdc_ops;

	/* Attach to ieee80211 stack */
	bwfm_attach(bwfm);

	return (0);

fail:
	cv_destroy(&sc->sc_ctl_cv);
	mtx_destroy(&sc->sc_usb_mtx);
	return (error);
}

static int
bwfm_usb_detach(device_t self)
{
	struct bwfm_usb_softc *sc = device_get_softc(self);
	struct bwfm_softc *bwfm = &sc->sc_sc;

	/* Stop USB transfers */
	usbd_transfer_unsetup(sc->sc_xfer, BWFM_USB_N_TRANSFER);

	/* Detach from ieee80211 */
	bwfm_detach(bwfm);

	/* Free resources */
	bwfm_usb_free_rx_list(sc);
	bwfm_usb_free_tx_list(sc);

	cv_destroy(&sc->sc_ctl_cv);
	mtx_destroy(&sc->sc_usb_mtx);

	return (0);
}

static int
bwfm_usb_preinit(struct bwfm_softc *bwfm)
{
	struct bwfm_usb_softc *sc = (struct bwfm_usb_softc *)bwfm;
	const struct firmware *fw;
	struct bootrom_id brom;
	const char *fwname = NULL;
	int error;

	if (sc->sc_initialized)
		return (0);

	/* Read chip info */
	memset(&brom, 0, sizeof(brom));
	error = bwfm_usb_dl_cmd(sc, DL_GETVER, &brom, sizeof(brom));
	if (error != 0) {
		device_printf(sc->sc_dev, "failed to get chip version\n");
		return (error);
	}
	sc->sc_chip = le32toh(brom.chip);
	sc->sc_chiprev = le32toh(brom.chiprev);

	DPRINTF(("%s: chip=%u chiprev=%u\n", device_get_nameunit(sc->sc_dev),
	    sc->sc_chip, sc->sc_chiprev));

	/* Check if firmware is already loaded */
	if (sc->sc_chip != BRCMF_POSTBOOT_ID) {
		/* Select firmware based on chip */
		switch (sc->sc_chip) {
		case BRCM_CC_43143_CHIP_ID:
			fwname = "brcmfmac43143.bin";
			break;
		case BRCM_CC_43235_CHIP_ID:
		case BRCM_CC_43236_CHIP_ID:
		case BRCM_CC_43238_CHIP_ID:
			if (sc->sc_chiprev == 3)
				fwname = "brcmfmac43236b.bin";
			break;
		case BRCM_CC_43242_CHIP_ID:
			fwname = "brcmfmac43242a.bin";
			break;
		case BRCM_CC_43566_CHIP_ID:
		case BRCM_CC_43569_CHIP_ID:
			fwname = "brcmfmac43569.bin";
			break;
		}

		if (fwname == NULL) {
			device_printf(sc->sc_dev,
			    "no firmware for chip %u rev %u\n",
			    sc->sc_chip, sc->sc_chiprev);
			return (ENOENT);
		}

		/* Load firmware */
		fw = firmware_get(fwname);
		if (fw == NULL) {
			device_printf(sc->sc_dev,
			    "failed to load firmware: %s\n", fwname);
			return (ENOENT);
		}

		error = bwfm_usb_load_microcode(sc, fw->data, fw->datasize);
		firmware_put(fw, FIRMWARE_UNLOAD);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "failed to load microcode\n");
			return (error);
		}

		/* Wait for firmware to start */
		int i;
		for (i = 0; i < 10; i++) {
			DELAY(100000);
			memset(&brom, 0, sizeof(brom));
			bwfm_usb_dl_cmd(sc, DL_GETVER, &brom, sizeof(brom));
			if (le32toh(brom.chip) == BRCMF_POSTBOOT_ID)
				break;
		}

		if (le32toh(brom.chip) != BRCMF_POSTBOOT_ID) {
			device_printf(sc->sc_dev,
			    "firmware did not start\n");
			return (ETIMEDOUT);
		}

		sc->sc_chip = le32toh(brom.chip);
		sc->sc_chiprev = le32toh(brom.chiprev);
	}

	/* Reset configuration */
	bwfm_usb_dl_cmd(sc, DL_RESETCFG, &brom, sizeof(brom));

	/* Allocate RX/TX buffers */
	error = bwfm_usb_alloc_rx_list(sc);
	if (error != 0) {
		device_printf(sc->sc_dev, "failed to alloc rx list\n");
		return (error);
	}
	error = bwfm_usb_alloc_tx_list(sc);
	if (error != 0) {
		device_printf(sc->sc_dev, "failed to alloc tx list\n");
		bwfm_usb_free_rx_list(sc);
		return (error);
	}

	/* Start RX transfers */
	mtx_lock(&sc->sc_usb_mtx);
	usbd_transfer_start(sc->sc_xfer[BWFM_USB_BULK_RX]);
	mtx_unlock(&sc->sc_usb_mtx);

	sc->sc_initialized = 1;
	return (0);
}

static void
bwfm_usb_stop(struct bwfm_softc *bwfm)
{
	struct bwfm_usb_softc *sc = (struct bwfm_usb_softc *)bwfm;

	mtx_lock(&sc->sc_usb_mtx);
	usbd_transfer_stop(sc->sc_xfer[BWFM_USB_BULK_RX]);
	usbd_transfer_stop(sc->sc_xfer[BWFM_USB_BULK_TX]);
	mtx_unlock(&sc->sc_usb_mtx);
}

static int
bwfm_usb_alloc_rx_list(struct bwfm_usb_softc *sc)
{
	struct bwfm_usb_rx_data *data;
	int i;

	for (i = 0; i < BWFM_RX_LIST_COUNT; i++) {
		data = &sc->sc_rx_data[i];
		data->sc = sc;
		data->buf = malloc(BWFM_RXBUFSZ, M_BWFM_USB, M_NOWAIT);
		if (data->buf == NULL) {
			device_printf(sc->sc_dev,
			    "failed to alloc rx buffer %d\n", i);
			bwfm_usb_free_rx_list(sc);
			return (ENOMEM);
		}
	}
	return (0);
}

static void
bwfm_usb_free_rx_list(struct bwfm_usb_softc *sc)
{
	int i;

	for (i = 0; i < BWFM_RX_LIST_COUNT; i++) {
		if (sc->sc_rx_data[i].buf != NULL) {
			free(sc->sc_rx_data[i].buf, M_BWFM_USB);
			sc->sc_rx_data[i].buf = NULL;
		}
	}
}

static int
bwfm_usb_alloc_tx_list(struct bwfm_usb_softc *sc)
{
	struct bwfm_usb_tx_data *data;
	int i;

	for (i = 0; i < BWFM_TX_LIST_COUNT; i++) {
		data = &sc->sc_tx_data[i];
		data->sc = sc;
		data->buf = malloc(BWFM_TXBUFSZ, M_BWFM_USB, M_NOWAIT);
		if (data->buf == NULL) {
			device_printf(sc->sc_dev,
			    "failed to alloc tx buffer %d\n", i);
			bwfm_usb_free_tx_list(sc);
			return (ENOMEM);
		}
		STAILQ_INSERT_TAIL(&sc->sc_tx_free_list, data, next);
	}
	return (0);
}

static void
bwfm_usb_free_tx_list(struct bwfm_usb_softc *sc)
{
	struct bwfm_usb_tx_data *data;
	int i;

	for (i = 0; i < BWFM_TX_LIST_COUNT; i++) {
		data = &sc->sc_tx_data[i];
		if (data->mbuf != NULL) {
			m_freem(data->mbuf);
			data->mbuf = NULL;
		}
		if (data->buf != NULL) {
			free(data->buf, M_BWFM_USB);
			data->buf = NULL;
		}
	}
	STAILQ_INIT(&sc->sc_tx_free_list);
	STAILQ_INIT(&sc->sc_tx_pending_list);
}

static void
bwfm_usb_bulk_rx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct bwfm_usb_softc *sc = usbd_xfer_softc(xfer);
	struct bwfm_softc *bwfm = &sc->sc_sc;
	struct usb_page_cache *pc;
	struct mbuf_list ml;
	struct mbuf *m;
	int actlen;

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		DPRINTFN(2, ("%s: rx %d bytes\n",
		    device_get_nameunit(sc->sc_dev), actlen));

		/* Allocate mbuf */
		m = m_get2(actlen, M_NOWAIT, MT_DATA, M_PKTHDR);
		if (m == NULL) {
			device_printf(sc->sc_dev, "failed to alloc mbuf\n");
			goto submit;
		}

		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_out(pc, 0, mtod(m, void *), actlen);
		m->m_len = m->m_pkthdr.len = actlen;

		/* Process received frame */
		ml_init(&ml);
		mtx_unlock(&sc->sc_usb_mtx);
		bwfm->sc_proto_ops->proto_rx(bwfm, m, &ml);
		mtx_lock(&sc->sc_usb_mtx);

		/* FALLTHROUGH */
	case USB_ST_SETUP:
submit:
		usbd_xfer_set_frame_len(xfer, 0, BWFM_RXBUFSZ);
		usbd_transfer_submit(xfer);
		break;

	default:
		if (error != USB_ERR_CANCELLED) {
			usbd_xfer_set_stall(xfer);
			goto submit;
		}
		break;
	}
}

static void
bwfm_usb_bulk_tx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct bwfm_usb_softc *sc = usbd_xfer_softc(xfer);
	struct bwfm_usb_tx_data *data;
	struct usb_page_cache *pc;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		DPRINTFN(2, ("%s: tx complete\n",
		    device_get_nameunit(sc->sc_dev)));

		data = STAILQ_FIRST(&sc->sc_tx_pending_list);
		if (data != NULL) {
			STAILQ_REMOVE_HEAD(&sc->sc_tx_pending_list, next);
			if (data->mbuf != NULL) {
				m_freem(data->mbuf);
				data->mbuf = NULL;
			}
			STAILQ_INSERT_TAIL(&sc->sc_tx_free_list, data, next);
		}
		/* FALLTHROUGH */
	case USB_ST_SETUP:
		data = STAILQ_FIRST(&sc->sc_tx_pending_list);
		if (data == NULL)
			break;

		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_in(pc, 0, data->buf, BWFM_TXBUFSZ);
		usbd_xfer_set_frame_len(xfer, 0, BWFM_TXBUFSZ);
		usbd_transfer_submit(xfer);
		break;

	default:
		if (error != USB_ERR_CANCELLED) {
			usbd_xfer_set_stall(xfer);
			/* Return buffer to free list */
			data = STAILQ_FIRST(&sc->sc_tx_pending_list);
			if (data != NULL) {
				STAILQ_REMOVE_HEAD(&sc->sc_tx_pending_list, next);
				if (data->mbuf != NULL) {
					m_freem(data->mbuf);
					data->mbuf = NULL;
				}
				STAILQ_INSERT_TAIL(&sc->sc_tx_free_list, data, next);
			}
		}
		break;
	}
}

static int
bwfm_usb_dl_cmd(struct bwfm_usb_softc *sc, uint8_t cmd, void *buf, int len)
{
	struct usb_device_request req;
	usb_error_t error;

	req.bmRequestType = UT_READ_VENDOR_INTERFACE;
	req.bRequest = cmd;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, len);

	error = usbd_do_request(sc->sc_udev, NULL, &req, buf);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "USB control request failed: %s\n", usbd_errstr(error));
		return (EIO);
	}
	return (0);
}

static int
bwfm_usb_load_microcode(struct bwfm_usb_softc *sc,
    const uint8_t *ucode, size_t size)
{
	const struct trx_header *trx = (const struct trx_header *)ucode;
	struct rdl_state state;
	struct usb_device_request req;
	uint32_t rdlstate, rdlbytes;
	size_t sent = 0, sendlen;
	usb_error_t error;

	if (le32toh(trx->magic) != TRX_MAGIC ||
	    (le32toh(trx->flag_version) & TRX_UNCOMP_IMAGE) == 0) {
		device_printf(sc->sc_dev, "invalid firmware format\n");
		return (EINVAL);
	}

	/* Start download */
	bwfm_usb_dl_cmd(sc, DL_START, &state, sizeof(state));
	rdlstate = le32toh(state.state);
	if (rdlstate != DL_WAITING) {
		device_printf(sc->sc_dev, "cannot start firmware download\n");
		return (EIO);
	}

	/* Download firmware in chunks */
	while (sent < size) {
		sendlen = MIN(size - sent, TRX_RDL_CHUNK);

		req.bmRequestType = UT_WRITE_VENDOR_INTERFACE;
		req.bRequest = 0;
		USETW(req.wValue, 0);
		USETW(req.wIndex, 0);
		USETW(req.wLength, sendlen);

		error = usbd_do_request(sc->sc_udev, NULL, &req,
		    __DECONST(void *, ucode + sent));
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "firmware download failed: %s\n",
			    usbd_errstr(error));
			return (EIO);
		}
		sent += sendlen;

		/* Check state */
		bwfm_usb_dl_cmd(sc, DL_GETSTATE, &state, sizeof(state));
		rdlstate = le32toh(state.state);
		rdlbytes = le32toh(state.bytes);

		if (rdlbytes != sent) {
			device_printf(sc->sc_dev,
			    "firmware download size mismatch\n");
			return (EIO);
		}

		if (rdlstate == DL_BAD_HDR || rdlstate == DL_BAD_CRC) {
			device_printf(sc->sc_dev,
			    "firmware download error: bad hdr/crc\n");
			return (EIO);
		}
	}

	/* Verify download is complete */
	bwfm_usb_dl_cmd(sc, DL_GETSTATE, &state, sizeof(state));
	rdlstate = le32toh(state.state);
	if (rdlstate != DL_RUNNABLE) {
		device_printf(sc->sc_dev, "firmware not runnable\n");
		return (EIO);
	}

	/* Execute firmware */
	bwfm_usb_dl_cmd(sc, DL_GO, &state, sizeof(state));

	return (0);
}

static int
bwfm_usb_txcheck(struct bwfm_softc *bwfm)
{
	struct bwfm_usb_softc *sc = (struct bwfm_usb_softc *)bwfm;

	if (STAILQ_EMPTY(&sc->sc_tx_free_list))
		return (ENOBUFS);

	return (0);
}

static int
bwfm_usb_txdata(struct bwfm_softc *bwfm, struct mbuf *m)
{
	struct bwfm_usb_softc *sc = (struct bwfm_usb_softc *)bwfm;
	struct bwfm_proto_bcdc_hdr *hdr;
	struct bwfm_usb_tx_data *data;
	uint32_t len = 0;

	DPRINTFN(2, ("%s: txdata\n", device_get_nameunit(sc->sc_dev)));

	mtx_lock(&sc->sc_usb_mtx);

	if (STAILQ_EMPTY(&sc->sc_tx_free_list)) {
		mtx_unlock(&sc->sc_usb_mtx);
		m_freem(m);
		return (ENOBUFS);
	}

	/* Get a TX buffer */
	data = STAILQ_FIRST(&sc->sc_tx_free_list);
	STAILQ_REMOVE_HEAD(&sc->sc_tx_free_list, next);

	/* Build BCDC header */
	hdr = (struct bwfm_proto_bcdc_hdr *)&data->buf[len];
	hdr->data_offset = 0;
	hdr->priority = 0;	/* TODO: proper priority classification */
	hdr->flags = BWFM_BCDC_FLAG_VER(BWFM_BCDC_FLAG_PROTO_VER);
	hdr->flags2 = 0;
	len += sizeof(*hdr);

	/* Copy mbuf data */
	m_copydata(m, 0, m->m_pkthdr.len, (caddr_t)&data->buf[len]);
	len += m->m_pkthdr.len;

	data->mbuf = m;

	/* Queue for transmission */
	STAILQ_INSERT_TAIL(&sc->sc_tx_pending_list, data, next);
	usbd_transfer_start(sc->sc_xfer[BWFM_USB_BULK_TX]);

	mtx_unlock(&sc->sc_usb_mtx);
	return (0);
}

static int
bwfm_usb_txctl(struct bwfm_softc *bwfm, void *buf, size_t len)
{
	struct bwfm_usb_softc *sc = (struct bwfm_usb_softc *)bwfm;
	struct usb_device_request req;
	usb_error_t error;

	DPRINTFN(2, ("%s: txctl len=%zu\n", device_get_nameunit(sc->sc_dev), len));

	/* Send control packet */
	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = 0;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, len);

	error = usbd_do_request(sc->sc_udev, NULL, &req, buf);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "control write failed: %s\n", usbd_errstr(error));
		return (EIO);
	}

	return (0);
}

static int
bwfm_usb_rxctl(struct bwfm_softc *bwfm, void *buf, size_t *len)
{
	struct bwfm_usb_softc *sc = (struct bwfm_usb_softc *)bwfm;
	struct usb_device_request req;
	usb_error_t error;

	DPRINTFN(2, ("%s: rxctl len=%zu\n", device_get_nameunit(sc->sc_dev), *len));

	/* Receive control response */
	req.bmRequestType = UT_READ_CLASS_INTERFACE;
	req.bRequest = 1;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, *len);

	error = usbd_do_request(sc->sc_udev, NULL, &req, buf);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "control read failed: %s\n", usbd_errstr(error));
		return (EIO);
	}

	return (0);
}

static device_method_t bwfm_usb_methods[] = {
	DEVMETHOD(device_probe,		bwfm_usb_match),
	DEVMETHOD(device_attach,	bwfm_usb_attach),
	DEVMETHOD(device_detach,	bwfm_usb_detach),
	DEVMETHOD_END
};

static driver_t bwfm_usb_driver = {
	"bwfm",
	bwfm_usb_methods,
	sizeof(struct bwfm_usb_softc)
};

DRIVER_MODULE(bwfm_usb, uhub, bwfm_usb_driver, NULL, NULL);
MODULE_VERSION(bwfm_usb, 1);
MODULE_DEPEND(bwfm_usb, bwfm, 1, 1, 1);
MODULE_DEPEND(bwfm_usb, usb, 1, 1, 1);
MODULE_DEPEND(bwfm_usb, wlan, 1, 1, 1);
MODULE_DEPEND(bwfm_usb, firmware, 1, 1, 1);
USB_PNP_HOST_INFO(bwfm_usbdevs);
