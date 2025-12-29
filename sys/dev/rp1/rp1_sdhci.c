/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 The FreeBSD Project
 *
 * Raspberry Pi 5 RP1 SDHCI Controller Driver
 *
 * The RP1 contains a DesignWare Mobile Storage Host Controller (dwcmshc)
 * accessed via the RP1 PCIe bus. This driver provides SDHCI support for
 * both SD card and SDIO (WiFi) functionality.
 *
 * Based on sdhci_fdt_rockchip.c and Linux sdhci-of-dwcmshc.c
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/mmc/bridge.h>

#include <dev/sdhci/sdhci.h>

#include "rp1_var.h"

#include "mmcbr_if.h"
#include "sdhci_if.h"

/*
 * RP1 SDHCI register offsets within each SDIO block
 * SDIO0: 0x180000 (SD card slot)
 * SDIO1: 0x184000 (SDIO for WiFi)
 */
#define	RP1_SDHCI_SIZE		0x100

/*
 * RP1-specific quirks (from Linux sdhci-of-dwcmshc.c)
 * The RP1 dwcmshc is simpler than Rockchip - no DLL configuration needed
 */
#define	RP1_SDHCI_QUIRKS	(SDHCI_QUIRK_BROKEN_CARD_DETECTION | \
				 SDHCI_QUIRK_MISSING_CAPS)
#define	RP1_SDHCI_QUIRKS2	(SDHCI_QUIRK_PRESET_VALUE_BROKEN)

struct rp1_sdhci_softc {
	device_t		dev;
	struct resource		*mem_res;
	struct resource		*irq_res;
	void			*irq_ih;
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	bus_size_t		reg_off;	/* Offset within RP1 BAR */
	int			slot_id;	/* 0 = SD, 1 = WiFi */
	struct sdhci_slot	slot;
};

static uint8_t
rp1_sdhci_read_1(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct rp1_sdhci_softc *sc = device_get_softc(dev);

	return (bus_space_read_1(sc->bst, sc->bsh, sc->reg_off + off));
}

static uint16_t
rp1_sdhci_read_2(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct rp1_sdhci_softc *sc = device_get_softc(dev);

	return (bus_space_read_2(sc->bst, sc->bsh, sc->reg_off + off));
}

static uint32_t
rp1_sdhci_read_4(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct rp1_sdhci_softc *sc = device_get_softc(dev);

	return (bus_space_read_4(sc->bst, sc->bsh, sc->reg_off + off));
}

static void
rp1_sdhci_read_multi_4(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint32_t *data, bus_size_t count)
{
	struct rp1_sdhci_softc *sc = device_get_softc(dev);

	bus_space_read_multi_4(sc->bst, sc->bsh, sc->reg_off + off, data, count);
}

static void
rp1_sdhci_write_1(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint8_t val)
{
	struct rp1_sdhci_softc *sc = device_get_softc(dev);

	bus_space_write_1(sc->bst, sc->bsh, sc->reg_off + off, val);
}

static void
rp1_sdhci_write_2(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint16_t val)
{
	struct rp1_sdhci_softc *sc = device_get_softc(dev);

	bus_space_write_2(sc->bst, sc->bsh, sc->reg_off + off, val);
}

static void
rp1_sdhci_write_4(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint32_t val)
{
	struct rp1_sdhci_softc *sc = device_get_softc(dev);

	bus_space_write_4(sc->bst, sc->bsh, sc->reg_off + off, val);
}

static void
rp1_sdhci_write_multi_4(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint32_t *data, bus_size_t count)
{
	struct rp1_sdhci_softc *sc = device_get_softc(dev);

	bus_space_write_multi_4(sc->bst, sc->bsh, sc->reg_off + off, data, count);
}

static void
rp1_sdhci_intr(void *arg)
{
	struct rp1_sdhci_softc *sc = arg;

	sdhci_generic_intr(&sc->slot);

	/* Acknowledge interrupt to RP1 parent */
	rp1_intr_ack(device_get_parent(sc->dev), rp1_get_irq(sc->dev));
}

static int
rp1_sdhci_get_ro(device_t bus, device_t child)
{
	/* For SDIO (WiFi), always return not read-only */
	return (0);
}

static bool
rp1_sdhci_get_card_present(device_t dev, struct sdhci_slot *slot)
{
	struct rp1_sdhci_softc *sc = device_get_softc(dev);

	/*
	 * For SDIO1 (WiFi), always report card present since the
	 * CYW43455 is soldered to the board.
	 * For SDIO0 (SD card slot), check the actual card detect.
	 */
	if (sc->slot_id == 1)
		return (true);

	/* For SD card slot, use generic detection */
	return (sdhci_generic_get_card_present(dev, slot));
}

static int
rp1_sdhci_probe(device_t dev)
{
	device_set_desc(dev, "RPi5 RP1 SDHCI Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
rp1_sdhci_attach(device_t dev)
{
	struct rp1_sdhci_softc *sc;
	struct rp1_softc *rp1_sc;
	device_t parent;
	int err, rid;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Get parent RP1 resources */
	parent = device_get_parent(dev);
	rp1_sc = device_get_softc(parent);

	sc->reg_off = rp1_get_offset(dev);

	/* Determine which slot this is based on offset */
	if (sc->reg_off == RP1_SDIO0_BASE)
		sc->slot_id = 0;	/* SD card */
	else
		sc->slot_id = 1;	/* SDIO/WiFi */

	/*
	 * Get the parent's BAR resource. The parent RP1 driver returns
	 * its own BAR, and we access registers at our configured offset.
	 */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate memory resource\n");
		return (ENXIO);
	}

	/*
	 * Use the parent's bus space tag/handle directly since the
	 * parent's rp1_alloc_resource returns its own BAR.
	 */
	sc->bst = rp1_sc->bst;
	sc->bsh = rp1_sc->bsh;

	/* Allocate interrupt */
	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "cannot allocate IRQ resource\n");
		err = ENXIO;
		goto fail;
	}

	/* Setup interrupt handler */
	err = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, rp1_sdhci_intr, sc, &sc->irq_ih);
	if (err != 0) {
		device_printf(dev, "cannot setup interrupt handler\n");
		goto fail;
	}

	/*
	 * Initialize the SDHCI slot.
	 * RP1's dwcmshc has minimal quirks compared to Rockchip.
	 */
	sc->slot.quirks = RP1_SDHCI_QUIRKS;

	/*
	 * Set capabilities - the RP1 dwcmshc supports:
	 * - 4-bit bus width
	 * - High-speed mode
	 * - 3.3V signaling
	 * - SDMA (no ADMA)
	 */
	sc->slot.caps = SDHCI_CAN_VDD_330 | SDHCI_CAN_DO_HISPD;

	/* For SDIO1 (WiFi), mark as non-removable embedded slot */
	if (sc->slot_id == 1) {
		sc->slot.opt |= SDHCI_NON_REMOVABLE | SDHCI_SLOT_EMBEDDED;
		sc->slot.quirks |= SDHCI_QUIRK_POLL_CARD_PRESENT;
	}

	/*
	 * Set max clock. The RP1 SDIO runs at 200MHz source but
	 * typically limited to 50MHz for SD and 25MHz for SDIO.
	 */
	sc->slot.max_clk = 200000000;

	err = sdhci_init_slot(dev, &sc->slot, 0);
	if (err != 0) {
		device_printf(dev, "cannot init SDHCI slot\n");
		goto fail;
	}

	sdhci_start_slot(&sc->slot);

	device_printf(dev, "SDHCI slot %d initialized\n", sc->slot_id);

	return (0);

fail:
	if (sc->irq_ih != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_ih);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
	return (err);
}

static int
rp1_sdhci_detach(device_t dev)
{
	struct rp1_sdhci_softc *sc = device_get_softc(dev);

	sdhci_cleanup_slot(&sc->slot);

	if (sc->irq_ih != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_ih);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return (0);
}

static device_method_t rp1_sdhci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rp1_sdhci_probe),
	DEVMETHOD(device_attach,	rp1_sdhci_attach),
	DEVMETHOD(device_detach,	rp1_sdhci_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	sdhci_generic_read_ivar),
	DEVMETHOD(bus_write_ivar,	sdhci_generic_write_ivar),

	/* MMC bridge interface */
	DEVMETHOD(mmcbr_update_ios,	sdhci_generic_update_ios),
	DEVMETHOD(mmcbr_request,	sdhci_generic_request),
	DEVMETHOD(mmcbr_get_ro,		rp1_sdhci_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	sdhci_generic_acquire_host),
	DEVMETHOD(mmcbr_release_host,	sdhci_generic_release_host),
	DEVMETHOD(mmcbr_switch_vccq,	sdhci_generic_switch_vccq),
	DEVMETHOD(mmcbr_tune,		sdhci_generic_tune),
	DEVMETHOD(mmcbr_retune,		sdhci_generic_retune),

	/* SDHCI interface */
	DEVMETHOD(sdhci_read_1,		rp1_sdhci_read_1),
	DEVMETHOD(sdhci_read_2,		rp1_sdhci_read_2),
	DEVMETHOD(sdhci_read_4,		rp1_sdhci_read_4),
	DEVMETHOD(sdhci_read_multi_4,	rp1_sdhci_read_multi_4),
	DEVMETHOD(sdhci_write_1,	rp1_sdhci_write_1),
	DEVMETHOD(sdhci_write_2,	rp1_sdhci_write_2),
	DEVMETHOD(sdhci_write_4,	rp1_sdhci_write_4),
	DEVMETHOD(sdhci_write_multi_4,	rp1_sdhci_write_multi_4),
	DEVMETHOD(sdhci_get_card_present, rp1_sdhci_get_card_present),

	DEVMETHOD_END
};

static driver_t rp1_sdhci_driver = {
	"sdhci_rp1",
	rp1_sdhci_methods,
	sizeof(struct rp1_sdhci_softc)
};

DRIVER_MODULE(sdhci_rp1, rp1, rp1_sdhci_driver, NULL, NULL);
SDHCI_DEPEND(sdhci_rp1);
MMC_DECLARE_BRIDGE(sdhci_rp1);
MODULE_DEPEND(sdhci_rp1, rp1, 1, 1, 1);
MODULE_VERSION(sdhci_rp1, 1);
