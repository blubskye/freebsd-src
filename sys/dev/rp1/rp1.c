/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 The FreeBSD Project
 *
 * Raspberry Pi 5 RP1 I/O Controller Driver
 *
 * The RP1 is an I/O controller chip on the Raspberry Pi 5, connected
 * via PCIe. It provides access to various peripherals including:
 *   - Gigabit Ethernet (GENET)
 *   - USB 3.0
 *   - GPIO
 *   - UART
 *   - SPI
 *   - I2C
 *   - PWM
 *
 * This driver handles PCIe enumeration and provides a bus for child
 * devices to attach to.
 *
 * Phase 1: Basic PCIe attachment and BAR mapping
 * Phase 2: Interrupt routing (MSI-X)
 * Phase 3: Clock management
 * Phase 4: Child device enumeration
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "rp1_var.h"

#define	RP1_VENDOR_ID		0x1de4	/* Raspberry Pi */
#define	RP1_DEVICE_ID_C0	0x0001	/* RP1 C0 stepping */

/* BAR layout */
#define	RP1_BAR_PERIPH		0	/* Peripheral registers */
#define	RP1_BAR_MSIX		2	/* MSI-X configuration */

/* System info registers */
#define	RP1_SYSINFO_BASE	0x000000
#define	RP1_SYSINFO_CHIP_ID	0x000000
#define	RP1_SYSINFO_PLATFORM	0x000004

/* MSI-X configuration registers */
#define	RP1_MSIX_CFG_BASE	0x008
#define	RP1_MSIX_CFG(x)		(RP1_MSIX_CFG_BASE + ((x) * 4))
#define	RP1_MSIX_CFG_IACK_EN	(1 << 3)
#define	RP1_MSIX_CFG_IACK	(1 << 2)
#define	RP1_MSIX_CFG_TEST	(1 << 1)
#define	RP1_MSIX_CFG_ENABLE	(1 << 0)

/* Register access macros */
#define	RP1_RD4(sc, off)	\
	bus_space_read_4((sc)->bst, (sc)->bsh, (off))
#define	RP1_WR4(sc, off, val)	\
	bus_space_write_4((sc)->bst, (sc)->bsh, (off), (val))

static int	rp1_probe(device_t);
static int	rp1_attach(device_t);
static int	rp1_detach(device_t);

static int
rp1_probe(device_t dev)
{
	if (pci_get_vendor(dev) != RP1_VENDOR_ID)
		return (ENXIO);
	if (pci_get_device(dev) != RP1_DEVICE_ID_C0)
		return (ENXIO);

	device_set_desc(dev, "Raspberry Pi 5 RP1 I/O Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
rp1_attach(device_t dev)
{
	struct rp1_softc *sc;
	uint32_t chip_id, platform;
	int error, rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), "rp1", MTX_DEF);

	/* Enable bus mastering */
	pci_enable_busmaster(dev);

	/* Map peripheral BAR */
	rid = PCIR_BAR(RP1_BAR_PERIPH);
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot allocate peripheral BAR\n");
		error = ENXIO;
		goto fail;
	}
	sc->bst = rman_get_bustag(sc->res);
	sc->bsh = rman_get_bushandle(sc->res);

	/* Read chip identification */
	chip_id = RP1_RD4(sc, RP1_SYSINFO_CHIP_ID);
	platform = RP1_RD4(sc, RP1_SYSINFO_PLATFORM);

	device_printf(dev, "RP1 chip_id=0x%08x platform=0x%08x\n",
	    chip_id, platform);

	/* TODO Phase 2: Set up MSI-X interrupts */
	/* TODO Phase 3: Initialize clocks */
	/* TODO Phase 4: Create child devices (genet, etc.) */

	sc->attached = true;
	return (0);

fail:
	rp1_detach(dev);
	return (error);
}

static int
rp1_detach(device_t dev)
{
	struct rp1_softc *sc;

	sc = device_get_softc(dev);

	/* TODO: Tear down child devices */

	if (sc->res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(RP1_BAR_PERIPH), sc->res);
	}

	if (mtx_initialized(&sc->mtx))
		mtx_destroy(&sc->mtx);

	return (0);
}

/*
 * Bus interface methods - allows child devices to access RP1 resources
 */
static struct resource *
rp1_alloc_resource(device_t dev, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct rp1_softc *sc;

	sc = device_get_softc(dev);

	/* Child devices access registers through the parent's BAR */
	if (type == SYS_RES_MEMORY) {
		/* Translate child's offset to parent's resource */
		/* For now, return the parent resource - children will
		 * use offsets within it */
		return (sc->res);
	}

	return (NULL);
}

static int
rp1_release_resource(device_t dev, device_t child, struct resource *r)
{
	/* Resources are owned by parent, nothing to release */
	return (0);
}

static device_method_t rp1_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rp1_probe),
	DEVMETHOD(device_attach,	rp1_attach),
	DEVMETHOD(device_detach,	rp1_detach),

	/* Bus interface */
	DEVMETHOD(bus_alloc_resource,	rp1_alloc_resource),
	DEVMETHOD(bus_release_resource,	rp1_release_resource),

	DEVMETHOD_END
};

static driver_t rp1_driver = {
	"rp1",
	rp1_methods,
	sizeof(struct rp1_softc)
};

DRIVER_MODULE(rp1, pci, rp1_driver, NULL, NULL);
MODULE_DEPEND(rp1, pci, 1, 1, 1);
MODULE_VERSION(rp1, 1);
