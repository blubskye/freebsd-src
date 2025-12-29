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
 * This driver handles PCIe enumeration, MSI-X interrupt routing, and
 * provides a bus for child devices to attach to.
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
#define	RP1_BAR_MSIX_CFG	2	/* MSI-X configuration */

/* System info registers */
#define	RP1_SYSINFO_CHIP_ID	0x000000
#define	RP1_SYSINFO_PLATFORM	0x000004

/* MSI-X configuration registers (in BAR2) */
#define	RP1_MSIX_CFG_BASE	0x008
#define	RP1_MSIX_CFG(x)		(RP1_MSIX_CFG_BASE + ((x) * 4))
#define	RP1_MSIX_CFG_IACK_EN	(1 << 3)
#define	RP1_MSIX_CFG_IACK	(1 << 2)
#define	RP1_MSIX_CFG_TEST	(1 << 1)
#define	RP1_MSIX_CFG_ENABLE	(1 << 0)

/* Register set/clear offsets */
#define	RP1_REG_RW		0x000
#define	RP1_REG_SET		0x800
#define	RP1_REG_CLR		0xc00

/* Register access macros */
#define	RP1_RD4(sc, off)	\
	bus_space_read_4((sc)->bst, (sc)->bsh, (off))
#define	RP1_WR4(sc, off, val)	\
	bus_space_write_4((sc)->bst, (sc)->bsh, (off), (val))

/* MSI-X config register access */
#define	RP1_MSIX_RD4(sc, off)	\
	bus_space_read_4((sc)->msix_bst, (sc)->msix_bsh, (off))
#define	RP1_MSIX_WR4(sc, off, val)	\
	bus_space_write_4((sc)->msix_bst, (sc)->msix_bsh, (off), (val))

static int	rp1_probe(device_t);
static int	rp1_attach(device_t);
static int	rp1_detach(device_t);
static int	rp1_setup_msix(struct rp1_softc *);
static void	rp1_teardown_msix(struct rp1_softc *);
static int	rp1_create_children(struct rp1_softc *);

/* Child device definitions */
static const struct rp1_child_info rp1_children[] = {
	{ "rp1eth", RP1_ETH_BASE, RP1_ETH_SIZE, RP1_INT_ETH },
	{ "sdhci_rp1", RP1_SDIO0_BASE, RP1_SDIO0_SIZE, RP1_INT_SDIO0 },
	{ "sdhci_rp1", RP1_SDIO1_BASE, RP1_SDIO1_SIZE, RP1_INT_SDIO1 },
	/* Future: USB, GPIO, UART, etc. */
	{ NULL, 0, 0, 0 }
};

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

	/* Map peripheral BAR (BAR0) */
	rid = PCIR_BAR(RP1_BAR_PERIPH);
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot allocate peripheral BAR\n");
		error = ENXIO;
		goto fail;
	}
	sc->bst = rman_get_bustag(sc->res);
	sc->bsh = rman_get_bushandle(sc->res);

	/* Map MSI-X configuration BAR (BAR2) */
	rid = PCIR_BAR(RP1_BAR_MSIX_CFG);
	sc->msix_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->msix_res == NULL) {
		device_printf(dev, "cannot allocate MSI-X config BAR\n");
		error = ENXIO;
		goto fail;
	}
	sc->msix_bst = rman_get_bustag(sc->msix_res);
	sc->msix_bsh = rman_get_bushandle(sc->msix_res);

	/* Read chip identification */
	chip_id = RP1_RD4(sc, RP1_SYSINFO_CHIP_ID);
	platform = RP1_RD4(sc, RP1_SYSINFO_PLATFORM);
	sc->chip_id = chip_id;
	sc->platform = platform;

	device_printf(dev, "RP1 chip_id=0x%08x platform=0x%08x\n",
	    chip_id, platform);

	/* Setup MSI-X interrupts */
	error = rp1_setup_msix(sc);
	if (error != 0) {
		device_printf(dev, "MSI-X setup failed: %d\n", error);
		goto fail;
	}

	/* Create child devices */
	error = rp1_create_children(sc);
	if (error != 0) {
		device_printf(dev, "child enumeration failed: %d\n", error);
		goto fail;
	}

	/* Attach children */
	bus_attach_children(dev);

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

	/* Detach children first */
	bus_generic_detach(dev);

	/* Teardown MSI-X */
	rp1_teardown_msix(sc);

	/* Release BAR2 (MSI-X config) */
	if (sc->msix_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(RP1_BAR_MSIX_CFG), sc->msix_res);
		sc->msix_res = NULL;
	}

	/* Release BAR0 (peripherals) */
	if (sc->res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(RP1_BAR_PERIPH), sc->res);
		sc->res = NULL;
	}

	if (mtx_initialized(&sc->mtx))
		mtx_destroy(&sc->mtx);

	return (0);
}

/*
 * Setup MSI-X interrupts
 */
static int
rp1_setup_msix(struct rp1_softc *sc)
{
	device_t dev = sc->dev;
	int count, error, i, rid;

	/* Request MSI-X vectors */
	count = RP1_INT_END;
	error = pci_alloc_msix(dev, &count);
	if (error != 0) {
		device_printf(dev, "pci_alloc_msix failed: %d\n", error);
		return (error);
	}

	if (count < RP1_INT_END) {
		device_printf(dev, "only got %d MSI-X vectors (wanted %d)\n",
		    count, RP1_INT_END);
		/* Continue with what we got */
	}

	sc->msix_count = count;
	device_printf(dev, "allocated %d MSI-X vectors\n", count);

	/* Allocate IRQ resources for each vector */
	for (i = 0; i < count; i++) {
		rid = i + 1;  /* MSI-X rids start at 1 */
		sc->msix_irq[i] = bus_alloc_resource_any(dev, SYS_RES_IRQ,
		    &rid, RF_ACTIVE);
		if (sc->msix_irq[i] == NULL) {
			device_printf(dev, "cannot allocate IRQ %d\n", i);
			/* Non-fatal: some vectors may not be needed */
		}
	}

	return (0);
}

/*
 * Teardown MSI-X interrupts
 */
static void
rp1_teardown_msix(struct rp1_softc *sc)
{
	device_t dev = sc->dev;
	int i, rid;

	for (i = 0; i < sc->msix_count; i++) {
		if (sc->msix_ih[i] != NULL) {
			bus_teardown_intr(dev, sc->msix_irq[i], sc->msix_ih[i]);
			sc->msix_ih[i] = NULL;
		}
		if (sc->msix_irq[i] != NULL) {
			rid = i + 1;
			bus_release_resource(dev, SYS_RES_IRQ, rid,
			    sc->msix_irq[i]);
			sc->msix_irq[i] = NULL;
		}
	}

	if (sc->msix_count > 0) {
		pci_release_msi(dev);
		sc->msix_count = 0;
	}
}

/*
 * Create child devices for each peripheral
 */
static int
rp1_create_children(struct rp1_softc *sc)
{
	device_t dev = sc->dev;
	device_t child;
	const struct rp1_child_info *ci;
	int i;

	for (i = 0; rp1_children[i].name != NULL; i++) {
		ci = &rp1_children[i];

		child = device_add_child(dev, ci->name, -1);
		if (child == NULL) {
			device_printf(dev, "cannot add child %s\n", ci->name);
			continue;
		}

		/* Store child info for later retrieval */
		device_set_ivars(child, __DECONST(void *, ci));
	}

	return (0);
}

/*
 * Bus interface methods - allows child devices to access RP1 resources
 */

static int
rp1_read_ivar(device_t dev, device_t child, int which, uintptr_t *result)
{
	const struct rp1_child_info *ci;

	ci = device_get_ivars(child);
	if (ci == NULL)
		return (EINVAL);

	switch (which) {
	case RP1_IVAR_OFFSET:
		*result = ci->offset;
		return (0);
	case RP1_IVAR_SIZE:
		*result = ci->size;
		return (0);
	case RP1_IVAR_IRQ:
		*result = ci->irq;
		return (0);
	default:
		return (ENOENT);
	}
}

static struct resource *
rp1_alloc_resource(device_t dev, device_t child, int type, int rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct rp1_softc *sc;

	sc = device_get_softc(dev);

	/* Child devices access registers through the parent's BAR */
	if (type == SYS_RES_MEMORY) {
		/*
		 * Return the parent's resource. Children use their
		 * configured offset within this resource.
		 */
		return (sc->res);
	}

	/* For IRQ resources, return the appropriate MSI-X vector */
	if (type == SYS_RES_IRQ) {
		const struct rp1_child_info *ci;
		int irq;

		ci = device_get_ivars(child);
		if (ci == NULL)
			return (NULL);

		irq = ci->irq;
		if (irq >= 0 && irq < sc->msix_count)
			return (sc->msix_irq[irq]);
	}

	return (NULL);
}

static int
rp1_release_resource(device_t dev, device_t child, struct resource *r)
{
	/* Resources are owned by parent, nothing to release */
	return (0);
}

static int
rp1_setup_intr(device_t dev, device_t child, struct resource *irq, int flags,
    driver_filter_t *filter, driver_intr_t *intr, void *arg, void **cookiep)
{
	struct rp1_softc *sc;
	const struct rp1_child_info *ci;
	int error, irqnum;

	sc = device_get_softc(dev);
	ci = device_get_ivars(child);
	if (ci == NULL)
		return (EINVAL);

	irqnum = ci->irq;
	if (irqnum < 0 || irqnum >= sc->msix_count)
		return (EINVAL);

	if (sc->msix_irq[irqnum] == NULL)
		return (ENXIO);

	/* Setup the interrupt handler */
	error = bus_setup_intr(dev, sc->msix_irq[irqnum], flags,
	    filter, intr, arg, &sc->msix_ih[irqnum]);
	if (error != 0)
		return (error);

	*cookiep = sc->msix_ih[irqnum];

	/* Enable the MSI-X vector and configure for level-triggered */
	RP1_MSIX_WR4(sc, RP1_REG_SET + RP1_MSIX_CFG(irqnum),
	    RP1_MSIX_CFG_IACK_EN | RP1_MSIX_CFG_ENABLE);

	return (0);
}

static int
rp1_teardown_intr(device_t dev, device_t child, struct resource *irq,
    void *cookie)
{
	struct rp1_softc *sc;
	const struct rp1_child_info *ci;
	int error, irqnum;

	sc = device_get_softc(dev);
	ci = device_get_ivars(child);
	if (ci == NULL)
		return (EINVAL);

	irqnum = ci->irq;
	if (irqnum < 0 || irqnum >= sc->msix_count)
		return (EINVAL);

	/* Disable the MSI-X vector */
	RP1_MSIX_WR4(sc, RP1_REG_CLR + RP1_MSIX_CFG(irqnum),
	    RP1_MSIX_CFG_IACK_EN | RP1_MSIX_CFG_ENABLE);

	/* Teardown the interrupt handler */
	error = bus_teardown_intr(dev, sc->msix_irq[irqnum], cookie);
	if (error == 0)
		sc->msix_ih[irqnum] = NULL;

	return (error);
}

/*
 * Acknowledge a level-triggered interrupt
 * Called by child drivers after handling the interrupt
 */
void
rp1_intr_ack(device_t dev, int irqnum)
{
	struct rp1_softc *sc;

	sc = device_get_softc(dev);
	if (irqnum >= 0 && irqnum < sc->msix_count) {
		RP1_MSIX_WR4(sc, RP1_REG_SET + RP1_MSIX_CFG(irqnum),
		    RP1_MSIX_CFG_IACK);
	}
}

static device_method_t rp1_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rp1_probe),
	DEVMETHOD(device_attach,	rp1_attach),
	DEVMETHOD(device_detach,	rp1_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	rp1_read_ivar),
	DEVMETHOD(bus_alloc_resource,	rp1_alloc_resource),
	DEVMETHOD(bus_release_resource,	rp1_release_resource),
	DEVMETHOD(bus_setup_intr,	rp1_setup_intr),
	DEVMETHOD(bus_teardown_intr,	rp1_teardown_intr),

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
