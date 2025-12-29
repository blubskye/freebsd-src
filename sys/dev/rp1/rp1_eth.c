/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 The FreeBSD Project
 * Copyright (c) 2020 Michael J Karels
 * Copyright (c) 2016, 2020 Jared McNeill <jmcneill@invisible.ca>
 *
 * Raspberry Pi 5 Ethernet Driver (GENET via RP1)
 *
 * This driver adapts the existing FreeBSD GENET driver for use on the
 * Raspberry Pi 5, where the GENET MAC is accessed through the RP1 I/O
 * controller via PCIe rather than being directly memory-mapped.
 *
 * Key differences from RPi4 genet driver:
 *   - Attaches to rp1 bus instead of simplebus (FDT)
 *   - Registers accessed via RP1's BAR at RP1_ETH_BASE offset
 *   - Interrupts routed through RP1's MSI-X
 *   - PHY is BCM54213PE (RGMII, same as RPi4)
 *
 * This is a phased implementation:
 *   Phase 1: Basic structure and probe/attach
 *   Phase 2: DMA ring setup
 *   Phase 3: TX/RX path
 *   Phase 4: PHY management
 *   Phase 5: Full functionality
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/taskqueue.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/ethernet.h>

#include <machine/bus.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include "rp1_var.h"
#include "rp1_ethreg.h"
#include "miibus_if.h"

/* Use the same register definitions as arm64/broadcom/genet */
#define	__BIT(_x)	(1 << (_x))

#define	RD4(sc, reg)		\
	bus_space_read_4((sc)->bst, (sc)->bsh, (sc)->eth_base + (reg))
#define	WR4(sc, reg, val)	\
	bus_space_write_4((sc)->bst, (sc)->bsh, (sc)->eth_base + (reg), (val))

#define	RP1_ETH_LOCK(sc)	mtx_lock(&(sc)->mtx)
#define	RP1_ETH_UNLOCK(sc)	mtx_unlock(&(sc)->mtx)

#define	TX_DESC_COUNT		256
#define	RX_DESC_COUNT		256

struct rp1_eth_ring_ent {
	bus_dmamap_t		map;
	struct mbuf		*mbuf;
};

struct rp1_eth_softc {
	device_t		dev;
	device_t		miibus;
	struct mtx		mtx;
	if_t			ifp;

	/* Parent RP1 resources */
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	bus_size_t		eth_base;

	/* Interrupt */
	struct resource		*irq_res;
	void			*irq_ih;

	/* MII/PHY */
	int			phy_mode;
	int			link;

	/* Callouts */
	struct callout		stat_ch;
	struct task		link_task;

	/* DMA */
	bus_dma_tag_t		tx_buf_tag;
	bus_dma_tag_t		rx_buf_tag;

	struct rp1_eth_ring_ent	tx_ring[TX_DESC_COUNT];
	struct rp1_eth_ring_ent	rx_ring[RX_DESC_COUNT];

	int			tx_cur;
	int			tx_next;
	int			tx_queued;
	int			rx_cur;

	int			if_flags;
};

static int	rp1_eth_probe(device_t);
static int	rp1_eth_attach(device_t);
static int	rp1_eth_detach(device_t);

/* Interface methods */
static void	rp1_eth_init(void *);
static void	rp1_eth_start(if_t);
static int	rp1_eth_ioctl(if_t, u_long, caddr_t);
static void	rp1_eth_stop(struct rp1_eth_softc *);

/* MII methods */
static int	rp1_eth_miibus_readreg(device_t, int, int);
static int	rp1_eth_miibus_writereg(device_t, int, int, int);
static void	rp1_eth_miibus_statchg(device_t);

/* Internal methods */
static void	rp1_eth_reset(struct rp1_eth_softc *);
static void	rp1_eth_tick(void *);
static void	rp1_eth_link_task(void *, int);
static void	rp1_eth_intr(void *);
static void	rp1_eth_media_status(if_t, struct ifmediareq *);
static int	rp1_eth_media_change(if_t);

static int
rp1_eth_probe(device_t dev)
{
	/*
	 * This driver attaches as a child of the rp1 bus.
	 * The parent creates us when it finds a GENET at the expected offset.
	 */
	device_set_desc(dev, "RPi5 GENET Gigabit Ethernet");
	return (BUS_PROBE_DEFAULT);
}

static int
rp1_eth_attach(device_t dev)
{
	struct rp1_eth_softc *sc;
	struct rp1_softc *rp1_sc;
	device_t parent;
	uint32_t rev;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK, MTX_DEF);
	callout_init_mtx(&sc->stat_ch, &sc->mtx, 0);
	TASK_INIT(&sc->link_task, 0, rp1_eth_link_task, sc);

	/* Get parent RP1 resources */
	parent = device_get_parent(dev);
	rp1_sc = device_get_softc(parent);

	sc->bst = rp1_sc->bst;
	sc->bsh = rp1_sc->bsh;
	sc->eth_base = RP1_ETH_BASE;

	/* Verify GENET presence by reading revision */
	rev = RD4(sc, GENET_SYS_REV_CTRL);
	device_printf(dev, "GENET revision 0x%08x\n", rev);

	/* Check for expected version (GENET v5) */
	if (((rev & REV_MAJOR) >> REV_MAJOR_SHIFT) != REV_MAJOR_V5) {
		device_printf(dev, "unsupported GENET version %d\n",
		    (rev & REV_MAJOR) >> REV_MAJOR_SHIFT);
		error = ENXIO;
		goto fail;
	}

	/* Reset the MAC */
	rp1_eth_reset(sc);

	/* TODO Phase 2: Setup DMA rings */
	/* TODO Phase 3: Setup interrupt handler */
	/* TODO Phase 4: Attach MII bus */

	/* Allocate network interface */
	sc->ifp = if_alloc(IFT_ETHER);
	if (sc->ifp == NULL) {
		device_printf(dev, "cannot allocate ifnet\n");
		error = ENOMEM;
		goto fail;
	}

	if_setsoftc(sc->ifp, sc);
	if_initname(sc->ifp, device_get_name(dev), device_get_unit(dev));
	if_setflags(sc->ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setinitfn(sc->ifp, rp1_eth_init);
	if_setstartfn(sc->ifp, rp1_eth_start);
	if_setioctlfn(sc->ifp, rp1_eth_ioctl);
	if_setsendqlen(sc->ifp, TX_DESC_COUNT - 1);
	if_setsendqready(sc->ifp);

	/* Hardware capabilities */
	if_setcapabilities(sc->ifp, IFCAP_VLAN_MTU | IFCAP_HWCSUM);
	if_setcapenable(sc->ifp, if_getcapabilities(sc->ifp));
	if_sethwassist(sc->ifp, CSUM_TCP | CSUM_UDP);

	/* Generate MAC address - will be replaced by proper source later */
	{
		struct ether_addr eaddr;
		ether_gen_addr(sc->ifp, &eaddr);
		ether_ifattach(sc->ifp, eaddr.octet);
	}

	device_printf(dev, "RPi5 Ethernet attached (Phase 1 - skeleton)\n");
	return (0);

fail:
	rp1_eth_detach(dev);
	return (error);
}

static int
rp1_eth_detach(device_t dev)
{
	struct rp1_eth_softc *sc;

	sc = device_get_softc(dev);

	if (sc->ifp != NULL) {
		ether_ifdetach(sc->ifp);
		if_free(sc->ifp);
	}

	callout_drain(&sc->stat_ch);

	if (mtx_initialized(&sc->mtx))
		mtx_destroy(&sc->mtx);

	return (0);
}

static void
rp1_eth_reset(struct rp1_eth_softc *sc)
{
	uint32_t val;

	/* Flush RX buffer */
	val = RD4(sc, GENET_SYS_RBUF_FLUSH_CTRL);
	val |= GENET_SYS_RBUF_FLUSH_RESET;
	WR4(sc, GENET_SYS_RBUF_FLUSH_CTRL, val);
	DELAY(10);

	val &= ~GENET_SYS_RBUF_FLUSH_RESET;
	WR4(sc, GENET_SYS_RBUF_FLUSH_CTRL, val);
	DELAY(10);

	WR4(sc, GENET_SYS_RBUF_FLUSH_CTRL, 0);
	DELAY(10);

	/* Reset UniMAC */
	WR4(sc, GENET_UMAC_CMD, 0);
	WR4(sc, GENET_UMAC_CMD,
	    GENET_UMAC_CMD_LCL_LOOP_EN | GENET_UMAC_CMD_SW_RESET);
	DELAY(10);
	WR4(sc, GENET_UMAC_CMD, 0);

	/* Reset MIB counters */
	WR4(sc, GENET_UMAC_MIB_CTRL, GENET_UMAC_MIB_RESET_RUNT |
	    GENET_UMAC_MIB_RESET_RX | GENET_UMAC_MIB_RESET_TX);
	WR4(sc, GENET_UMAC_MIB_CTRL, 0);
}

static void
rp1_eth_init(void *softc)
{
	struct rp1_eth_softc *sc = softc;

	RP1_ETH_LOCK(sc);
	device_printf(sc->dev, "rp1_eth_init called (not yet implemented)\n");
	RP1_ETH_UNLOCK(sc);
}

static void
rp1_eth_start(if_t ifp)
{
	struct rp1_eth_softc *sc = if_getsoftc(ifp);

	RP1_ETH_LOCK(sc);
	/* TODO: Implement TX path */
	RP1_ETH_UNLOCK(sc);
}

static int
rp1_eth_ioctl(if_t ifp, u_long cmd, caddr_t data)
{
	struct rp1_eth_softc *sc;
	int error = 0;

	sc = if_getsoftc(ifp);

	switch (cmd) {
	case SIOCSIFFLAGS:
		RP1_ETH_LOCK(sc);
		if (if_getflags(ifp) & IFF_UP) {
			if (!(if_getdrvflags(ifp) & IFF_DRV_RUNNING))
				rp1_eth_init(sc);
		} else {
			if (if_getdrvflags(ifp) & IFF_DRV_RUNNING)
				rp1_eth_stop(sc);
		}
		RP1_ETH_UNLOCK(sc);
		break;

	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		/* TODO: MII media handling */
		error = EINVAL;
		break;

	default:
		error = ether_ioctl(ifp, cmd, data);
		break;
	}

	return (error);
}

static void
rp1_eth_stop(struct rp1_eth_softc *sc)
{
	if_setdrvflagbits(sc->ifp, 0, IFF_DRV_RUNNING);
	/* TODO: Disable TX/RX, clear rings */
}

static void
rp1_eth_tick(void *softc)
{
	/* TODO: Periodic link status check */
}

static void
rp1_eth_link_task(void *arg, int pending)
{
	/* TODO: Link state change handling */
}

static void
rp1_eth_intr(void *softc)
{
	/* TODO: Interrupt handler */
}

static void
rp1_eth_media_status(if_t ifp, struct ifmediareq *ifmr)
{
	/* TODO: Report media status */
}

static int
rp1_eth_media_change(if_t ifp)
{
	/* TODO: Handle media change */
	return (0);
}

/* MII bus methods - same as original genet driver */
static int
rp1_eth_miibus_readreg(device_t dev, int phy, int reg)
{
	struct rp1_eth_softc *sc;
	int retry, val;

	sc = device_get_softc(dev);
	val = 0;

	WR4(sc, GENET_MDIO_CMD, GENET_MDIO_READ |
	    (phy << GENET_MDIO_ADDR_SHIFT) | (reg << GENET_MDIO_REG_SHIFT));
	val = RD4(sc, GENET_MDIO_CMD);
	WR4(sc, GENET_MDIO_CMD, val | GENET_MDIO_START_BUSY);

	for (retry = 1000; retry > 0; retry--) {
		val = RD4(sc, GENET_MDIO_CMD);
		if ((val & GENET_MDIO_START_BUSY) == 0) {
			if (val & GENET_MDIO_READ_FAILED)
				return (0);
			return (val & GENET_MDIO_VAL_MASK);
		}
		DELAY(10);
	}

	device_printf(dev, "PHY read timeout, phy=%d reg=%d\n", phy, reg);
	return (0);
}

static int
rp1_eth_miibus_writereg(device_t dev, int phy, int reg, int val)
{
	struct rp1_eth_softc *sc;
	int retry;

	sc = device_get_softc(dev);

	WR4(sc, GENET_MDIO_CMD, GENET_MDIO_WRITE |
	    (phy << GENET_MDIO_ADDR_SHIFT) | (reg << GENET_MDIO_REG_SHIFT) |
	    (val & GENET_MDIO_VAL_MASK));
	val = RD4(sc, GENET_MDIO_CMD);
	WR4(sc, GENET_MDIO_CMD, val | GENET_MDIO_START_BUSY);

	for (retry = 1000; retry > 0; retry--) {
		val = RD4(sc, GENET_MDIO_CMD);
		if ((val & GENET_MDIO_START_BUSY) == 0)
			break;
		DELAY(10);
	}

	if (retry == 0)
		device_printf(dev, "PHY write timeout, phy=%d reg=%d\n",
		    phy, reg);

	return (0);
}

static void
rp1_eth_miibus_statchg(device_t dev)
{
	/* TODO: Handle PHY status change */
}

static device_method_t rp1_eth_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rp1_eth_probe),
	DEVMETHOD(device_attach,	rp1_eth_attach),
	DEVMETHOD(device_detach,	rp1_eth_detach),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	rp1_eth_miibus_readreg),
	DEVMETHOD(miibus_writereg,	rp1_eth_miibus_writereg),
	DEVMETHOD(miibus_statchg,	rp1_eth_miibus_statchg),

	DEVMETHOD_END
};

static driver_t rp1_eth_driver = {
	"rp1eth",
	rp1_eth_methods,
	sizeof(struct rp1_eth_softc)
};

DRIVER_MODULE(rp1eth, rp1, rp1_eth_driver, NULL, NULL);
DRIVER_MODULE(miibus, rp1eth, miibus_driver, NULL, NULL);
MODULE_DEPEND(rp1eth, rp1, 1, 1, 1);
MODULE_DEPEND(rp1eth, ether, 1, 1, 1);
MODULE_DEPEND(rp1eth, miibus, 1, 1, 1);
MODULE_VERSION(rp1eth, 1);
