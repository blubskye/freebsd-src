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
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/taskqueue.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/ethernet.h>

#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>

#include <machine/bus.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include "rp1_var.h"
#include "rp1_ethreg.h"
#include "miibus_if.h"

/* FreeBSD doesn't define BUS_SPACE_MAXADDR_40BIT, define it for GENET DMA */
#ifndef BUS_SPACE_MAXADDR_40BIT
#define	BUS_SPACE_MAXADDR_40BIT	0xFFFFFFFFFFULL
#endif

#define	__BIT(_x)	(1 << (_x))

#define	RD4(sc, reg)		\
	bus_space_read_4((sc)->bst, (sc)->bsh, (sc)->eth_base + (reg))
#define	WR4(sc, reg, val)	\
	bus_space_write_4((sc)->bst, (sc)->bsh, (sc)->eth_base + (reg), (val))

#define	RP1_ETH_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	RP1_ETH_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	RP1_ETH_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->mtx, MA_OWNED)

#define	TX_DESC_COUNT		256
#define	RX_DESC_COUNT		256
#define	TX_MAX_SEGS		20

#define	TX_NEXT(n)		(((n) + 1) & (TX_DESC_COUNT - 1))
#define	RX_NEXT(n)		(((n) + 1) & (RX_DESC_COUNT - 1))

/* Minimum header size to pull up for TX */
#define	TX_HDR_MIN		56	/* ether + ip6 + icmp6 */

/* RX batch size for if_input */
#define	RX_BATCH		16

/* Checksum offload flags */
#define	CSUM_DELAY_ANY	(CSUM_TCP | CSUM_UDP | CSUM_IP6_TCP | CSUM_IP6_UDP)

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
	int			irq_num;

	/* Interrupt */
	struct resource		*irq_res;
	void			*irq_ih;

	/* MII/PHY */
	int			link;

	/* Callouts */
	struct callout		stat_ch;
	struct task		link_task;

	/* DMA */
	bus_dma_tag_t		tx_buf_tag;
	bus_dma_tag_t		rx_buf_tag;

	struct rp1_eth_ring_ent	tx_ring[TX_DESC_COUNT];
	struct rp1_eth_ring_ent	rx_ring[RX_DESC_COUNT];

	/* TX queue state */
	u_int			tx_cur;
	u_int			tx_next;
	u_int			tx_queued;
	u_int			tx_prod_idx;
	u_int			tx_cons_idx;

	/* RX queue state */
	u_int			rx_cur;
	u_int			rx_prod_idx;
	u_int			rx_cons_idx;

	int			if_flags;
};

static int	rp1_eth_probe(device_t);
static int	rp1_eth_attach(device_t);
static int	rp1_eth_detach(device_t);

/* Interface methods */
static void	rp1_eth_init(void *);
static void	rp1_eth_init_locked(struct rp1_eth_softc *);
static void	rp1_eth_start(if_t);
static void	rp1_eth_start_locked(struct rp1_eth_softc *);
static int	rp1_eth_ioctl(if_t, u_long, caddr_t);
static void	rp1_eth_stop(struct rp1_eth_softc *);

/* MII methods */
static int	rp1_eth_miibus_readreg(device_t, int, int);
static int	rp1_eth_miibus_writereg(device_t, int, int, int);
static void	rp1_eth_miibus_statchg(device_t);

/* Internal methods */
static void	rp1_eth_reset(struct rp1_eth_softc *);
static int	rp1_eth_dma_init(struct rp1_eth_softc *);
static void	rp1_eth_dma_teardown(struct rp1_eth_softc *);
static void	rp1_eth_init_txring(struct rp1_eth_softc *);
static void	rp1_eth_init_rxring(struct rp1_eth_softc *);
static void	rp1_eth_enable(struct rp1_eth_softc *);
static void	rp1_eth_disable(struct rp1_eth_softc *);
static void	rp1_eth_dma_disable(struct rp1_eth_softc *);
static void	rp1_eth_tick(void *);
static void	rp1_eth_link_task(void *, int);
static void	rp1_eth_intr(void *);
static void	rp1_eth_txintr(struct rp1_eth_softc *);
static int	rp1_eth_rxintr(struct rp1_eth_softc *);
static int	rp1_eth_encap(struct rp1_eth_softc *, struct mbuf **);
static int	rp1_eth_newbuf_rx(struct rp1_eth_softc *, int);
static void	rp1_eth_setup_rxfilter(struct rp1_eth_softc *);
static void	rp1_eth_set_enaddr(struct rp1_eth_softc *);
static void	rp1_eth_update_link(struct rp1_eth_softc *);
static void	rp1_eth_media_status(if_t, struct ifmediareq *);
static int	rp1_eth_media_change(if_t);

static int
rp1_eth_probe(device_t dev)
{
	device_set_desc(dev, "RPi5 GENET Gigabit Ethernet");
	return (BUS_PROBE_DEFAULT);
}

static int
rp1_eth_attach(device_t dev)
{
	struct rp1_eth_softc *sc;
	struct rp1_softc *rp1_sc;
	device_t parent;
	struct ether_addr eaddr;
	uint32_t rev, major, minor;
	int error, rid;

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
	sc->eth_base = rp1_get_offset(dev);
	sc->irq_num = rp1_get_irq(dev);

	/* Verify GENET presence by reading revision */
	rev = RD4(sc, GENET_SYS_REV_CTRL);
	major = (rev & REV_MAJOR) >> REV_MAJOR_SHIFT;
	minor = (rev & REV_MINOR) >> REV_MINOR_SHIFT;

	if (major != REV_MAJOR_V5) {
		device_printf(dev, "unsupported GENET version %d\n", major);
		error = ENXIO;
		goto fail;
	}

	device_printf(dev, "GENET version 5.%d\n", minor);

	/* Reset the MAC */
	rp1_eth_reset(sc);
	rp1_eth_dma_disable(sc);

	/* Setup DMA */
	error = rp1_eth_dma_init(sc);
	if (error != 0) {
		device_printf(dev, "cannot setup DMA\n");
		goto fail;
	}

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
	if_setcapabilities(sc->ifp, IFCAP_VLAN_MTU | IFCAP_HWCSUM |
	    IFCAP_HWCSUM_IPV6);
	if_setcapenable(sc->ifp, if_getcapabilities(sc->ifp));
	if_sethwassist(sc->ifp, CSUM_TCP | CSUM_UDP);

	/* Setup interrupt handler */
	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "cannot allocate IRQ resource\n");
		error = ENXIO;
		goto fail;
	}

	error = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, rp1_eth_intr, sc, &sc->irq_ih);
	if (error != 0) {
		device_printf(dev, "cannot setup interrupt handler\n");
		goto fail;
	}

	/* Attach MII bus */
	error = mii_attach(dev, &sc->miibus, sc->ifp, rp1_eth_media_change,
	    rp1_eth_media_status, BMSR_DEFCAPMASK, MII_PHY_ANY,
	    MII_OFFSET_ANY, MIIF_DOPAUSE);
	if (error != 0) {
		device_printf(dev, "cannot attach PHY\n");
		goto fail;
	}

	/* Generate MAC address from hostid */
	ether_gen_addr(sc->ifp, &eaddr);
	ether_ifattach(sc->ifp, eaddr.octet);

	device_printf(dev, "Ethernet address: %6D\n", eaddr.octet, ":");

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
		RP1_ETH_LOCK(sc);
		rp1_eth_stop(sc);
		RP1_ETH_UNLOCK(sc);
		callout_drain(&sc->stat_ch);
		ether_ifdetach(sc->ifp);
	}

	if (sc->miibus != NULL)
		device_delete_child(dev, sc->miibus);

	if (sc->irq_ih != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_ih);

	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);

	rp1_eth_dma_teardown(sc);

	if (sc->ifp != NULL)
		if_free(sc->ifp);

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

static int
rp1_eth_dma_init(struct rp1_eth_softc *sc)
{
	device_t dev = sc->dev;
	int error, i;

	/* Create TX buffer tag */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(device_get_parent(dev)),	/* Parent tag */
	    4, 0,					/* alignment, boundary */
	    BUS_SPACE_MAXADDR_40BIT,			/* lowaddr */
	    BUS_SPACE_MAXADDR,				/* highaddr */
	    NULL, NULL,					/* filter, filterarg */
	    MCLBYTES, TX_MAX_SEGS,			/* maxsize, nsegs */
	    MCLBYTES,					/* maxsegsize */
	    0,						/* flags */
	    NULL, NULL,					/* lockfunc, lockarg */
	    &sc->tx_buf_tag);
	if (error != 0) {
		device_printf(dev, "cannot create TX buffer tag\n");
		return (error);
	}

	/* Create TX DMA maps */
	for (i = 0; i < TX_DESC_COUNT; i++) {
		error = bus_dmamap_create(sc->tx_buf_tag, 0,
		    &sc->tx_ring[i].map);
		if (error != 0) {
			device_printf(dev, "cannot create TX buffer map\n");
			return (error);
		}
	}

	/* Create RX buffer tag */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(device_get_parent(dev)),	/* Parent tag */
	    4, 0,					/* alignment, boundary */
	    BUS_SPACE_MAXADDR_40BIT,			/* lowaddr */
	    BUS_SPACE_MAXADDR,				/* highaddr */
	    NULL, NULL,					/* filter, filterarg */
	    MCLBYTES, 1,				/* maxsize, nsegs */
	    MCLBYTES,					/* maxsegsize */
	    0,						/* flags */
	    NULL, NULL,					/* lockfunc, lockarg */
	    &sc->rx_buf_tag);
	if (error != 0) {
		device_printf(dev, "cannot create RX buffer tag\n");
		return (error);
	}

	/* Create RX DMA maps */
	for (i = 0; i < RX_DESC_COUNT; i++) {
		error = bus_dmamap_create(sc->rx_buf_tag, 0,
		    &sc->rx_ring[i].map);
		if (error != 0) {
			device_printf(dev, "cannot create RX buffer map\n");
			return (error);
		}
	}

	return (0);
}

static void
rp1_eth_dma_teardown(struct rp1_eth_softc *sc)
{
	int i;

	/* Destroy TX resources */
	if (sc->tx_buf_tag != NULL) {
		for (i = 0; i < TX_DESC_COUNT; i++) {
			if (sc->tx_ring[i].map != NULL) {
				bus_dmamap_destroy(sc->tx_buf_tag,
				    sc->tx_ring[i].map);
				sc->tx_ring[i].map = NULL;
			}
		}
		bus_dma_tag_destroy(sc->tx_buf_tag);
		sc->tx_buf_tag = NULL;
	}

	/* Destroy RX resources */
	if (sc->rx_buf_tag != NULL) {
		for (i = 0; i < RX_DESC_COUNT; i++) {
			if (sc->rx_ring[i].map != NULL) {
				bus_dmamap_destroy(sc->rx_buf_tag,
				    sc->rx_ring[i].map);
				sc->rx_ring[i].map = NULL;
			}
		}
		bus_dma_tag_destroy(sc->rx_buf_tag);
		sc->rx_buf_tag = NULL;
	}
}

static void
rp1_eth_dma_disable(struct rp1_eth_softc *sc)
{
	uint32_t val;

	/* Disable TX DMA */
	val = RD4(sc, GENET_TX_DMA_CTRL);
	val &= ~GENET_TX_DMA_CTRL_EN;
	val &= ~GENET_TX_DMA_CTRL_RBUF_EN(GENET_DMA_DEFAULT_QUEUE);
	WR4(sc, GENET_TX_DMA_CTRL, val);

	/* Disable RX DMA */
	val = RD4(sc, GENET_RX_DMA_CTRL);
	val &= ~GENET_RX_DMA_CTRL_EN;
	val &= ~GENET_RX_DMA_CTRL_RBUF_EN(GENET_DMA_DEFAULT_QUEUE);
	WR4(sc, GENET_RX_DMA_CTRL, val);
}

static void
rp1_eth_init_txring(struct rp1_eth_softc *sc)
{
	int qid = GENET_DMA_DEFAULT_QUEUE;
	uint32_t val;

	sc->tx_cur = 0;
	sc->tx_next = 0;
	sc->tx_queued = 0;
	sc->tx_prod_idx = 0;
	sc->tx_cons_idx = 0;

	WR4(sc, GENET_TX_SCB_BURST_SIZE, 0x08);

	WR4(sc, GENET_TX_DMA_READ_PTR_LO(qid), 0);
	WR4(sc, GENET_TX_DMA_READ_PTR_HI(qid), 0);
	WR4(sc, GENET_TX_DMA_CONS_INDEX(qid), 0);
	WR4(sc, GENET_TX_DMA_PROD_INDEX(qid), 0);
	WR4(sc, GENET_TX_DMA_RING_BUF_SIZE(qid),
	    (TX_DESC_COUNT << GENET_TX_DMA_RING_BUF_SIZE_DESC_SHIFT) |
	    (MCLBYTES & GENET_TX_DMA_RING_BUF_SIZE_BUF_LEN_MASK));
	WR4(sc, GENET_TX_DMA_START_ADDR_LO(qid), 0);
	WR4(sc, GENET_TX_DMA_START_ADDR_HI(qid), 0);
	WR4(sc, GENET_TX_DMA_END_ADDR_LO(qid),
	    TX_DESC_COUNT * GENET_DMA_DESC_SIZE / 4 - 1);
	WR4(sc, GENET_TX_DMA_END_ADDR_HI(qid), 0);
	WR4(sc, GENET_TX_DMA_MBUF_DONE_THRES(qid), 1);
	WR4(sc, GENET_TX_DMA_FLOW_PERIOD(qid), 0);
	WR4(sc, GENET_TX_DMA_WRITE_PTR_LO(qid), 0);
	WR4(sc, GENET_TX_DMA_WRITE_PTR_HI(qid), 0);

	WR4(sc, GENET_TX_DMA_RING_CFG, __BIT(qid));

	/* Enable TX DMA */
	val = RD4(sc, GENET_TX_DMA_CTRL);
	val |= GENET_TX_DMA_CTRL_EN;
	val |= GENET_TX_DMA_CTRL_RBUF_EN(qid);
	WR4(sc, GENET_TX_DMA_CTRL, val);
}

static void
rp1_eth_init_rxring(struct rp1_eth_softc *sc)
{
	int qid = GENET_DMA_DEFAULT_QUEUE;
	uint32_t val;
	int i;

	sc->rx_cur = 0;
	sc->rx_prod_idx = 0;
	sc->rx_cons_idx = 0;

	WR4(sc, GENET_RX_SCB_BURST_SIZE, 0x08);

	WR4(sc, GENET_RX_DMA_WRITE_PTR_LO(qid), 0);
	WR4(sc, GENET_RX_DMA_WRITE_PTR_HI(qid), 0);
	WR4(sc, GENET_RX_DMA_PROD_INDEX(qid), 0);
	WR4(sc, GENET_RX_DMA_CONS_INDEX(qid), 0);
	WR4(sc, GENET_RX_DMA_RING_BUF_SIZE(qid),
	    (RX_DESC_COUNT << GENET_RX_DMA_RING_BUF_SIZE_DESC_SHIFT) |
	    (MCLBYTES & GENET_RX_DMA_RING_BUF_SIZE_BUF_LEN_MASK));
	WR4(sc, GENET_RX_DMA_START_ADDR_LO(qid), 0);
	WR4(sc, GENET_RX_DMA_START_ADDR_HI(qid), 0);
	WR4(sc, GENET_RX_DMA_END_ADDR_LO(qid),
	    RX_DESC_COUNT * GENET_DMA_DESC_SIZE / 4 - 1);
	WR4(sc, GENET_RX_DMA_END_ADDR_HI(qid), 0);
	WR4(sc, GENET_RX_DMA_XON_XOFF_THRES(qid),
	    (5 << GENET_RX_DMA_XON_XOFF_THRES_LO_SHIFT) | (RX_DESC_COUNT >> 4));
	WR4(sc, GENET_RX_DMA_READ_PTR_LO(qid), 0);
	WR4(sc, GENET_RX_DMA_READ_PTR_HI(qid), 0);

	WR4(sc, GENET_RX_DMA_RING_CFG, __BIT(qid));

	/* Pre-allocate RX buffers */
	for (i = 0; i < RX_DESC_COUNT; i++)
		rp1_eth_newbuf_rx(sc, i);

	/* Enable RX DMA */
	val = RD4(sc, GENET_RX_DMA_CTRL);
	val |= GENET_RX_DMA_CTRL_EN;
	val |= GENET_RX_DMA_CTRL_RBUF_EN(qid);
	WR4(sc, GENET_RX_DMA_CTRL, val);
}

static int
rp1_eth_newbuf_rx(struct rp1_eth_softc *sc, int index)
{
	struct rp1_eth_ring_ent *ent;
	struct mbuf *m;
	bus_dma_segment_t seg;
	int nsegs;

	m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL)
		return (ENOBUFS);

	m->m_pkthdr.len = m->m_len = m->m_ext.ext_size;
	m_adj(m, ETHER_ALIGN);

	ent = &sc->rx_ring[index];

	if (bus_dmamap_load_mbuf_sg(sc->rx_buf_tag, ent->map, m, &seg,
	    &nsegs, BUS_DMA_NOWAIT) != 0) {
		m_freem(m);
		return (ENOBUFS);
	}

	bus_dmamap_sync(sc->rx_buf_tag, ent->map, BUS_DMASYNC_PREREAD);

	ent->mbuf = m;
	WR4(sc, GENET_RX_DESC_ADDRESS_LO(index), (uint32_t)seg.ds_addr);
	WR4(sc, GENET_RX_DESC_ADDRESS_HI(index), (uint32_t)(seg.ds_addr >> 32));

	return (0);
}

static void
rp1_eth_enable(struct rp1_eth_softc *sc)
{
	uint32_t val;

	WR4(sc, GENET_UMAC_MAX_FRAME_LEN, 1536);

	val = RD4(sc, GENET_RBUF_CTRL);
	val |= GENET_RBUF_ALIGN_2B;
	WR4(sc, GENET_RBUF_CTRL, val);

	WR4(sc, GENET_RBUF_TBUF_SIZE_CTRL, 1);

	/* Enable TX and RX */
	val = RD4(sc, GENET_UMAC_CMD);
	val |= GENET_UMAC_CMD_TXEN;
	val |= GENET_UMAC_CMD_RXEN;
	WR4(sc, GENET_UMAC_CMD, val);

	/* Enable interrupts */
	WR4(sc, GENET_INTRL2_CPU_CLEAR_MASK,
	    GENET_IRQ_TXDMA_DONE | GENET_IRQ_RXDMA_DONE);
}

static void
rp1_eth_disable(struct rp1_eth_softc *sc)
{
	uint32_t val;

	/* Disable interrupts */
	WR4(sc, GENET_INTRL2_CPU_SET_MASK, 0xffffffff);
	WR4(sc, GENET_INTRL2_CPU_CLEAR_MASK, 0xffffffff);

	/* Stop RX */
	val = RD4(sc, GENET_UMAC_CMD);
	val &= ~GENET_UMAC_CMD_RXEN;
	WR4(sc, GENET_UMAC_CMD, val);

	/* Stop TX */
	val = RD4(sc, GENET_UMAC_CMD);
	val &= ~GENET_UMAC_CMD_TXEN;
	WR4(sc, GENET_UMAC_CMD, val);
}

static void
rp1_eth_init(void *softc)
{
	struct rp1_eth_softc *sc = softc;

	RP1_ETH_LOCK(sc);
	rp1_eth_init_locked(sc);
	RP1_ETH_UNLOCK(sc);
}

static void
rp1_eth_init_locked(struct rp1_eth_softc *sc)
{
	struct mii_data *mii;

	RP1_ETH_ASSERT_LOCKED(sc);

	if (if_getdrvflags(sc->ifp) & IFF_DRV_RUNNING)
		return;

	/* Set port mode for external GPHY */
	WR4(sc, GENET_SYS_PORT_CTRL, GENET_SYS_PORT_MODE_EXT_GPHY);

	rp1_eth_set_enaddr(sc);
	rp1_eth_setup_rxfilter(sc);

	rp1_eth_init_txring(sc);
	rp1_eth_init_rxring(sc);
	rp1_eth_enable(sc);

	if_setdrvflagbits(sc->ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);

	mii = device_get_softc(sc->miibus);
	mii_mediachg(mii);
	callout_reset(&sc->stat_ch, hz, rp1_eth_tick, sc);
}

static void
rp1_eth_stop(struct rp1_eth_softc *sc)
{
	struct rp1_eth_ring_ent *ent;
	int i;

	RP1_ETH_ASSERT_LOCKED(sc);

	callout_stop(&sc->stat_ch);
	if_setdrvflagbits(sc->ifp, 0, IFF_DRV_RUNNING | IFF_DRV_OACTIVE);

	rp1_eth_reset(sc);
	rp1_eth_disable(sc);
	rp1_eth_dma_disable(sc);

	/* Free TX mbufs */
	for (i = 0; i < TX_DESC_COUNT; i++) {
		ent = &sc->tx_ring[i];
		if (ent->mbuf != NULL) {
			bus_dmamap_sync(sc->tx_buf_tag, ent->map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->tx_buf_tag, ent->map);
			m_freem(ent->mbuf);
			ent->mbuf = NULL;
		}
	}

	/* Free RX mbufs */
	for (i = 0; i < RX_DESC_COUNT; i++) {
		ent = &sc->rx_ring[i];
		if (ent->mbuf != NULL) {
			bus_dmamap_sync(sc->rx_buf_tag, ent->map,
			    BUS_DMASYNC_POSTREAD);
			bus_dmamap_unload(sc->rx_buf_tag, ent->map);
			m_freem(ent->mbuf);
			ent->mbuf = NULL;
		}
	}
}

static void
rp1_eth_start(if_t ifp)
{
	struct rp1_eth_softc *sc = if_getsoftc(ifp);

	RP1_ETH_LOCK(sc);
	rp1_eth_start_locked(sc);
	RP1_ETH_UNLOCK(sc);
}

static void
rp1_eth_start_locked(struct rp1_eth_softc *sc)
{
	struct mbuf *m;
	if_t ifp = sc->ifp;
	int error;

	RP1_ETH_ASSERT_LOCKED(sc);

	if (!sc->link)
		return;

	if ((if_getdrvflags(ifp) & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING)
		return;

	while (1) {
		m = if_dequeue(ifp);
		if (m == NULL)
			break;

		error = rp1_eth_encap(sc, &m);
		if (error != 0) {
			if (error == ENOBUFS)
				if_setdrvflagbits(ifp, IFF_DRV_OACTIVE, 0);
			if (m != NULL) {
				if_sendq_prepend(ifp, m);
			} else {
				if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
			}
			break;
		}

		bpf_mtap_if(ifp, m);
	}
}

static int
rp1_eth_encap(struct rp1_eth_softc *sc, struct mbuf **mp)
{
	bus_dma_segment_t segs[TX_MAX_SEGS];
	struct rp1_eth_ring_ent *ent;
	struct mbuf *m;
	uint32_t status;
	int error, nsegs, i, first, index;

	RP1_ETH_ASSERT_LOCKED(sc);

	if (sc->tx_queued >= TX_DESC_COUNT - TX_MAX_SEGS) {
		return (ENOBUFS);
	}

	m = *mp;

	/* Pull up short headers */
	if (m->m_len < TX_HDR_MIN && m->m_pkthdr.len >= TX_HDR_MIN) {
		m = m_pullup(m, MIN(m->m_pkthdr.len, TX_HDR_MIN));
		if (m == NULL) {
			*mp = NULL;
			return (ENOMEM);
		}
		*mp = m;
	}

	first = sc->tx_cur;
	ent = &sc->tx_ring[first];

	error = bus_dmamap_load_mbuf_sg(sc->tx_buf_tag, ent->map, m, segs,
	    &nsegs, BUS_DMA_NOWAIT);
	if (error == EFBIG) {
		m = m_collapse(m, M_NOWAIT, TX_MAX_SEGS);
		if (m == NULL) {
			*mp = NULL;
			return (ENOMEM);
		}
		*mp = m;
		error = bus_dmamap_load_mbuf_sg(sc->tx_buf_tag, ent->map, m,
		    segs, &nsegs, BUS_DMA_NOWAIT);
	}
	if (error != 0) {
		m_freem(*mp);
		*mp = NULL;
		return (error);
	}
	if (nsegs == 0) {
		m_freem(*mp);
		*mp = NULL;
		return (EIO);
	}

	if (sc->tx_queued + nsegs > TX_DESC_COUNT) {
		bus_dmamap_unload(sc->tx_buf_tag, ent->map);
		return (ENOBUFS);
	}

	bus_dmamap_sync(sc->tx_buf_tag, ent->map, BUS_DMASYNC_PREWRITE);

	index = sc->tx_prod_idx & (TX_DESC_COUNT - 1);
	for (i = 0; i < nsegs; i++) {
		status = GENET_TX_DESC_STATUS_QTAG_MASK;
		if (i == 0) {
			status |= GENET_TX_DESC_STATUS_SOP |
			    GENET_TX_DESC_STATUS_CRC;
		}
		if (i == nsegs - 1)
			status |= GENET_TX_DESC_STATUS_EOP;

		status |= (segs[i].ds_len << GENET_TX_DESC_STATUS_BUFLEN_SHIFT);

		WR4(sc, GENET_TX_DESC_ADDRESS_LO(index),
		    (uint32_t)segs[i].ds_addr);
		WR4(sc, GENET_TX_DESC_ADDRESS_HI(index),
		    (uint32_t)(segs[i].ds_addr >> 32));
		WR4(sc, GENET_TX_DESC_STATUS(index), status);

		sc->tx_queued++;
		sc->tx_cur = TX_NEXT(sc->tx_cur);
		index = TX_NEXT(index);
	}

	sc->tx_prod_idx = (sc->tx_prod_idx + nsegs) & GENET_TX_DMA_PROD_CONS_MASK;
	WR4(sc, GENET_TX_DMA_PROD_INDEX(GENET_DMA_DEFAULT_QUEUE), sc->tx_prod_idx);

	/* Store mbuf in first entry */
	sc->tx_ring[first].mbuf = m;

	return (0);
}

static void
rp1_eth_intr(void *arg)
{
	struct rp1_eth_softc *sc = arg;
	uint32_t status;

	RP1_ETH_LOCK(sc);

	status = RD4(sc, GENET_INTRL2_CPU_STAT);
	status &= ~RD4(sc, GENET_INTRL2_CPU_STAT_MASK);
	WR4(sc, GENET_INTRL2_CPU_CLEAR, status);

	if (status & GENET_IRQ_RXDMA_DONE)
		rp1_eth_rxintr(sc);

	if (status & GENET_IRQ_TXDMA_DONE) {
		rp1_eth_txintr(sc);
		if (!if_sendq_empty(sc->ifp))
			rp1_eth_start_locked(sc);
	}

	RP1_ETH_UNLOCK(sc);

	/* Acknowledge interrupt to RP1 */
	rp1_intr_ack(device_get_parent(sc->dev), sc->irq_num);
}

static void
rp1_eth_txintr(struct rp1_eth_softc *sc)
{
	struct rp1_eth_ring_ent *ent;
	uint32_t cons_idx, total;
	int i;

	RP1_ETH_ASSERT_LOCKED(sc);

	cons_idx = RD4(sc, GENET_TX_DMA_CONS_INDEX(GENET_DMA_DEFAULT_QUEUE)) &
	    GENET_TX_DMA_PROD_CONS_MASK;
	total = (cons_idx - sc->tx_cons_idx) & GENET_TX_DMA_PROD_CONS_MASK;

	for (i = sc->tx_next; sc->tx_queued > 0 && total > 0;
	    i = TX_NEXT(i), total--) {
		ent = &sc->tx_ring[i];
		if (ent->mbuf != NULL) {
			bus_dmamap_sync(sc->tx_buf_tag, ent->map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->tx_buf_tag, ent->map);
			m_freem(ent->mbuf);
			ent->mbuf = NULL;
			if_inc_counter(sc->ifp, IFCOUNTER_OPACKETS, 1);
		}
		sc->tx_queued--;
	}

	sc->tx_next = i;
	sc->tx_cons_idx = cons_idx;

	if (sc->tx_queued < TX_DESC_COUNT)
		if_setdrvflagbits(sc->ifp, 0, IFF_DRV_OACTIVE);
}

static int
rp1_eth_rxintr(struct rp1_eth_softc *sc)
{
	struct rp1_eth_ring_ent *ent;
	struct mbuf *m, *mh, *mt;
	uint32_t prod_idx, status, total;
	int cnt, index, len, npkt;

	RP1_ETH_ASSERT_LOCKED(sc);

	mh = mt = NULL;
	cnt = npkt = 0;

	prod_idx = RD4(sc, GENET_RX_DMA_PROD_INDEX(GENET_DMA_DEFAULT_QUEUE)) &
	    GENET_RX_DMA_PROD_CONS_MASK;
	total = (prod_idx - sc->rx_cons_idx) & GENET_RX_DMA_PROD_CONS_MASK;

	index = sc->rx_cons_idx & (RX_DESC_COUNT - 1);

	for (; total > 0; total--) {
		ent = &sc->rx_ring[index];

		bus_dmamap_sync(sc->rx_buf_tag, ent->map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->rx_buf_tag, ent->map);

		m = ent->mbuf;
		status = RD4(sc, GENET_RX_DESC_STATUS(index));

		len = (status & GENET_RX_DESC_STATUS_BUFLEN_MASK) >>
		    GENET_RX_DESC_STATUS_BUFLEN_SHIFT;

		/* Check for errors */
		if ((status & (GENET_RX_DESC_STATUS_SOP |
		    GENET_RX_DESC_STATUS_EOP | GENET_RX_DESC_STATUS_RX_ERROR))
		    != (GENET_RX_DESC_STATUS_SOP | GENET_RX_DESC_STATUS_EOP)) {
			if_inc_counter(sc->ifp, IFCOUNTER_IERRORS, 1);
			m_freem(m);
			ent->mbuf = NULL;
			rp1_eth_newbuf_rx(sc, index);
			goto next;
		}

		/* Allocate new buffer */
		if (rp1_eth_newbuf_rx(sc, index) != 0) {
			if_inc_counter(sc->ifp, IFCOUNTER_IQDROPS, 1);
			/* Reuse old buffer */
			bus_dmamap_load_mbuf_sg(sc->rx_buf_tag, ent->map, m,
			    NULL, NULL, BUS_DMA_NOWAIT);
			goto next;
		}

		/* Adjust for alignment padding */
		if (len > ETHER_ALIGN) {
			m_adj(m, ETHER_ALIGN);
			len -= ETHER_ALIGN;
		}

		m->m_pkthdr.rcvif = sc->ifp;
		m->m_pkthdr.len = len;
		m->m_len = len;

		/* Check RX checksum */
		if ((status & GENET_RX_DESC_STATUS_CKSUM_OK) &&
		    (if_getcapenable(sc->ifp) & IFCAP_RXCSUM)) {
			m->m_pkthdr.csum_flags = CSUM_DATA_VALID |
			    CSUM_PSEUDO_HDR;
			m->m_pkthdr.csum_data = 0xffff;
		}

		if_inc_counter(sc->ifp, IFCOUNTER_IPACKETS, 1);

		m->m_nextpkt = NULL;
		if (mh == NULL)
			mh = m;
		else
			mt->m_nextpkt = m;
		mt = m;
		cnt++;
		npkt++;

next:
		index = RX_NEXT(index);
		sc->rx_cons_idx = (sc->rx_cons_idx + 1) &
		    GENET_RX_DMA_PROD_CONS_MASK;
		WR4(sc, GENET_RX_DMA_CONS_INDEX(GENET_DMA_DEFAULT_QUEUE),
		    sc->rx_cons_idx);

		if (cnt >= RX_BATCH) {
			RP1_ETH_UNLOCK(sc);
			if_input(sc->ifp, mh);
			RP1_ETH_LOCK(sc);
			mh = mt = NULL;
			cnt = 0;
		}
	}

	if (mh != NULL) {
		RP1_ETH_UNLOCK(sc);
		if_input(sc->ifp, mh);
		RP1_ETH_LOCK(sc);
	}

	return (npkt);
}

static void
rp1_eth_tick(void *softc)
{
	struct rp1_eth_softc *sc = softc;
	struct mii_data *mii;
	int link;

	RP1_ETH_ASSERT_LOCKED(sc);

	if (!(if_getdrvflags(sc->ifp) & IFF_DRV_RUNNING))
		return;

	mii = device_get_softc(sc->miibus);
	link = sc->link;
	mii_tick(mii);

	if (sc->link && !link)
		rp1_eth_start_locked(sc);

	callout_reset(&sc->stat_ch, hz, rp1_eth_tick, sc);
}

static void
rp1_eth_link_task(void *arg, int pending)
{
	struct rp1_eth_softc *sc = arg;

	RP1_ETH_LOCK(sc);
	rp1_eth_update_link(sc);
	RP1_ETH_UNLOCK(sc);
}

static void
rp1_eth_update_link(struct rp1_eth_softc *sc)
{
	struct mii_data *mii;
	uint32_t val, speed;

	RP1_ETH_ASSERT_LOCKED(sc);

	if (!(if_getdrvflags(sc->ifp) & IFF_DRV_RUNNING))
		return;

	mii = device_get_softc(sc->miibus);

	if ((mii->mii_media_status & (IFM_ACTIVE | IFM_AVALID)) ==
	    (IFM_ACTIVE | IFM_AVALID)) {
		switch (IFM_SUBTYPE(mii->mii_media_active)) {
		case IFM_1000_T:
		case IFM_1000_SX:
			speed = GENET_UMAC_CMD_SPEED_1000;
			sc->link = 1;
			break;
		case IFM_100_TX:
			speed = GENET_UMAC_CMD_SPEED_100;
			sc->link = 1;
			break;
		case IFM_10_T:
			speed = GENET_UMAC_CMD_SPEED_10;
			sc->link = 1;
			break;
		default:
			sc->link = 0;
			return;
		}
	} else {
		sc->link = 0;
		return;
	}

	val = RD4(sc, GENET_EXT_RGMII_OOB_CTRL);
	val &= ~GENET_EXT_RGMII_OOB_OOB_DISABLE;
	val |= GENET_EXT_RGMII_OOB_RGMII_LINK;
	val |= GENET_EXT_RGMII_OOB_RGMII_MODE_EN;
	WR4(sc, GENET_EXT_RGMII_OOB_CTRL, val);

	val = RD4(sc, GENET_UMAC_CMD);
	val &= ~GENET_UMAC_CMD_SPEED;
	val |= speed;
	WR4(sc, GENET_UMAC_CMD, val);
}

static void
rp1_eth_set_enaddr(struct rp1_eth_softc *sc)
{
	uint8_t *enaddr;
	uint32_t val;

	RP1_ETH_ASSERT_LOCKED(sc);

	enaddr = if_getlladdr(sc->ifp);
	val = enaddr[3] | (enaddr[2] << 8) | (enaddr[1] << 16) |
	    (enaddr[0] << 24);
	WR4(sc, GENET_UMAC_MAC0, val);
	val = enaddr[5] | (enaddr[4] << 8);
	WR4(sc, GENET_UMAC_MAC1, val);
}

static u_int
rp1_eth_setup_multi(void *arg, struct sockaddr_dl *sdl, u_int count)
{
	struct rp1_eth_softc *sc = arg;
	uint32_t addr0, addr1;

	/* count + 2 to account for unicast and broadcast */
	addr0 = (LLADDR(sdl)[0] << 8) | LLADDR(sdl)[1];
	addr1 = (LLADDR(sdl)[2] << 24) | (LLADDR(sdl)[3] << 16) |
	    (LLADDR(sdl)[4] << 8) | LLADDR(sdl)[5];

	WR4(sc, GENET_UMAC_MDF_ADDR0(count + 2), addr0);
	WR4(sc, GENET_UMAC_MDF_ADDR1(count + 2), addr1);

	return (1);
}

static void
rp1_eth_setup_rxfilter(struct rp1_eth_softc *sc)
{
	static const uint8_t bcast[ETHER_ADDR_LEN] =
	    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	if_t ifp = sc->ifp;
	uint32_t cmd, mdf_ctrl, addr0, addr1;
	u_int n;

	RP1_ETH_ASSERT_LOCKED(sc);

	cmd = RD4(sc, GENET_UMAC_CMD);

	n = if_llmaddr_count(ifp) + 2;

	if (n > GENET_MAX_MDF_FILTER)
		if_setflagbits(ifp, IFF_ALLMULTI, 0);
	else
		if_setflagbits(ifp, 0, IFF_ALLMULTI);

	if ((if_getflags(ifp) & (IFF_PROMISC | IFF_ALLMULTI)) != 0) {
		cmd |= GENET_UMAC_CMD_PROMISC;
		mdf_ctrl = 0;
	} else {
		cmd &= ~GENET_UMAC_CMD_PROMISC;

		/* Program broadcast address */
		addr0 = (bcast[0] << 8) | bcast[1];
		addr1 = (bcast[2] << 24) | (bcast[3] << 16) |
		    (bcast[4] << 8) | bcast[5];
		WR4(sc, GENET_UMAC_MDF_ADDR0(0), addr0);
		WR4(sc, GENET_UMAC_MDF_ADDR1(0), addr1);

		/* Program unicast address */
		addr0 = (if_getlladdr(ifp)[0] << 8) | if_getlladdr(ifp)[1];
		addr1 = (if_getlladdr(ifp)[2] << 24) |
		    (if_getlladdr(ifp)[3] << 16) |
		    (if_getlladdr(ifp)[4] << 8) | if_getlladdr(ifp)[5];
		WR4(sc, GENET_UMAC_MDF_ADDR0(1), addr0);
		WR4(sc, GENET_UMAC_MDF_ADDR1(1), addr1);

		/* Program multicast addresses */
		if_foreach_llmaddr(ifp, rp1_eth_setup_multi, sc);

		mdf_ctrl = (__BIT(GENET_MAX_MDF_FILTER) - 1) &
		    ~(__BIT(GENET_MAX_MDF_FILTER - n) - 1);
	}

	WR4(sc, GENET_UMAC_CMD, cmd);
	WR4(sc, GENET_UMAC_MDF_CTRL, mdf_ctrl);
}

static int
rp1_eth_ioctl(if_t ifp, u_long cmd, caddr_t data)
{
	struct rp1_eth_softc *sc;
	struct mii_data *mii;
	struct ifreq *ifr;
	int error, flags, mask;

	sc = if_getsoftc(ifp);
	ifr = (struct ifreq *)data;
	error = 0;

	switch (cmd) {
	case SIOCSIFFLAGS:
		RP1_ETH_LOCK(sc);
		if (if_getflags(ifp) & IFF_UP) {
			if (if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
				flags = if_getflags(ifp) ^ sc->if_flags;
				if ((flags & (IFF_PROMISC | IFF_ALLMULTI)) != 0)
					rp1_eth_setup_rxfilter(sc);
			} else {
				rp1_eth_init_locked(sc);
			}
		} else {
			if (if_getdrvflags(ifp) & IFF_DRV_RUNNING)
				rp1_eth_stop(sc);
		}
		sc->if_flags = if_getflags(ifp);
		RP1_ETH_UNLOCK(sc);
		break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if (if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
			RP1_ETH_LOCK(sc);
			rp1_eth_setup_rxfilter(sc);
			RP1_ETH_UNLOCK(sc);
		}
		break;

	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		mii = device_get_softc(sc->miibus);
		error = ifmedia_ioctl(ifp, ifr, &mii->mii_media, cmd);
		break;

	case SIOCSIFCAP:
		mask = ifr->ifr_reqcap ^ if_getcapenable(ifp);
		if (mask & IFCAP_RXCSUM)
			if_togglecapenable(ifp, IFCAP_RXCSUM);
		if (mask & IFCAP_TXCSUM)
			if_togglecapenable(ifp, IFCAP_TXCSUM);
		if (if_getcapenable(ifp) & IFCAP_TXCSUM)
			if_sethwassist(ifp, CSUM_TCP | CSUM_UDP);
		else
			if_sethwassist(ifp, 0);
		break;

	default:
		error = ether_ioctl(ifp, cmd, data);
		break;
	}

	return (error);
}

static void
rp1_eth_media_status(if_t ifp, struct ifmediareq *ifmr)
{
	struct rp1_eth_softc *sc;
	struct mii_data *mii;

	sc = if_getsoftc(ifp);
	mii = device_get_softc(sc->miibus);

	RP1_ETH_LOCK(sc);
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
	RP1_ETH_UNLOCK(sc);
}

static int
rp1_eth_media_change(if_t ifp)
{
	struct rp1_eth_softc *sc;
	struct mii_data *mii;
	int error;

	sc = if_getsoftc(ifp);
	mii = device_get_softc(sc->miibus);

	RP1_ETH_LOCK(sc);
	error = mii_mediachg(mii);
	RP1_ETH_UNLOCK(sc);

	return (error);
}

/* MII bus methods */
static int
rp1_eth_miibus_readreg(device_t dev, int phy, int reg)
{
	struct rp1_eth_softc *sc;
	int retry, val;

	sc = device_get_softc(dev);

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
	struct rp1_eth_softc *sc;

	sc = device_get_softc(dev);
	taskqueue_enqueue(taskqueue_swi, &sc->link_task);
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
