/*	$OpenBSD: bwfm.c,v 1.100 2024/04/14 03:23:05 jsg Exp $	*/
/*
 * Copyright (c) 2010-2016 Broadcom Corporation
 * Copyright (c) 2016,2017 Patrick Wildt <patrick@blueri.se>
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
 * Ported from OpenBSD/NetBSD for FreeBSD
 * Broadcom FullMAC WiFi driver
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
#include <sys/sockio.h>
#include <sys/taskqueue.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/ethernet.h>

#include <netinet/in.h>
#include <netinet/if_ether.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_ratectl.h>
#include <net80211/ieee80211_radiotap.h>

#include <dev/bwfm/bwfmreg.h>
#include <dev/bwfm/bwfmvar.h>

#ifdef BWFM_DEBUG
int bwfm_debug = 0;
#define DPRINTF(x)	do { if (bwfm_debug) printf x; } while (0)
#define DPRINTFN(n, x)	do { if (bwfm_debug >= (n)) printf x; } while (0)
#else
#define DPRINTF(x)
#define DPRINTFN(n, x)
#endif

/* Memory allocation tag */
static MALLOC_DEFINE(M_BWFM, "bwfm", "Broadcom FullMAC WiFi driver");

/* Function prototypes */
static void	bwfm_init(struct bwfm_softc *);
static void	bwfm_stop(struct bwfm_softc *);
static int	bwfm_preinit(struct bwfm_softc *);
static void	bwfm_scan(struct bwfm_softc *);
static void	bwfm_connect(struct bwfm_softc *);
static void	bwfm_get_station(struct bwfm_softc *, struct ieee80211_node *);

static void	bwfm_task(void *, int);
static int	bwfm_newstate(struct ieee80211vap *, enum ieee80211_state, int);
static void	bwfm_newstate_task(struct bwfm_softc *, void *);
static void	bwfm_set_key_task(struct bwfm_softc *, void *);
static void	bwfm_delete_key_task(struct bwfm_softc *, void *);

static int	bwfm_media_change(struct ifnet *);
static void	bwfm_parent(struct ieee80211com *);
static int	bwfm_transmit(struct ieee80211com *, struct mbuf *);
static void	bwfm_scan_start(struct ieee80211com *);
static void	bwfm_scan_end(struct ieee80211com *);
static void	bwfm_set_channel(struct ieee80211com *);
static void	bwfm_update_mcast(struct ieee80211com *);
static struct ieee80211vap *bwfm_vap_create(struct ieee80211com *,
		    const char [IFNAMSIZ], int, enum ieee80211_opmode, int,
		    const uint8_t [IEEE80211_ADDR_LEN],
		    const uint8_t [IEEE80211_ADDR_LEN]);
static void	bwfm_vap_delete(struct ieee80211vap *);
static struct ieee80211_node *bwfm_node_alloc(struct ieee80211vap *,
		    const uint8_t mac[IEEE80211_ADDR_LEN]);

static int	bwfm_set_iovar(struct bwfm_softc *, const char *, void *, size_t);
static int	bwfm_get_iovar(struct bwfm_softc *, const char *, void *, size_t);
static int	bwfm_send_cmd(struct bwfm_softc *, int, void *, size_t);
static int	bwfm_query_cmd(struct bwfm_softc *, int, void *, size_t);

/*
 * Supported rates for 802.11b/g
 */
static const struct ieee80211_rateset bwfm_rateset_11b =
    { 4, { 2, 4, 11, 22 } };
static const struct ieee80211_rateset bwfm_rateset_11g =
    { 12, { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108 } };

/*
 * BCDC protocol operations
 */
const struct bwfm_proto_ops bwfm_proto_bcdc_ops = {
	.proto_query_dcmd = bwfm_proto_bcdc_query_dcmd,
	.proto_set_dcmd = bwfm_proto_bcdc_set_dcmd,
	.proto_rx = bwfm_proto_bcdc_rx,
	.proto_rxctl = bwfm_proto_bcdc_rxctl,
};

/*
 * VAP structure for bwfm driver
 */
struct bwfm_vap {
	struct ieee80211vap	bv_vap;
	int			(*bv_newstate)(struct ieee80211vap *,
				    enum ieee80211_state, int);
};

#define	BWFM_VAP(vap)	((struct bwfm_vap *)(vap))

/*
 * Attach the driver to ieee80211 stack
 */
void
bwfm_attach(struct bwfm_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;

	DPRINTF(("%s: bwfm_attach\n", device_get_nameunit(sc->sc_dev)));

	/* Initialize mutex */
	mtx_init(&sc->sc_mtx, device_get_nameunit(sc->sc_dev),
	    MTX_NETWORK_LOCK, MTX_DEF);

	/* Create taskqueue for deferred work */
	sc->sc_taskq = taskqueue_create("bwfm_taskq", M_WAITOK,
	    taskqueue_thread_enqueue, &sc->sc_taskq);
	taskqueue_start_threads(&sc->sc_taskq, 1, PI_NET, "%s taskq",
	    device_get_nameunit(sc->sc_dev));

	TASK_INIT(&sc->sc_task, 0, bwfm_task, sc);

	/* Initialize command queue */
	sc->sc_cmdq.cur = 0;
	sc->sc_cmdq.next = 0;
	sc->sc_cmdq.queued = 0;

	/* Setup 802.11 common structure */
	ic->ic_softc = sc;
	ic->ic_name = device_get_nameunit(sc->sc_dev);
	ic->ic_phytype = IEEE80211_T_OFDM;	/* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA;

	/* Set supported capabilities */
	ic->ic_caps =
	    IEEE80211_C_STA |		/* station mode supported */
	    IEEE80211_C_BGSCAN |	/* bg scanning supported */
	    IEEE80211_C_SHPREAMBLE |	/* short preamble supported */
	    IEEE80211_C_SHSLOT |	/* short slot time supported */
	    IEEE80211_C_WME |		/* WME supported */
	    IEEE80211_C_WPA;		/* WPA/RSN supported */

	/* Set HT capabilities if supported */
	if (sc->sc_io_type >= BWFM_IO_TYPE_D11N) {
		ic->ic_htcaps =
		    IEEE80211_HTCAP_SMPS_OFF |
		    IEEE80211_HTCAP_SHORTGI20 |
		    IEEE80211_HTCAP_CHWIDTH40 |
		    IEEE80211_HTCAP_SHORTGI40 |
		    IEEE80211_HTC_HT;	/* Enable HT operation */
	}

	/* Copy MAC address */
	IEEE80211_ADDR_COPY(ic->ic_macaddr, sc->sc_lladdr);

	/*
	 * Initialize channels. The bwfm firmware handles regulatory,
	 * but we need to set up basic channel list for net80211.
	 */
	bwfm_init_channels(ic);

	/* Set supported rates */
	memcpy(&ic->ic_sup_rates[IEEE80211_MODE_11B], &bwfm_rateset_11b,
	    sizeof(bwfm_rateset_11b));
	memcpy(&ic->ic_sup_rates[IEEE80211_MODE_11G], &bwfm_rateset_11g,
	    sizeof(bwfm_rateset_11g));

	/* Attach to 802.11 stack */
	ieee80211_ifattach(ic);

	/* Override methods */
	ic->ic_vap_create = bwfm_vap_create;
	ic->ic_vap_delete = bwfm_vap_delete;
	ic->ic_node_alloc = bwfm_node_alloc;
	ic->ic_scan_start = bwfm_scan_start;
	ic->ic_scan_end = bwfm_scan_end;
	ic->ic_set_channel = bwfm_set_channel;
	ic->ic_update_mcast = bwfm_update_mcast;
	ic->ic_parent = bwfm_parent;
	ic->ic_transmit = bwfm_transmit;

	ieee80211_announce(ic);
}

/*
 * Detach the driver
 */
void
bwfm_detach(struct bwfm_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;

	BWFM_LOCK(sc);
	bwfm_stop(sc);
	BWFM_UNLOCK(sc);

	/* Drain taskqueue */
	if (sc->sc_taskq != NULL) {
		taskqueue_drain(sc->sc_taskq, &sc->sc_task);
		taskqueue_free(sc->sc_taskq);
		sc->sc_taskq = NULL;
	}

	/* Detach from ieee80211 */
	ieee80211_ifdetach(ic);

	/* Free CLM data */
	if (sc->sc_clm != NULL) {
		free(sc->sc_clm, M_BWFM);
		sc->sc_clm = NULL;
	}

	mtx_destroy(&sc->sc_mtx);
}

/*
 * Initialize channels based on what firmware reports
 */
void
bwfm_init_channels(struct ieee80211com *ic)
{
	uint8_t bands[IEEE80211_MODE_BYTES];
	int cbw_flags = 0;

	/* Check for HT40 support */
	if (ic->ic_htcaps & IEEE80211_HTCAP_CHWIDTH40)
		cbw_flags |= NET80211_CBW_FLAG_HT40;

	/* Add 2.4GHz channels */
	memset(bands, 0, sizeof(bands));
	setbit(bands, IEEE80211_MODE_11B);
	setbit(bands, IEEE80211_MODE_11G);
	if (ic->ic_htcaps & IEEE80211_HTC_HT)
		setbit(bands, IEEE80211_MODE_11NG);
	ieee80211_add_channels_default_2ghz(ic->ic_channels, IEEE80211_CHAN_MAX,
	    &ic->ic_nchans, bands, cbw_flags);

	/* Add 5GHz channels - basic set */
	memset(bands, 0, sizeof(bands));
	setbit(bands, IEEE80211_MODE_11A);
	if (ic->ic_htcaps & IEEE80211_HTC_HT)
		setbit(bands, IEEE80211_MODE_11NA);

	/* UNII-1 (36, 40, 44, 48) */
	{
		static const uint8_t unii1[] = { 36, 40, 44, 48 };
		ieee80211_add_channel_list_5ghz(ic->ic_channels, IEEE80211_CHAN_MAX,
		    &ic->ic_nchans, unii1, nitems(unii1), bands, cbw_flags);
	}
	/* UNII-2A (52, 56, 60, 64) */
	{
		static const uint8_t unii2a[] = { 52, 56, 60, 64 };
		ieee80211_add_channel_list_5ghz(ic->ic_channels, IEEE80211_CHAN_MAX,
		    &ic->ic_nchans, unii2a, nitems(unii2a), bands, cbw_flags);
	}
	/* UNII-2C (100-144) */
	{
		static const uint8_t unii2c[] = {
		    100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144
		};
		ieee80211_add_channel_list_5ghz(ic->ic_channels, IEEE80211_CHAN_MAX,
		    &ic->ic_nchans, unii2c, nitems(unii2c), bands, cbw_flags);
	}
	/* UNII-3 (149, 153, 157, 161, 165) */
	{
		static const uint8_t unii3[] = { 149, 153, 157, 161, 165 };
		ieee80211_add_channel_list_5ghz(ic->ic_channels, IEEE80211_CHAN_MAX,
		    &ic->ic_nchans, unii3, nitems(unii3), bands, cbw_flags);
	}
}

/*
 * Create a VAP
 */
static struct ieee80211vap *
bwfm_vap_create(struct ieee80211com *ic, const char name[IFNAMSIZ], int unit,
    enum ieee80211_opmode opmode, int flags,
    const uint8_t bssid[IEEE80211_ADDR_LEN],
    const uint8_t mac[IEEE80211_ADDR_LEN])
{
	struct bwfm_softc *sc = ic->ic_softc;
	struct bwfm_vap *bvp;
	struct ieee80211vap *vap;

	/* Only one VAP supported */
	if (!TAILQ_EMPTY(&ic->ic_vaps))
		return NULL;

	/* Only station mode for now */
	if (opmode != IEEE80211_M_STA) {
		device_printf(sc->sc_dev,
		    "only station mode supported\n");
		return NULL;
	}

	bvp = malloc(sizeof(struct bwfm_vap), M_80211_VAP, M_WAITOK | M_ZERO);
	vap = &bvp->bv_vap;

	ieee80211_vap_setup(ic, vap, name, unit, opmode, flags, bssid);

	/* Override state machine */
	bvp->bv_newstate = vap->iv_newstate;
	vap->iv_newstate = bwfm_newstate;

	/* Initialize rate control */
	ieee80211_ratectl_init(vap);

	/* Complete VAP setup */
	ieee80211_vap_attach(vap, bwfm_media_change, ieee80211_media_status,
	    mac);

	sc->sc_vap = vap;
	ic->ic_opmode = opmode;

	return vap;
}

/*
 * Delete a VAP
 */
static void
bwfm_vap_delete(struct ieee80211vap *vap)
{
	struct bwfm_softc *sc = vap->iv_ic->ic_softc;
	struct bwfm_vap *bvp = BWFM_VAP(vap);

	sc->sc_vap = NULL;

	ieee80211_ratectl_deinit(vap);
	ieee80211_vap_detach(vap);
	free(bvp, M_80211_VAP);
}

/*
 * Allocate a node
 */
static struct ieee80211_node *
bwfm_node_alloc(struct ieee80211vap *vap,
    const uint8_t mac[IEEE80211_ADDR_LEN])
{
	struct ieee80211_node *ni;

	ni = malloc(sizeof(struct ieee80211_node), M_80211_NODE,
	    M_NOWAIT | M_ZERO);
	return ni;
}

/*
 * Pre-initialization: load firmware, setup chip
 */
static int
bwfm_preinit(struct bwfm_softc *sc)
{
	int error;

	/* Call bus-specific pre-init */
	if (sc->sc_bus_ops->bs_preinit != NULL) {
		error = sc->sc_bus_ops->bs_preinit(sc);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "bus pre-init failed: %d\n", error);
			return error;
		}
	}

	/* Get MAC address from firmware */
	error = bwfm_get_iovar(sc, "cur_etheraddr", sc->sc_lladdr,
	    sizeof(sc->sc_lladdr));
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "cannot get MAC address: %d\n", error);
		return error;
	}

	/* Set country code if specified */
	if (sc->sc_ccode[0] != '\0') {
		struct bwfm_country_code cc;

		memset(&cc, 0, sizeof(cc));
		memcpy(cc.country_abbrev, sc->sc_ccode, 2);
		cc.rev = 0;
		memcpy(cc.ccode, sc->sc_ccode, 2);

		error = bwfm_set_iovar(sc, "country", &cc, sizeof(cc));
		if (error != 0)
			DPRINTF(("%s: cannot set country: %d\n",
			    device_get_nameunit(sc->sc_dev), error));
	}

	/* Enable events */
	error = bwfm_enable_events(sc);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "cannot enable events: %d\n", error);
		return error;
	}

	/* Get scan version */
	error = bwfm_get_iovar(sc, "scan_ver", &sc->sc_scan_ver,
	    sizeof(sc->sc_scan_ver));
	if (error != 0)
		sc->sc_scan_ver = 0;

	return 0;
}

/*
 * Initialize the device
 */
static void
bwfm_init(struct bwfm_softc *sc)
{
	int error;

	BWFM_LOCK_ASSERT(sc);

	if (sc->sc_if_flags & BWFM_RUNNING)
		return;

	error = bwfm_preinit(sc);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "pre-init failed: %d\n", error);
		return;
	}

	/* Enable the device */
	sc->sc_if_flags |= BWFM_RUNNING;
}

/*
 * Stop the device
 */
static void
bwfm_stop(struct bwfm_softc *sc)
{
	BWFM_LOCK_ASSERT(sc);

	if (!(sc->sc_if_flags & BWFM_RUNNING))
		return;

	/* Call bus-specific stop */
	if (sc->sc_bus_ops->bs_stop != NULL)
		sc->sc_bus_ops->bs_stop(sc);

	sc->sc_if_flags &= ~BWFM_RUNNING;
}

/*
 * Parent state change
 */
static void
bwfm_parent(struct ieee80211com *ic)
{
	struct bwfm_softc *sc = ic->ic_softc;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);

	BWFM_LOCK(sc);
	if (ic->ic_nrunning > 0) {
		if (!(sc->sc_if_flags & BWFM_RUNNING))
			bwfm_init(sc);
	} else {
		if (sc->sc_if_flags & BWFM_RUNNING)
			bwfm_stop(sc);
	}
	BWFM_UNLOCK(sc);

	if (vap != NULL)
		ieee80211_start_all(ic);
}

/*
 * Transmit a frame
 */
static int
bwfm_transmit(struct ieee80211com *ic, struct mbuf *m)
{
	struct bwfm_softc *sc = ic->ic_softc;
	int error;

	BWFM_LOCK(sc);
	if (!(sc->sc_if_flags & BWFM_RUNNING)) {
		BWFM_UNLOCK(sc);
		m_freem(m);
		return ENETDOWN;
	}

	/* Check if bus can transmit */
	if (sc->sc_bus_ops->bs_txcheck != NULL) {
		error = sc->sc_bus_ops->bs_txcheck(sc);
		if (error != 0) {
			BWFM_UNLOCK(sc);
			m_freem(m);
			return error;
		}
	}

	/* Send via bus */
	error = sc->sc_bus_ops->bs_txdata(sc, m);
	BWFM_UNLOCK(sc);

	return error;
}

/*
 * Media change callback
 */
static int
bwfm_media_change(struct ifnet *ifp)
{
	return ieee80211_media_change(ifp);
}

/*
 * Scan start callback
 */
static void
bwfm_scan_start(struct ieee80211com *ic)
{
	struct bwfm_softc *sc = ic->ic_softc;

	DPRINTF(("%s: scan start\n", device_get_nameunit(sc->sc_dev)));

	BWFM_LOCK(sc);
	bwfm_scan(sc);
	BWFM_UNLOCK(sc);
}

/*
 * Scan end callback
 */
static void
bwfm_scan_end(struct ieee80211com *ic)
{
#ifdef BWFM_DEBUG
	struct bwfm_softc *sc = ic->ic_softc;
	DPRINTF(("%s: scan end\n", device_get_nameunit(sc->sc_dev)));
#else
	(void)ic;
#endif
}

/*
 * Set channel callback
 */
static void
bwfm_set_channel(struct ieee80211com *ic)
{
	/* Channel is set by firmware during scan/connect */
}

/*
 * Update multicast filter
 */
static void
bwfm_update_mcast(struct ieee80211com *ic)
{
	/* FullMAC handles multicast filtering */
}

/*
 * State change handler
 */
static int
bwfm_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
	struct bwfm_vap *bvp = BWFM_VAP(vap);
	struct ieee80211com *ic = vap->iv_ic;
	struct bwfm_softc *sc = ic->ic_softc;
	struct bwfm_host_cmd_newstate cmd;

	DPRINTF(("%s: %s -> %s\n", device_get_nameunit(sc->sc_dev),
	    ieee80211_state_name[vap->iv_state],
	    ieee80211_state_name[nstate]));

	/* Queue state change for taskqueue */
	cmd.state = nstate;
	cmd.arg = arg;

	bwfm_do_async(sc, bwfm_newstate_task, &cmd, sizeof(cmd));

	return bvp->bv_newstate(vap, nstate, arg);
}

/*
 * Queue an async task
 */
void
bwfm_do_async(struct bwfm_softc *sc,
    void (*func)(struct bwfm_softc *, void *), void *arg, size_t arglen)
{
	struct bwfm_host_cmd_ring *ring = &sc->sc_cmdq;
	struct bwfm_host_cmd *cmd;
	int next;

	BWFM_LOCK_ASSERT(sc);

	next = (ring->next + 1) % BWFM_HOST_CMD_RING_COUNT;
	if (next == ring->cur) {
		/* Ring full */
		return;
	}

	cmd = &ring->cmd[ring->next];
	cmd->type = BWFM_HOST_CMD_NEWSTATE;	/* Will be overridden */
	if (arg != NULL && arglen > 0)
		memcpy(&cmd->cmd, arg, arglen);

	ring->next = next;
	ring->queued++;

	taskqueue_enqueue(sc->sc_taskq, &sc->sc_task);
}

/*
 * Process async tasks
 */
static void
bwfm_task(void *arg, int pending)
{
	struct bwfm_softc *sc = arg;
	struct bwfm_host_cmd_ring *ring = &sc->sc_cmdq;
	struct bwfm_host_cmd *cmd;

	BWFM_LOCK(sc);
	while (ring->queued > 0) {
		cmd = &ring->cmd[ring->cur];

		switch (cmd->type) {
		case BWFM_HOST_CMD_NEWSTATE:
			bwfm_newstate_task(sc, &cmd->cmd.newstate);
			break;
		case BWFM_HOST_CMD_KEY_SET:
			bwfm_set_key_task(sc, &cmd->cmd.key);
			break;
		case BWFM_HOST_CMD_KEY_DEL:
			bwfm_delete_key_task(sc, &cmd->cmd.key);
			break;
		}

		ring->cur = (ring->cur + 1) % BWFM_HOST_CMD_RING_COUNT;
		ring->queued--;
	}
	BWFM_UNLOCK(sc);
}

/*
 * Handle state change in taskqueue
 */
static void
bwfm_newstate_task(struct bwfm_softc *sc, void *arg)
{
	struct bwfm_host_cmd_newstate *cmd = arg;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);

	BWFM_LOCK_ASSERT(sc);

	if (vap == NULL)
		return;

	switch (cmd->state) {
	case IEEE80211_S_INIT:
		/* Disconnect if connected */
		bwfm_send_cmd(sc, BWFM_C_DISASSOC, NULL, 0);
		break;

	case IEEE80211_S_SCAN:
		/* Scan is handled by bwfm_scan_start */
		break;

	case IEEE80211_S_AUTH:
		bwfm_connect(sc);
		break;

	case IEEE80211_S_ASSOC:
		/* Association handled by firmware */
		break;

	case IEEE80211_S_RUN:
		/* We're now associated */
		if (vap->iv_bss != NULL)
			bwfm_get_station(sc, vap->iv_bss);
		break;

	default:
		break;
	}
}

/*
 * Start a scan
 */
static void
bwfm_scan(struct bwfm_softc *sc)
{
	struct bwfm_escan_params *eparams;
	struct bwfm_scan_params *sparams;
	size_t len;
	int error;

	BWFM_LOCK_ASSERT(sc);

	len = sizeof(*eparams) + BWFM_SCAN_PARAMS_FIXED_SIZE;
	eparams = malloc(len, M_BWFM, M_NOWAIT | M_ZERO);
	if (eparams == NULL)
		return;

	sparams = &eparams->escan_params;

	/* Set broadcast BSSID for scan */
	memset(sparams->bssid, 0xff, sizeof(sparams->bssid));

	/* Scan type: active */
	sparams->bss_type = 2;		/* any */
	sparams->scan_type = 0;		/* active */
	sparams->nprobes = htole32(-1);
	sparams->active_time = htole32(-1);
	sparams->passive_time = htole32(-1);
	sparams->home_time = htole32(-1);
	sparams->channel_num = htole32(0);	/* all channels */

	eparams->version = htole32(BWFM_ESCAN_REQ_VERSION);
	eparams->action = htole16(1);	/* start */
	eparams->sync_id = htole16(0x1234);

	error = bwfm_set_iovar(sc, "escan", eparams, len);
	free(eparams, M_BWFM);

	if (error != 0)
		DPRINTF(("%s: escan failed: %d\n",
		    device_get_nameunit(sc->sc_dev), error));
}

/*
 * Connect to a network
 */
static void
bwfm_connect(struct bwfm_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct ieee80211_node *ni;
	struct bwfm_ext_join_params *params;
	size_t len;
	int error;

	BWFM_LOCK_ASSERT(sc);

	if (vap == NULL)
		return;

	ni = vap->iv_bss;
	if (ni == NULL)
		return;

	len = sizeof(*params);
	params = malloc(len, M_BWFM, M_NOWAIT | M_ZERO);
	if (params == NULL)
		return;

	/* Set SSID */
	params->ssid.len = htole32(ni->ni_esslen);
	memcpy(params->ssid.ssid, ni->ni_essid, ni->ni_esslen);

	/* Scan parameters */
	params->scan.scan_type = -1;
	params->scan.nprobes = htole32(-1);
	params->scan.active_time = htole32(-1);
	params->scan.passive_time = htole32(-1);
	params->scan.home_time = htole32(-1);

	/* Association parameters */
	params->assoc.chanspec_num = htole32(0);

	error = bwfm_set_iovar(sc, "join", params, len);
	free(params, M_BWFM);

	if (error != 0)
		DPRINTF(("%s: join failed: %d\n",
		    device_get_nameunit(sc->sc_dev), error));
}

/*
 * Get station info after association
 */
static void
bwfm_get_station(struct bwfm_softc *sc, struct ieee80211_node *ni)
{
	/* Query RSSI and update node */
	int32_t rssi;

	/* Note: FreeBSD doesn't store RSSI directly in ni_rssi like OpenBSD */
	/* The RSSI is typically handled through ieee80211_input callbacks */
	(void)bwfm_query_cmd(sc, BWFM_C_GET_RSSI, &rssi, sizeof(rssi));
	(void)ni;
}

/*
 * Set WEP/WPA key
 */
static void
bwfm_set_key_task(struct bwfm_softc *sc, void *arg)
{
	struct bwfm_host_cmd_key *cmd = arg;
	struct ieee80211_key *k = &cmd->key;
	struct bwfm_wsec_key key;
	int error;

	BWFM_LOCK_ASSERT(sc);

	memset(&key, 0, sizeof(key));

	key.index = htole32(k->wk_keyix);
	key.len = htole32(k->wk_keylen);
	memcpy(key.data, k->wk_key, k->wk_keylen);

	if (k->wk_flags & IEEE80211_KEY_GROUP) {
		key.flags = htole32(BWFM_WSEC_PRIMARY_KEY);
		memset(key.ea, 0xff, sizeof(key.ea));
	} else {
		memcpy(key.ea, cmd->mac, sizeof(key.ea));
	}

	switch (k->wk_cipher->ic_cipher) {
	case IEEE80211_CIPHER_WEP:
		if (k->wk_keylen == 5)
			key.algo = htole32(BWFM_CRYPTO_ALGO_WEP1);
		else
			key.algo = htole32(BWFM_CRYPTO_ALGO_WEP128);
		break;
	case IEEE80211_CIPHER_TKIP:
		key.algo = htole32(BWFM_CRYPTO_ALGO_TKIP);
		break;
	case IEEE80211_CIPHER_AES_CCM:
		key.algo = htole32(BWFM_CRYPTO_ALGO_AES_CCM);
		break;
	default:
		return;
	}

	error = bwfm_set_iovar(sc, "wsec_key", &key, sizeof(key));
	if (error != 0)
		DPRINTF(("%s: wsec_key failed: %d\n",
		    device_get_nameunit(sc->sc_dev), error));
}

/*
 * Delete a key
 */
static void
bwfm_delete_key_task(struct bwfm_softc *sc, void *arg)
{
	struct bwfm_host_cmd_key *cmd = arg;
	struct bwfm_wsec_key key;

	BWFM_LOCK_ASSERT(sc);

	memset(&key, 0, sizeof(key));
	key.index = htole32(cmd->key.wk_keyix);
	key.flags = htole32(BWFM_WSEC_PRIMARY_KEY);

	bwfm_set_iovar(sc, "wsec_key", &key, sizeof(key));
}

/*
 * Enable firmware events
 */
int
bwfm_enable_events(struct bwfm_softc *sc)
{
	uint8_t eventmask[BWFM_EVENT_MASK_LEN];
	int error;

	/* Get current event mask */
	error = bwfm_get_iovar(sc, "event_msgs", eventmask, sizeof(eventmask));
	if (error != 0)
		return error;

	/* Enable events we care about */
#define	BWFM_SET_EVENT(mask, e)	((mask)[(e) / 8] |= (1 << ((e) % 8)))
	BWFM_SET_EVENT(eventmask, BWFM_E_SET_SSID);
	BWFM_SET_EVENT(eventmask, BWFM_E_LINK);
	BWFM_SET_EVENT(eventmask, BWFM_E_AUTH);
	BWFM_SET_EVENT(eventmask, BWFM_E_DEAUTH);
	BWFM_SET_EVENT(eventmask, BWFM_E_DEAUTH_IND);
	BWFM_SET_EVENT(eventmask, BWFM_E_DISASSOC);
	BWFM_SET_EVENT(eventmask, BWFM_E_DISASSOC_IND);
	BWFM_SET_EVENT(eventmask, BWFM_E_ASSOC);
	BWFM_SET_EVENT(eventmask, BWFM_E_ASSOC_IND);
	BWFM_SET_EVENT(eventmask, BWFM_E_REASSOC);
	BWFM_SET_EVENT(eventmask, BWFM_E_REASSOC_IND);
	BWFM_SET_EVENT(eventmask, BWFM_E_ESCAN_RESULT);
	BWFM_SET_EVENT(eventmask, BWFM_E_PSK_SUP);
#undef BWFM_SET_EVENT

	return bwfm_set_iovar(sc, "event_msgs", eventmask, sizeof(eventmask));
}

/*
 * Set an iovar
 */
static int
bwfm_set_iovar(struct bwfm_softc *sc, const char *name, void *data, size_t len)
{
	char *buf;
	size_t namelen, buflen;
	int error;

	namelen = strlen(name) + 1;
	buflen = namelen + len;
	buf = malloc(buflen, M_BWFM, M_NOWAIT);
	if (buf == NULL)
		return ENOMEM;

	memcpy(buf, name, namelen);
	if (data != NULL && len > 0)
		memcpy(buf + namelen, data, len);

	error = sc->sc_proto_ops->proto_set_dcmd(sc, 0, BWFM_C_SET_VAR,
	    buf, buflen);

	free(buf, M_BWFM);
	return error;
}

/*
 * Get an iovar
 */
static int
bwfm_get_iovar(struct bwfm_softc *sc, const char *name, void *data, size_t len)
{
	char *buf;
	size_t namelen, buflen;
	int error;

	namelen = strlen(name) + 1;
	buflen = max(namelen, len);
	buf = malloc(buflen, M_BWFM, M_NOWAIT | M_ZERO);
	if (buf == NULL)
		return ENOMEM;

	memcpy(buf, name, namelen);

	error = sc->sc_proto_ops->proto_query_dcmd(sc, 0, BWFM_C_GET_VAR,
	    buf, &buflen);

	if (error == 0 && data != NULL && len > 0)
		memcpy(data, buf, len);

	free(buf, M_BWFM);
	return error;
}

/*
 * Send a command
 */
static int
bwfm_send_cmd(struct bwfm_softc *sc, int cmd, void *data, size_t len)
{
	return sc->sc_proto_ops->proto_set_dcmd(sc, 0, cmd, data, len);
}

/*
 * Query a command
 */
static int
bwfm_query_cmd(struct bwfm_softc *sc, int cmd, void *data, size_t len)
{
	size_t dlen = len;
	return sc->sc_proto_ops->proto_query_dcmd(sc, 0, cmd, data, &dlen);
}

/*
 * Process received frame
 */
void
bwfm_rx(struct bwfm_softc *sc, struct mbuf *m, struct mbuf_list *ml)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct ieee80211_node *ni;
	struct ieee80211_frame *wh;

	if (vap == NULL) {
		m_freem(m);
		return;
	}

	/* Get ieee80211 header */
	wh = mtod(m, struct ieee80211_frame *);

	/* Find or create node */
	ni = ieee80211_find_rxnode(ic, (struct ieee80211_frame_min *)wh);
	if (ni == NULL)
		ni = vap->iv_bss;

	/* Input frame to ieee80211 */
	if (ni != NULL) {
		/* For FullMAC, frames come as 802.3, convert to 802.11 */
		if (ieee80211_input(ni, m, 0, 0) != 0)
			ieee80211_free_node(ni);
	} else {
		m_freem(m);
	}
}

/*
 * Process firmware event
 */
void
bwfm_rx_event(struct bwfm_softc *sc, char *buf, size_t len)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct bwfm_event *e;
	uint32_t event_type, status, reason;

	if (len < sizeof(struct bwfm_event))
		return;

	e = (struct bwfm_event *)buf;

	if (memcmp(BWFM_EVENT_OUI, e->hdr.oui, sizeof(e->hdr.oui)) != 0 ||
	    be16toh(e->hdr.usr_subtype) != BWFM_EVENT_MSG_TYPE)
		return;

	event_type = be32toh(e->msg.event_type);
	status = be32toh(e->msg.status);
	reason = be32toh(e->msg.reason);
	(void)reason;	/* Used only in DPRINTF */

	DPRINTF(("%s: event %u status %u reason %u\n",
	    device_get_nameunit(sc->sc_dev), event_type, status, reason));

	switch (event_type) {
	case BWFM_E_ESCAN_RESULT:
		bwfm_escan_result(sc, buf, len);
		break;

	case BWFM_E_SET_SSID:
		if (status == BWFM_E_STATUS_SUCCESS) {
			DPRINTF(("%s: connected\n",
			    device_get_nameunit(sc->sc_dev)));
		}
		break;

	case BWFM_E_LINK:
		if (status == BWFM_E_STATUS_SUCCESS && (e->msg.flags & 1)) {
			/* Link up */
			DPRINTF(("%s: link up\n",
			    device_get_nameunit(sc->sc_dev)));
		} else {
			/* Link down */
			DPRINTF(("%s: link down\n",
			    device_get_nameunit(sc->sc_dev)));
			if (vap != NULL)
				ieee80211_new_state(vap, IEEE80211_S_SCAN, 0);
		}
		break;

	case BWFM_E_DEAUTH:
	case BWFM_E_DEAUTH_IND:
	case BWFM_E_DISASSOC:
	case BWFM_E_DISASSOC_IND:
		DPRINTF(("%s: disassociated\n",
		    device_get_nameunit(sc->sc_dev)));
		if (vap != NULL)
			ieee80211_new_state(vap, IEEE80211_S_SCAN, 0);
		break;

	case BWFM_E_AUTH:
		if (status == BWFM_E_STATUS_SUCCESS) {
			DPRINTF(("%s: authenticated\n",
			    device_get_nameunit(sc->sc_dev)));
		}
		break;

	case BWFM_E_ASSOC:
	case BWFM_E_REASSOC:
		if (status == BWFM_E_STATUS_SUCCESS) {
			DPRINTF(("%s: associated\n",
			    device_get_nameunit(sc->sc_dev)));
			if (vap != NULL)
				ieee80211_new_state(vap, IEEE80211_S_RUN, 0);
		}
		break;

	default:
		break;
	}
}

/*
 * Process escan result
 */
void
bwfm_escan_result(struct bwfm_softc *sc, char *buf, size_t len)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct bwfm_escan_result *res;
	struct bwfm_bss_info *bi;
	struct ieee80211_scanparams sp;
	struct ieee80211_frame wh;
	uint8_t ssid_ie[2 + IEEE80211_NWID_LEN];  /* TLV format: [id][len][data] */
	uint8_t ssid_len;

	if (len < sizeof(struct bwfm_event) + sizeof(struct bwfm_escan_result))
		return;

	res = (struct bwfm_escan_result *)(buf + sizeof(struct bwfm_event));

	/* Check if scan is complete */
	if (le16toh(res->bss_count) == 0) {
		DPRINTF(("%s: scan complete\n",
		    device_get_nameunit(sc->sc_dev)));
		if (vap != NULL)
			ieee80211_scan_done(vap);
		return;
	}

	/* Process each BSS */
	bi = &res->bss_info[0];

	/* Build a fake 802.11 beacon frame header for scan input */
	memset(&wh, 0, sizeof(wh));
	wh.i_fc[0] = IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_BEACON;
	IEEE80211_ADDR_COPY(wh.i_addr2, bi->bssid);
	IEEE80211_ADDR_COPY(wh.i_addr3, bi->bssid);

	/* Build SSID IE in TLV format for FreeBSD */
	ssid_len = bi->ssid_len > IEEE80211_NWID_LEN ? IEEE80211_NWID_LEN : bi->ssid_len;
	ssid_ie[0] = IEEE80211_ELEMID_SSID;
	ssid_ie[1] = ssid_len;
	memcpy(&ssid_ie[2], bi->ssid, ssid_len);

	/* Set up scanparams - FreeBSD expects TLV pointers */
	memset(&sp, 0, sizeof(sp));
	sp.ssid = ssid_ie;	/* Points to TLV-formatted SSID IE */
	sp.bintval = le16toh(bi->beacon_period);
	sp.capinfo = le16toh(bi->capability);
	sp.chan = le16toh(bi->chanspec) & 0xff;
	sp.bchan = sp.chan;

	/* Add to scan cache - FreeBSD needs 7 args including noise */
	if (vap != NULL) {
		ieee80211_add_scan(vap, ic->ic_curchan, &sp, &wh,
		    IEEE80211_FC0_SUBTYPE_BEACON,
		    (int8_t)le16toh(bi->rssi),
		    (int8_t)bi->phy_noise);
	}
}

/*
 * BCDC protocol functions
 */
int
bwfm_proto_bcdc_query_dcmd(struct bwfm_softc *sc, int ifidx, int cmd,
    void *buf, size_t *len)
{
	struct bwfm_proto_bcdc_ctl *ctl = &sc->sc_bcdc_ctl;
	struct bwfm_proto_bcdc_dcmd *dcmd;
	size_t reqlen;
	int error;

	reqlen = sizeof(*dcmd) + *len;
	dcmd = malloc(reqlen, M_BWFM, M_NOWAIT | M_ZERO);
	if (dcmd == NULL)
		return ENOMEM;

	dcmd->cmd = htole32(cmd);
	dcmd->len = htole32(*len);
	dcmd->flags = htole32(BWFM_BCDC_DCMD_GET | (ifidx << 16) |
	    (++sc->sc_bcdc_reqid << 16));

	if (buf != NULL && *len > 0)
		memcpy(dcmd->data, buf, *len);

	/* Send via bus */
	error = sc->sc_bus_ops->bs_txctl(sc, dcmd, reqlen);
	if (error != 0) {
		free(dcmd, M_BWFM);
		return error;
	}

	/* Wait for response */
	ctl->buf = (char *)dcmd;
	ctl->len = reqlen;
	ctl->done = 0;

	/* The bus driver will call bwfm_proto_bcdc_rxctl when response arrives */
	error = sc->sc_bus_ops->bs_rxctl(sc, ctl->buf, &ctl->len);
	if (error == 0 && buf != NULL) {
		*len = min(*len, le32toh(dcmd->len));
		memcpy(buf, dcmd->data, *len);
	}

	free(dcmd, M_BWFM);
	return error;
}

int
bwfm_proto_bcdc_set_dcmd(struct bwfm_softc *sc, int ifidx, int cmd,
    void *buf, size_t len)
{
	struct bwfm_proto_bcdc_dcmd *dcmd;
	size_t reqlen;
	int error;

	reqlen = sizeof(*dcmd) + len;
	dcmd = malloc(reqlen, M_BWFM, M_NOWAIT | M_ZERO);
	if (dcmd == NULL)
		return ENOMEM;

	dcmd->cmd = htole32(cmd);
	dcmd->len = htole32(len);
	dcmd->flags = htole32(BWFM_BCDC_DCMD_SET | (ifidx << 16) |
	    (++sc->sc_bcdc_reqid << 16));

	if (buf != NULL && len > 0)
		memcpy(dcmd->data, buf, len);

	error = sc->sc_bus_ops->bs_txctl(sc, dcmd, reqlen);
	if (error == 0) {
		/* Wait for response to confirm success */
		size_t rlen = reqlen;
		error = sc->sc_bus_ops->bs_rxctl(sc, dcmd, &rlen);
		if (error == 0) {
			uint32_t flags = le32toh(dcmd->flags);
			if (flags & BWFM_BCDC_DCMD_ERROR)
				error = le32toh(dcmd->status);
		}
	}

	free(dcmd, M_BWFM);
	return error;
}

void
bwfm_proto_bcdc_rx(struct bwfm_softc *sc, struct mbuf *m,
    struct mbuf_list *ml)
{
	struct bwfm_proto_bcdc_hdr *hdr;

	if (m->m_len < sizeof(*hdr)) {
		m_freem(m);
		return;
	}

	hdr = mtod(m, struct bwfm_proto_bcdc_hdr *);

	/* Skip BCDC header */
	m_adj(m, sizeof(*hdr) + (hdr->data_offset << 2));

	/* Pass to rx handler */
	bwfm_rx(sc, m, ml);
}

void
bwfm_proto_bcdc_rxctl(struct bwfm_softc *sc, char *buf, size_t len)
{
	/* Control response received - handled in query/set functions */
}

MODULE_VERSION(bwfm, 1);
MODULE_DEPEND(bwfm, wlan, 1, 1, 1);
MODULE_DEPEND(bwfm, firmware, 1, 1, 1);
