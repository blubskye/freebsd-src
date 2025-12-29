/*	$OpenBSD: bwfmvar.h,v 1.36 2024/04/14 03:23:05 jsg Exp $	*/
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
 * Broadcom FullMAC WiFi driver variable definitions
 */

#ifndef _BWFMVAR_H_
#define _BWFMVAR_H_

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/taskqueue.h>
#include <sys/mutex.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_media.h>
#include <net/ethernet.h>

#include <net80211/ieee80211_var.h>

/* Chip IDs */
#define BWFM_CC_43143_CHIP_ID		43143
#define BWFM_CC_43235_CHIP_ID		43235
#define BWFM_CC_43236_CHIP_ID		43236
#define BWFM_CC_43238_CHIP_ID		43238
#define BWFM_CC_43241_CHIP_ID		0x4324
#define BWFM_CC_43242_CHIP_ID		43242
#define BWFM_CC_4329_CHIP_ID		0x4329
#define BWFM_CC_4330_CHIP_ID		0x4330
#define BWFM_CC_4334_CHIP_ID		0x4334
#define BWFM_CC_43340_CHIP_ID		43340
#define BWFM_CC_43341_CHIP_ID		43341
#define BWFM_CC_4335_CHIP_ID		0x4335
#define BWFM_CC_4339_CHIP_ID		0x4339
#define BWFM_CC_43430_CHIP_ID		43430
#define BWFM_CC_4345_CHIP_ID		0x4345
#define BWFM_CC_43454_CHIP_ID		43454
#define BWFM_CC_43455_CHIP_ID		43455
#define BWFM_CC_4350_CHIP_ID		0x4350
#define BWFM_CC_4354_CHIP_ID		0x4354
#define BWFM_CC_4356_CHIP_ID		0x4356
#define BWFM_CC_43566_CHIP_ID		43566
#define BWFM_CC_43567_CHIP_ID		43567
#define BWFM_CC_43569_CHIP_ID		43569
#define BWFM_CC_43570_CHIP_ID		43570
#define BWFM_CC_4358_CHIP_ID		0x4358
#define BWFM_CC_4359_CHIP_ID		0x4359
#define BWFM_CC_43602_CHIP_ID		43602
#define BWFM_CC_4364_CHIP_ID		0x4364
#define BWFM_CC_4365_CHIP_ID		0x4365
#define BWFM_CC_4366_CHIP_ID		0x4366
#define BWFM_CC_43664_CHIP_ID		43664
#define BWFM_CC_4371_CHIP_ID		0x4371
#define BWFM_CC_4377_CHIP_ID		0x4377
#define BWFM_CC_4378_CHIP_ID		0x4378
#define BWFM_CC_4387_CHIP_ID		0x4387

/* Cypress chips */
#define CY_CC_4373_CHIP_ID		0x4373
#define CY_CC_43012_CHIP_ID		43012
#define CY_CC_43439_CHIP_ID		43439
#define CY_CC_43752_CHIP_ID		43752

/* Defaults */
#define BWFM_DEFAULT_SCAN_CHANNEL_TIME	40
#define BWFM_DEFAULT_SCAN_UNASSOC_TIME	40
#define BWFM_DEFAULT_SCAN_PASSIVE_TIME	120

/* Firmware file types */
#define BWFM_FW_CODE			0
#define BWFM_FW_NVRAM			1
#define BWFM_FW_CLM			2

/* Forward declarations */
struct bwfm_softc;

/* mbuf_list compatibility for FreeBSD (must be before bwfm_proto_ops) */
struct mbuf_list {
	struct mbuf	*ml_head;
	struct mbuf	*ml_tail;
	int		ml_len;
};

static inline void
ml_init(struct mbuf_list *ml)
{
	ml->ml_head = ml->ml_tail = NULL;
	ml->ml_len = 0;
}

static inline void
ml_enqueue(struct mbuf_list *ml, struct mbuf *m)
{
	if (ml->ml_tail == NULL)
		ml->ml_head = m;
	else
		ml->ml_tail->m_nextpkt = m;
	ml->ml_tail = m;
	m->m_nextpkt = NULL;
	ml->ml_len++;
}

static inline struct mbuf *
ml_dequeue(struct mbuf_list *ml)
{
	struct mbuf *m;

	m = ml->ml_head;
	if (m != NULL) {
		ml->ml_head = m->m_nextpkt;
		if (ml->ml_head == NULL)
			ml->ml_tail = NULL;
		m->m_nextpkt = NULL;
		ml->ml_len--;
	}
	return m;
}

/* Firmware selector structure */
struct bwfm_firmware_selector {
	uint32_t	fwsel_chip;
	uint32_t	fwsel_revmask;
	const char	*fwsel_basename;
};

/* Firmware loading context */
struct bwfm_firmware_context {
	struct bwfm_softc	*ctx_sc;
	char			*ctx_model;
	uint32_t		ctx_chip;
	uint32_t		ctx_chiprev;
	uint8_t			ctx_data[3];
	size_t			ctx_size[3];
	int			ctx_error;
};

/* Core information */
struct bwfm_core {
	uint16_t	co_id;
	uint16_t	co_rev;
	uint32_t	co_base;
	uint32_t	co_wrapbase;
	LIST_ENTRY(bwfm_core) co_link;
};

/* Chip information */
struct bwfm_chip {
	uint32_t	ch_chip;
	uint32_t	ch_chiprev;
	uint32_t	ch_cc_caps;
	uint32_t	ch_cc_caps_ext;
	uint32_t	ch_pmucaps;
	uint32_t	ch_pmurev;
	uint32_t	ch_rambase;
	uint32_t	ch_ramsize;
	uint32_t	ch_srsize;
	char		ch_name[16];
	LIST_HEAD(, bwfm_core) ch_list;
	int		ch_core_initialized;
	struct bwfm_core *ch_active;
};

/* Bus core operations */
struct bwfm_buscore_ops {
	uint32_t (*bc_read)(struct bwfm_softc *, uint32_t);
	void	 (*bc_write)(struct bwfm_softc *, uint32_t, uint32_t);
	int	 (*bc_prepare)(struct bwfm_softc *);
	int	 (*bc_reset)(struct bwfm_softc *);
	void	 (*bc_setup)(struct bwfm_softc *);
	void	 (*bc_activate)(struct bwfm_softc *, uint32_t);
};

/* Bus operations */
struct bwfm_bus_ops {
	int	(*bs_preinit)(struct bwfm_softc *);
	void	(*bs_stop)(struct bwfm_softc *);
	int	(*bs_txcheck)(struct bwfm_softc *);
	int	(*bs_txdata)(struct bwfm_softc *, struct mbuf *);
	int	(*bs_txctl)(struct bwfm_softc *, void *, size_t);
	int	(*bs_rxctl)(struct bwfm_softc *, void *, size_t *);
};

/* Protocol operations */
struct bwfm_proto_ops {
	int	(*proto_query_dcmd)(struct bwfm_softc *, int, int,
		    void *, size_t *);
	int	(*proto_set_dcmd)(struct bwfm_softc *, int, int,
		    void *, size_t);
	void	(*proto_rx)(struct bwfm_softc *, struct mbuf *,
		    struct mbuf_list *);
	void	(*proto_rxctl)(struct bwfm_softc *, char *, size_t);
};

/* Host command types */
enum bwfm_cmd_type {
	BWFM_HOST_CMD_NEWSTATE,
	BWFM_HOST_CMD_KEY_SET,
	BWFM_HOST_CMD_KEY_DEL,
};

/* Host command for state changes */
struct bwfm_host_cmd_newstate {
	enum ieee80211_state	state;
	int			arg;
};

/* Host command for key operations */
struct bwfm_host_cmd_key {
	struct ieee80211_node	*ni;
	struct ieee80211_key	key;
	uint8_t			mac[IEEE80211_ADDR_LEN];
};

/* Generic host command structure */
struct bwfm_host_cmd {
	enum bwfm_cmd_type	type;
	union {
		struct bwfm_host_cmd_newstate newstate;
		struct bwfm_host_cmd_key key;
	} cmd;
};

/* Host command ring */
#define BWFM_HOST_CMD_RING_COUNT	32
struct bwfm_host_cmd_ring {
	struct bwfm_host_cmd	cmd[BWFM_HOST_CMD_RING_COUNT];
	int			cur;
	int			next;
	int			queued;
};

/* BCDC protocol control */
struct bwfm_proto_bcdc_ctl {
	int		reqid;
	char		*buf;
	size_t		len;
	int		done;
};

/* Main softc structure */
struct bwfm_softc {
	device_t		 sc_dev;
	struct ieee80211com	 sc_ic;
	struct ieee80211vap	*sc_vap;	/* Single VAP for now */
	struct mtx		 sc_mtx;

	/* Bus operations */
	struct bwfm_bus_ops	*sc_bus_ops;
	const struct bwfm_proto_ops *sc_proto_ops;

	/* Chip/firmware info */
	struct bwfm_chip	 sc_chip;
	uint8_t			 sc_io_type;
#define BWFM_IO_TYPE_D11N	1
#define BWFM_IO_TYPE_D11AC	2

	/* Task queue */
	struct taskqueue	*sc_taskq;
	struct task		 sc_task;

	/* Command queue */
	struct bwfm_host_cmd_ring sc_cmdq;

	/* Firmware */
	char			*sc_fwdir;
	uint8_t			*sc_clm;
	size_t			 sc_clmsize;

	/* Node management */
	int			(*sc_newstate)(struct ieee80211vap *,
				    enum ieee80211_state, int);

	/* Interface state */
	int			 sc_if_flags;
	uint8_t			 sc_ccode[4];	/* Country code */

	/* Scan state */
	int			 sc_scan_ver;

	/* Protocol state */
	struct bwfm_proto_bcdc_ctl sc_bcdc_ctl;
	int			 sc_bcdc_reqid;

	/* MAC address */
	uint8_t			 sc_lladdr[ETHER_ADDR_LEN];

	/* Bus-specific data */
	void			*sc_bus_data;
};

#define BWFM_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define BWFM_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define BWFM_LOCK_ASSERT(sc)	mtx_assert(&(sc)->sc_mtx, MA_OWNED)

/* Function prototypes */
void	bwfm_attach(struct bwfm_softc *);
void	bwfm_detach(struct bwfm_softc *);

int	bwfm_chip_attach(struct bwfm_softc *,
	    struct bwfm_buscore_ops *, char *);
void	bwfm_chip_detach(struct bwfm_softc *);
struct bwfm_core *bwfm_chip_get_core(struct bwfm_softc *, int);
struct bwfm_core *bwfm_chip_get_pmu(struct bwfm_softc *);

int	bwfm_proto_bcdc_query_dcmd(struct bwfm_softc *, int, int,
	    void *, size_t *);
int	bwfm_proto_bcdc_set_dcmd(struct bwfm_softc *, int, int,
	    void *, size_t);
void	bwfm_proto_bcdc_rx(struct bwfm_softc *, struct mbuf *,
	    struct mbuf_list *);
void	bwfm_proto_bcdc_rxctl(struct bwfm_softc *, char *, size_t);

void	bwfm_rx(struct bwfm_softc *, struct mbuf *, struct mbuf_list *);
void	bwfm_rx_event(struct bwfm_softc *, char *, size_t);
void	bwfm_escan_result(struct bwfm_softc *, char *, size_t);

void	bwfm_do_async(struct bwfm_softc *,
	    void (*)(struct bwfm_softc *, void *), void *, size_t);

int	bwfm_enable_events(struct bwfm_softc *);
void	bwfm_init_channels(struct ieee80211com *);

extern const struct bwfm_proto_ops bwfm_proto_bcdc_ops;

/* Driver interface state flags (used internally) */
#define BWFM_RUNNING		0x0001

#endif /* _BWFMVAR_H_ */
