/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 The FreeBSD Project
 * Copyright (c) 2020 Michael J Karels
 * Copyright (c) 2020 Jared McNeill <jmcneill@invisible.ca>
 *
 * Raspberry Pi 5 GENET Ethernet Register Definitions
 *
 * These definitions are derived from the FreeBSD arm64/broadcom/genet
 * driver and adapted for the RP1 context. The register layout is
 * identical - only the access method differs (PCIe BAR vs direct map).
 */

#ifndef _RP1_ETHREG_H_
#define _RP1_ETHREG_H_

/*
 * Broadcom GENETv5 Register Definitions
 */

#define	GENET_SYS_REV_CTRL		0x000
#define	 SYS_REV_MAJOR			0xf000000
#define	 SYS_REV_MINOR			0xf0000
#define	  REV_MAJOR			0xf000000
#define	  REV_MAJOR_SHIFT		24
#define	  REV_MAJOR_V5			6
#define	  REV_MINOR			0xf0000
#define	  REV_MINOR_SHIFT		16
#define	  REV_PHY			0xffff
#define	GENET_SYS_PORT_CTRL		0x004
#define	 GENET_SYS_PORT_MODE_EXT_GPHY	3
#define	GENET_SYS_RBUF_FLUSH_CTRL	0x008
#define	 GENET_SYS_RBUF_FLUSH_RESET	(1 << 1)
#define	GENET_SYS_TBUF_FLUSH_CTRL	0x00c

#define	GENET_EXT_RGMII_OOB_CTRL	0x08c
#define	 GENET_EXT_RGMII_OOB_ID_MODE_DISABLE	(1 << 16)
#define	 GENET_EXT_RGMII_OOB_RGMII_MODE_EN	(1 << 6)
#define	 GENET_EXT_RGMII_OOB_OOB_DISABLE	(1 << 5)
#define	 GENET_EXT_RGMII_OOB_RGMII_LINK		(1 << 4)

/* Interrupt registers */
#define	GENET_INTRL2_CPU_STAT		0x200
#define	GENET_INTRL2_CPU_CLEAR		0x208
#define	GENET_INTRL2_CPU_STAT_MASK	0x20c
#define	GENET_INTRL2_CPU_SET_MASK	0x210
#define	GENET_INTRL2_CPU_CLEAR_MASK	0x214
#define	 GENET_IRQ_MDIO_ERROR		(1 << 24)
#define	 GENET_IRQ_MDIO_DONE		(1 << 23)
#define	 GENET_IRQ_TXDMA_DONE		(1 << 16)
#define	 GENET_IRQ_RXDMA_DONE		(1 << 13)

/* RBUF registers */
#define	GENET_RBUF_CTRL			0x300
#define	 GENET_RBUF_BAD_DIS		(1 << 2)
#define	 GENET_RBUF_ALIGN_2B		(1 << 1)
#define	 GENET_RBUF_64B_EN		(1 << 0)
#define	GENET_RBUF_CHECK_CTRL		0x314
#define	 GENET_RBUF_CHECK_CTRL_EN	(1 << 0)
#define	 GENET_RBUF_CHECK_SKIP_FCS	(1 << 4)
#define	GENET_RBUF_TBUF_SIZE_CTRL	0x3b4

/* TBUF registers */
#define	GENET_TBUF_CTRL			0x600

/* UniMAC registers */
#define	GENET_UMAC_CMD			0x808
#define	 GENET_UMAC_CMD_LCL_LOOP_EN	(1 << 15)
#define	 GENET_UMAC_CMD_SW_RESET	(1 << 13)
#define	 GENET_UMAC_CMD_CRC_FWD		(1 << 6)
#define	 GENET_UMAC_CMD_PROMISC		(1 << 4)
#define	 GENET_UMAC_CMD_SPEED		(3 << 2)
#define	  GENET_UMAC_CMD_SPEED_10	(0 << 2)
#define	  GENET_UMAC_CMD_SPEED_100	(1 << 2)
#define	  GENET_UMAC_CMD_SPEED_1000	(2 << 2)
#define	 GENET_UMAC_CMD_RXEN		(1 << 1)
#define	 GENET_UMAC_CMD_TXEN		(1 << 0)
#define	GENET_UMAC_MAC0			0x80c
#define	GENET_UMAC_MAC1			0x810
#define	GENET_UMAC_MAX_FRAME_LEN	0x814
#define	GENET_UMAC_TX_FLUSH		0xb34
#define	GENET_UMAC_MIB_CTRL		0xd80
#define	 GENET_UMAC_MIB_RESET_TX	(1 << 2)
#define	 GENET_UMAC_MIB_RESET_RUNT	(1 << 1)
#define	 GENET_UMAC_MIB_RESET_RX	(1 << 0)

/* MDIO registers */
#define	GENET_MDIO_CMD			0xe14
#define	 GENET_MDIO_START_BUSY		(1 << 29)
#define	 GENET_MDIO_READ_FAILED		(1 << 28)
#define	 GENET_MDIO_READ		(1 << 27)
#define	 GENET_MDIO_WRITE		(1 << 26)
#define	 GENET_MDIO_PMD			0x3e00000
#define	 GENET_MDIO_REG			0x1f0000
#define	 GENET_MDIO_ADDR_SHIFT		21
#define	 GENET_MDIO_REG_SHIFT		16
#define	 GENET_MDIO_VAL_MASK		0xffff

/* MDF (multicast filter) registers */
#define	GENET_UMAC_MDF_CTRL		0xe50
#define	GENET_UMAC_MDF_ADDR0(n)		(0xe54 + (n) * 0x8)
#define	GENET_UMAC_MDF_ADDR1(n)		(0xe58 + (n) * 0x8)
#define	GENET_MAX_MDF_FILTER		17

/* DMA configuration */
#define	GENET_DMA_DESC_COUNT		256
#define	GENET_DMA_DESC_SIZE		12
#define	GENET_DMA_DEFAULT_QUEUE		16
#define	GENET_DMA_RING_SIZE		0x40
#define	GENET_DMA_RINGS_SIZE		(GENET_DMA_RING_SIZE * (GENET_DMA_DEFAULT_QUEUE + 1))

/* RX DMA base */
#define	GENET_RX_BASE			0x2000
#define	GENET_RX_DMA_RINGBASE(qid)	(GENET_RX_BASE + 0xc00 + GENET_DMA_RING_SIZE * (qid))
#define	GENET_RX_DMA_WRITE_PTR_LO(qid)	(GENET_RX_DMA_RINGBASE(qid) + 0x00)
#define	GENET_RX_DMA_WRITE_PTR_HI(qid)	(GENET_RX_DMA_RINGBASE(qid) + 0x04)
#define	GENET_RX_DMA_PROD_INDEX(qid)	(GENET_RX_DMA_RINGBASE(qid) + 0x08)
#define	GENET_RX_DMA_CONS_INDEX(qid)	(GENET_RX_DMA_RINGBASE(qid) + 0x0c)
#define	GENET_RX_DMA_PROD_CONS_MASK	0xffff
#define	GENET_RX_DMA_RING_BUF_SIZE(qid)	(GENET_RX_DMA_RINGBASE(qid) + 0x10)
#define	 GENET_RX_DMA_RING_BUF_SIZE_DESC_SHIFT	16
#define	 GENET_RX_DMA_RING_BUF_SIZE_BUF_LEN_MASK 0xffff
#define	GENET_RX_DMA_START_ADDR_LO(qid)	(GENET_RX_DMA_RINGBASE(qid) + 0x14)
#define	GENET_RX_DMA_START_ADDR_HI(qid)	(GENET_RX_DMA_RINGBASE(qid) + 0x18)
#define	GENET_RX_DMA_END_ADDR_LO(qid)	(GENET_RX_DMA_RINGBASE(qid) + 0x1c)
#define	GENET_RX_DMA_END_ADDR_HI(qid)	(GENET_RX_DMA_RINGBASE(qid) + 0x20)
#define	GENET_RX_DMA_XON_XOFF_THRES(qid) (GENET_RX_DMA_RINGBASE(qid) + 0x28)
#define	 GENET_RX_DMA_XON_XOFF_THRES_LO_SHIFT	16
#define	GENET_RX_DMA_READ_PTR_LO(qid)	(GENET_RX_DMA_RINGBASE(qid) + 0x2c)
#define	GENET_RX_DMA_READ_PTR_HI(qid)	(GENET_RX_DMA_RINGBASE(qid) + 0x30)

/* TX DMA base */
#define	GENET_TX_BASE			0x4000
#define	GENET_TX_DMA_RINGBASE(qid)	(GENET_TX_BASE + 0xc00 + GENET_DMA_RING_SIZE * (qid))
#define	GENET_TX_DMA_READ_PTR_LO(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x00)
#define	GENET_TX_DMA_READ_PTR_HI(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x04)
#define	GENET_TX_DMA_CONS_INDEX(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x08)
#define	GENET_TX_DMA_PROD_INDEX(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x0c)
#define	GENET_TX_DMA_PROD_CONS_MASK	0xffff
#define	GENET_TX_DMA_RING_BUF_SIZE(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x10)
#define	 GENET_TX_DMA_RING_BUF_SIZE_DESC_SHIFT	16
#define	 GENET_TX_DMA_RING_BUF_SIZE_BUF_LEN_MASK 0xffff
#define	GENET_TX_DMA_START_ADDR_LO(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x14)
#define	GENET_TX_DMA_START_ADDR_HI(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x18)
#define	GENET_TX_DMA_END_ADDR_LO(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x1c)
#define	GENET_TX_DMA_END_ADDR_HI(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x20)
#define	GENET_TX_DMA_MBUF_DONE_THRES(qid) (GENET_TX_DMA_RINGBASE(qid) + 0x24)
#define	GENET_TX_DMA_FLOW_PERIOD(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x28)
#define	GENET_TX_DMA_WRITE_PTR_LO(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x2c)
#define	GENET_TX_DMA_WRITE_PTR_HI(qid)	(GENET_TX_DMA_RINGBASE(qid) + 0x30)

/* RX descriptor */
#define	GENET_RX_DESC_STATUS(idx)	(GENET_RX_BASE + GENET_DMA_DESC_SIZE * (idx) + 0x00)
#define	 GENET_RX_DESC_STATUS_BUFLEN_MASK	0xfff0000
#define	 GENET_RX_DESC_STATUS_BUFLEN_SHIFT	16
#define	 GENET_RX_DESC_STATUS_OWN		(1 << 15)
#define	 GENET_RX_DESC_STATUS_CKSUM_OK		(1 << 15)
#define	 GENET_RX_DESC_STATUS_EOP		(1 << 14)
#define	 GENET_RX_DESC_STATUS_SOP		(1 << 13)
#define	 GENET_RX_DESC_STATUS_RX_ERROR		(1 << 2)
#define	GENET_RX_DESC_ADDRESS_LO(idx)	(GENET_RX_BASE + GENET_DMA_DESC_SIZE * (idx) + 0x04)
#define	GENET_RX_DESC_ADDRESS_HI(idx)	(GENET_RX_BASE + GENET_DMA_DESC_SIZE * (idx) + 0x08)

/* TX descriptor */
#define	GENET_TX_DESC_STATUS(idx)	(GENET_TX_BASE + GENET_DMA_DESC_SIZE * (idx) + 0x00)
#define	 GENET_TX_DESC_STATUS_BUFLEN_SHIFT	16
#define	 GENET_TX_DESC_STATUS_BUFLEN_MASK	0x7ff0000
#define	 GENET_TX_DESC_STATUS_OWN		(1 << 15)
#define	 GENET_TX_DESC_STATUS_EOP		(1 << 14)
#define	 GENET_TX_DESC_STATUS_SOP		(1 << 13)
#define	 GENET_TX_DESC_STATUS_QTAG_MASK		0x1f80
#define	 GENET_TX_DESC_STATUS_CRC		(1 << 6)
#define	 GENET_TX_DESC_STATUS_CKSUM		(1 << 4)
#define	GENET_TX_DESC_ADDRESS_LO(idx)	(GENET_TX_BASE + GENET_DMA_DESC_SIZE * (idx) + 0x04)
#define	GENET_TX_DESC_ADDRESS_HI(idx)	(GENET_TX_BASE + GENET_DMA_DESC_SIZE * (idx) + 0x08)

/* DMA control registers */
#define	GENET_RX_DMA_RING_CFG		(GENET_RX_BASE + 0x1040 + 0x00)
#define	GENET_RX_DMA_CTRL		(GENET_RX_BASE + 0x1040 + 0x04)
#define	 GENET_RX_DMA_CTRL_RBUF_EN(qid)	(1 << ((qid) + 1))
#define	 GENET_RX_DMA_CTRL_EN		(1 << 0)
#define	GENET_RX_SCB_BURST_SIZE		(GENET_RX_BASE + 0x1040 + 0x0c)

#define	GENET_TX_DMA_RING_CFG		(GENET_TX_BASE + 0x1040 + 0x00)
#define	GENET_TX_DMA_CTRL		(GENET_TX_BASE + 0x1040 + 0x04)
#define	 GENET_TX_DMA_CTRL_RBUF_EN(qid)	(1 << ((qid) + 1))
#define	 GENET_TX_DMA_CTRL_EN		(1 << 0)
#define	GENET_TX_SCB_BURST_SIZE		(GENET_TX_BASE + 0x1040 + 0x0c)

/* Status block prepended to TX/RX packets (when checksum offload enabled) */
struct rp1_eth_statusblock {
	uint32_t	status_buflen;
	uint32_t	extstatus;
	uint32_t	rxcsum;
	uint32_t	spare1[9];
	uint32_t	txcsuminfo;
	uint32_t	spare2[3];
};

/* Bits in txcsuminfo */
#define	TXCSUM_LEN_VALID	(1 << 31)
#define	TXCSUM_OFF_SHIFT	16
#define	TXCSUM_UDP		(1 << 15)

#endif /* _RP1_ETHREG_H_ */
