/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 The FreeBSD Project
 *
 * Raspberry Pi 5 RP1 I/O Controller - shared definitions
 */

#ifndef _RP1_VAR_H_
#define _RP1_VAR_H_

#include <sys/types.h>
#include <sys/lock.h>
#include <sys/mutex.h>

/*
 * RP1 peripheral base addresses (offsets within BAR0)
 * These are the locations of various IP blocks within the RP1
 */
#define	RP1_IO_BANK0_BASE	0x400d0000	/* GPIO bank 0 */
#define	RP1_IO_BANK1_BASE	0x400d4000	/* GPIO bank 1 */
#define	RP1_IO_BANK2_BASE	0x400d8000	/* GPIO bank 2 */
#define	RP1_SYS_RIO0_BASE	0x400e0000	/* RIO bank 0 */
#define	RP1_SYS_RIO1_BASE	0x400e4000	/* RIO bank 1 */
#define	RP1_SYS_RIO2_BASE	0x400e8000	/* RIO bank 2 */
#define	RP1_PADS_BANK0_BASE	0x400f0000	/* Pad control bank 0 */
#define	RP1_PADS_BANK1_BASE	0x400f4000	/* Pad control bank 1 */
#define	RP1_PADS_BANK2_BASE	0x400f8000	/* Pad control bank 2 */
#define	RP1_PADS_ETH_BASE	0x400fc000	/* Ethernet pad control */

#define	RP1_CLOCKS_BASE		0x40018000	/* Clock controller */
#define	RP1_ETH_BASE		0x00100000	/* GENET Ethernet MAC */
#define	RP1_ETH_SIZE		0x00010000

#define	RP1_USB_BASE		0x00200000	/* USB controller */

/* Interrupt numbers (from rp1.dtsi) */
#define	RP1_INT_IO_BANK0	0
#define	RP1_INT_IO_BANK1	1
#define	RP1_INT_IO_BANK2	2
#define	RP1_INT_AUDIO_IN	3
#define	RP1_INT_AUDIO_OUT	4
#define	RP1_INT_PWM0		5
#define	RP1_INT_ETH		6	/* Ethernet interrupt */
#define	RP1_INT_I2C0		7
#define	RP1_INT_I2C1		8
#define	RP1_INT_I2C2		9
#define	RP1_INT_I2C3		10
#define	RP1_INT_I2C4		11
#define	RP1_INT_I2C5		12
#define	RP1_INT_I2C6		13
#define	RP1_INT_SDIO0		22
#define	RP1_INT_SDIO1		23
#define	RP1_INT_SPI0		24
#define	RP1_INT_SPI1		25
#define	RP1_INT_SPI2		26
#define	RP1_INT_SPI3		27
#define	RP1_INT_SPI4		28
#define	RP1_INT_SPI5		29
#define	RP1_INT_UART0		30
#define	RP1_INT_TIMER_0		32
#define	RP1_INT_TIMER_1		33
#define	RP1_INT_TIMER_2		34
#define	RP1_INT_TIMER_3		35
#define	RP1_INT_USBHOST0	40
#define	RP1_INT_USBHOST0_0	41
#define	RP1_INT_USBHOST0_1	42
#define	RP1_INT_USBHOST0_2	43
#define	RP1_INT_USBHOST0_3	44
#define	RP1_INT_USBHOST1	45
#define	RP1_INT_USBHOST1_0	46
#define	RP1_INT_USBHOST1_1	47
#define	RP1_INT_USBHOST1_2	48
#define	RP1_INT_USBHOST1_3	49
#define	RP1_INT_DMA		50
#define	RP1_INT_SYSCFG		56
#define	RP1_INT_CLOCKS		57
#define	RP1_INT_VBUSCTRL	58
#define	RP1_INT_PROC_MISC	60
#define	RP1_INT_END		64

/* Instance variables for child devices */
enum {
	RP1_IVAR_OFFSET,	/* Peripheral offset within BAR0 */
	RP1_IVAR_SIZE,		/* Peripheral register size */
	RP1_IVAR_IRQ,		/* MSI-X interrupt number */
};

/* Accessor macros for child ivars */
#define	RP1_ACCESSOR(var, ivar, type)					\
	static __inline type						\
	rp1_get_ ## var(device_t dev)					\
	{								\
		uintptr_t v;						\
		BUS_READ_IVAR(device_get_parent(dev), dev,		\
		    RP1_IVAR_ ## ivar, &v);				\
		return ((type)v);					\
	}

RP1_ACCESSOR(offset, OFFSET, bus_size_t)
RP1_ACCESSOR(size, SIZE, bus_size_t)
RP1_ACCESSOR(irq, IRQ, int)

/* RP1 softc structure */
struct rp1_softc {
	device_t		dev;
	struct mtx		mtx;
	struct resource		*res;		/* BAR0 - peripherals */
	struct resource		*msix_res;	/* BAR2 - MSI-X config */
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	bus_space_tag_t		msix_bst;
	bus_space_handle_t	msix_bsh;
	uint32_t		chip_id;
	uint32_t		platform;
	bool			attached;

	/* MSI-X interrupt resources */
	int			msix_count;
	struct resource		*msix_irq[RP1_INT_END];
	void			*msix_ih[RP1_INT_END];
};

/* Child device information */
struct rp1_child_info {
	const char	*name;
	bus_size_t	offset;
	bus_size_t	size;
	int		irq;
};

/* Helper macros for child devices */
#define	RP1_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	RP1_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	RP1_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->mtx, MA_OWNED)

/* Function prototypes for child drivers */
void	rp1_intr_ack(device_t dev, int irqnum);

#endif /* _RP1_VAR_H_ */
