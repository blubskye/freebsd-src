# $FreeBSD$
#
# Broadcom FullMAC PCI WiFi driver
#
# NOTE: This driver is a work-in-progress skeleton. The msgbuf protocol
# implementation is not yet complete. See if_bwfm_pci.c for details.

.PATH:	${SRCTOP}/sys/dev/bwfm

KMOD=	bwfm_pci
SRCS=	if_bwfm_pci.c

# Device interface headers
SRCS+=	device_if.h bus_if.h pci_if.h

# Option headers
SRCS+=	opt_wlan.h

.include <bsd.kmod.mk>
