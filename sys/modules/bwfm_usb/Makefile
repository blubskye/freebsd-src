# $FreeBSD$
#
# Broadcom FullMAC USB WiFi driver

.PATH:	${SRCTOP}/sys/dev/bwfm

KMOD=	bwfm_usb
SRCS=	if_bwfm_usb.c

# Device interface headers
SRCS+=	device_if.h bus_if.h usb_if.h

# Option headers
SRCS+=	opt_wlan.h opt_usb.h

.include <bsd.kmod.mk>
