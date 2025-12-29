# $FreeBSD$

.PATH:	${SRCTOP}/sys/dev/bwfm

KMOD=	bwfm
SRCS=	bwfm.c if_bwfm_sdio.c
SRCS+=	device_if.h bus_if.h sdio_if.h

# Include option headers
SRCS+=	opt_wlan.h

.include <bsd.kmod.mk>
