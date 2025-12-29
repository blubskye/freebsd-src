# $FreeBSD$

.PATH:	${SRCTOP}/sys/dev/rp1

KMOD=	rp1
SRCS=	rp1.c rp1_eth.c rp1_sdhci.c
SRCS+=	device_if.h bus_if.h pci_if.h miibus_if.h
SRCS+=	mmcbr_if.h sdhci_if.h

# Include option headers
SRCS+=	opt_mmccam.h opt_platform.h

.include <bsd.kmod.mk>
