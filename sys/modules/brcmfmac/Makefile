DEVBRCMFMACDIR=	${SRCTOP}/sys/contrib/dev/brcm80211/brcmfmac
DEVBRCMINCDIR=	${SRCTOP}/sys/contrib/dev/brcm80211/include

.PATH: ${DEVBRCMFMACDIR}
.PATH: ${DEVBRCMFMACDIR}/wcc
.PATH: ${DEVBRCMFMACDIR}/cyw
.PATH: ${DEVBRCMFMACDIR}/bca

WITH_USB=	1
WITH_PCIE=	1
WITH_SDIO=	1
WITH_DEBUG=	1

KMOD=	if_brcmfmac

# Core sources
SRCS=	cfg80211.c chip.c fwil.c fweh.c p2p.c proto.c
SRCS+=	common.c core.c firmware.c fwvid.c feature.c
SRCS+=	btcoex.c vendor.c pno.c xtlv.c

# BCDC protocol (needed for SDIO/USB)
SRCS+=	bcdc.c fwsignal.c

# MSGBUF protocol (needed for PCIe)
SRCS+=	commonring.c flowring.c msgbuf.c

# Vendor-specific cores
SRCS+=	wcc/core.c cyw/core.c bca/core.c

# USB support
.if defined(WITH_USB) && ${WITH_USB} > 0
SRCS+=	usb.c
CFLAGS+=	-DCONFIG_BRCMFMAC_USB
.endif

# PCIe support
.if defined(WITH_PCIE) && ${WITH_PCIE} > 0
SRCS+=	pcie.c
CFLAGS+=	-DCONFIG_BRCMFMAC_PCIE
.endif

# SDIO support
.if defined(WITH_SDIO) && ${WITH_SDIO} > 0
SRCS+=	sdio.c bcmsdh.c
CFLAGS+=	-DCONFIG_BRCMFMAC_SDIO
.endif

# Debug support
.if defined(WITH_DEBUG) && ${WITH_DEBUG} > 0
SRCS+=	debug.c
CFLAGS+=	-DCONFIG_BRCMDBG
.endif

# ACPI support (FreeBSD has ACPI)
SRCS+=	acpi.c
CFLAGS+=	-DCONFIG_ACPI

# Other required sources
SRCS+=	${LINUXKPI_GENSRCS}
SRCS+=	opt_wlan.h opt_inet6.h opt_inet.h opt_acpi.h

CFLAGS+=	-DKBUILD_MODNAME='"brcmfmac"'
CFLAGS+=	-DCONFIG_BRCMFMAC
CFLAGS+=	-DCONFIG_BRCMFMAC_PROTO_BCDC
CFLAGS+=	-DCONFIG_BRCMFMAC_PROTO_MSGBUF

CFLAGS+=	-I${DEVBRCMFMACDIR}
CFLAGS+=	-I${DEVBRCMINCDIR}
CFLAGS+=	${LINUXKPI_INCLUDES}

# Helpful after fresh imports
#CFLAGS+=	-ferror-limit=0

.include <bsd.kmod.mk>
