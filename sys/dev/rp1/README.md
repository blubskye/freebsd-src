# Raspberry Pi 5 Network Drivers for FreeBSD

This branch contains experimental drivers for Raspberry Pi 5 network support on FreeBSD.

## Overview

The Raspberry Pi 5 uses the RP1 I/O controller chip connected via PCIe, which provides access to:
- Gigabit Ethernet (Broadcom GENET MAC with BCM54213PE PHY)
- WiFi/Bluetooth via SDIO (Broadcom BCM43455/BCM43456)

These drivers enable network connectivity on the RPi5 by implementing:
1. **rp1** - RP1 PCIe I/O controller driver with MSI-X interrupt support
2. **rp1_eth** - GENET Gigabit Ethernet driver adapted for RP1
3. **rp1_sdhci** - SDHCI controller driver for SD card and SDIO WiFi
4. **bwfm** - Broadcom FullMAC WiFi driver (ported from OpenBSD)

## Driver Components

### RP1 I/O Controller (`sys/dev/rp1/`)

| File | Description |
|------|-------------|
| `rp1.c` | Main RP1 PCIe driver, MSI-X setup, child enumeration |
| `rp1_var.h` | Shared definitions and softc structures |
| `rp1_eth.c` | GENET Ethernet MAC driver |
| `rp1_ethreg.h` | GENET register definitions |
| `rp1_sdhci.c` | SDHCI controller for SD/SDIO |

### Broadcom FullMAC WiFi (`sys/dev/bwfm/`)

| File | Description |
|------|-------------|
| `bwfm.c` | Core WiFi driver, net80211 integration |
| `bwfmvar.h` | Driver structures and definitions |
| `bwfmreg.h` | Firmware interface definitions |
| `if_bwfm_sdio.c` | SDIO bus attachment |

## Building

### Build the kernel modules

```sh
cd /usr/src/sys/modules/rp1
make clean && make

cd /usr/src/sys/modules/bwfm
make clean && make
```

### Install modules

```sh
cp /usr/obj/.../rp1.ko /boot/modules/
cp /usr/obj/.../bwfm.ko /boot/modules/
```

### Load at boot

Add to `/boot/loader.conf`:
```
rp1_load="YES"
bwfm_load="YES"
```

## Firmware Requirements

The bwfm WiFi driver requires Broadcom firmware files. On Raspberry Pi 5:

```
/boot/firmware/brcmfmac43455-sdio.bin
/boot/firmware/brcmfmac43455-sdio.txt
/boot/firmware/brcmfmac43455-sdio.clm_blob
```

These are typically available from the Raspberry Pi firmware repository or Linux firmware packages.

## Hardware Support

### Ethernet
- Broadcom GENET v5 MAC
- BCM54213PE Gigabit PHY (RGMII)
- 10/100/1000 Mbps
- Hardware checksum offload

### WiFi
- Broadcom BCM43455 or BCM43456
- 802.11a/b/g/n/ac
- 2.4GHz and 5GHz bands
- WPA/WPA2/WPA3 support (via wpa_supplicant)

## Status

**Experimental** - These drivers are under active development.

### Working
- [x] RP1 PCIe enumeration
- [x] MSI-X interrupt routing
- [x] Kernel module compilation (amd64)

### In Progress
- [ ] GENET Ethernet functionality
- [ ] SDHCI SD card support
- [ ] bwfm WiFi association

### TODO
- [ ] Testing on actual RPi5 hardware
- [ ] ARM64 cross-compilation
- [ ] DMA buffer handling optimization
- [ ] Power management

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    FreeBSD Kernel                        │
├──────────────┬──────────────┬───────────────────────────┤
│   net80211   │    miibus    │         mmc/sdhci         │
├──────────────┴──────────────┴───────────────────────────┤
│                      bwfm.ko                             │
│              (Broadcom FullMAC WiFi)                     │
├─────────────────────────────────────────────────────────┤
│                       rp1.ko                             │
│  ┌─────────────┬─────────────┬─────────────────────┐    │
│  │  rp1_eth    │ rp1_sdhci   │    rp1 (parent)     │    │
│  │  (GENET)    │  (SDIO)     │   PCIe + MSI-X      │    │
│  └─────────────┴─────────────┴─────────────────────┘    │
├─────────────────────────────────────────────────────────┤
│                     PCIe Bus                             │
├─────────────────────────────────────────────────────────┤
│                  RP1 I/O Controller                      │
│            (Vendor: 0x1de4 Device: 0x0001)               │
└─────────────────────────────────────────────────────────┘
```

## References

- [RP1 Peripherals Datasheet](https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf)
- [OpenBSD bwfm(4)](https://man.openbsd.org/bwfm)
- [Linux GENET driver](https://github.com/torvalds/linux/tree/master/drivers/net/ethernet/broadcom/genet)
- [Linux sdhci-of-dwcmshc](https://github.com/torvalds/linux/blob/master/drivers/mmc/host/sdhci-of-dwcmshc.c)

## License

BSD-2-Clause, consistent with FreeBSD kernel code.

## Contributors

Ported and adapted for FreeBSD with assistance from Claude Code.
