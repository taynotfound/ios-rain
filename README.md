# iOS 16 on the Redmi 10C (`rain`)

**Apple's XNU kernel and the iOS 16.7.16 userland of the iPhone X, booting on a Xiaomi Redmi 10C**
(`rain`, Xiaomi ROM family `fog`; Qualcomm **SM6225 "khaje"** / Snapdragon 680, 720×1650 panel, 4 GB RAM).

As of **2026-11-11** the phone boots to the **iOS lock screen and home screen with working touch**
and volume/lock keys. Drawing is done by the CPU on one core, so it is slow. This is an experiment,
not a usable phone OS.

> It installs into the `userdata` partition and so **erases Android's user data**. Nothing is flashed
> to the boot partitions: the kernel is started from RAM with `fastboot boot`.

## Where this comes from

This repo is a fork, and almost all of the hard work was done upstream:

- **[MakrSas/ios-topaz](https://github.com/MakrSas/ios-topaz)** is the direct parent: the same
  thing on the **Redmi Note 12 4G** (`topaz`), which uses the same SM6225 chip. The boot shim, the
  ARMv8.0 kernel work, UFS, display scanout, modem/Wi-Fi, GPU and audio drivers are theirs. Forked
  at `e477291`. Its README is kept as [docs/UPSTREAM-topaz.md](docs/UPSTREAM-topaz.md).
- **[dipa-studio/ios-sweet](https://github.com/dipa-studio/ios-sweet)** is what ios-topaz ports:
  XNU with the iPhone X userland on a Redmi Note 10 Pro. Its engineering log is [NOTES.md](NOTES.md).
- **[HackiOS](https://github.com/oosexgang/hackios-xnu-kernel)** (OOSEX GANG) is where much of the
  GPU, display, audio and Wi-Fi glue started.

Licences and what comes from where: [CREDITS.md](CREDITS.md). iOS itself is Apple's and is not
distributed here.

What this fork adds is the Redmi 10C part: patches `0960` and up in `xnu/patches/`, the `rain`
values in the shim and `tools/mkadt.py`, `build-rain/`, and the porting log
[PORTING-rain.md](PORTING-rain.md).

## State on the Redmi 10C

| Area | State |
| --- | --- |
| Boot | Works. Stock Xiaomi ABL → `fastboot boot` of the shim (boot image v4) → XNU on Gold core `0x100` |
| CPU | One Cortex-A73 core (`cpus=1`). More cores are untested on this phone |
| Memory | 3 GiB (`0x80000000` + 3 GiB, `mkadt.py --midmem`) |
| Storage | Works. UFS at `0x4804000`; the iOS root is an HFS+ volume in `userdata` |
| iOS userland | Works. iPhone X 16.7.16 (20H392) root; reaches the lock screen. The first boot sat on the Apple logo for over 10 minutes |
| Display | Works with `sweetfb_noshadow=1 sweetfb_fit=1`: the 1125×2436 iPhone X canvas is scaled to 720×1559 with bars above and below. With the shadow buffers on, the kernel hangs (open) |
| Touch | Works (patches 0962–0983). Novatek NT36525B over SPI through GPI DMA; the driver uploads Xiaomi's firmware at boot and again whenever the chip loses it (it is also the display driver, and a panel reset wipes it) |
| Keys | Volume and lock keys work |
| Audio, Wi-Fi | Untested on this phone. Audio needs a driver for the Awinic AW87xxx speaker amplifier |
| Cellular, Bluetooth, camera, sensors | Not planned / not started |

Security is off, as upstream: permissive AMFI and sandbox, no data protection.

## How the Redmi 10C differs from the Redmi Note 12

Same SoC, GIC, UFS and boot path. Different:

| | Redmi 10C (`rain`) | Redmi Note 12 4G (`topaz`) |
| --- | --- | --- |
| Panel | 720×1650, BOE with NT36525B | 1080×2400 |
| Touch | Novatek NT36525B, **SPI** on QUP0 SE0, firmware uploaded every boot | FocalTech FT5452, I²C on QUP0 SE2 |
| SE0 data path | GPI DMA only (FIFO interface disabled), needs SMMU streams `0xf6` and `0xe3` mapped | FIFO |
| DDR banks | `0x40000000`+989 MiB, `0x80000000`+1 GiB, `0xC0000000`+2 GiB | upper bank `0x1_40000000`+4 GiB |
| Speaker amp | Awinic AW87xxx | SIA8159 |
| Recovery logs | TWRP has no working USB, so only the on-screen console | log ring pulled from recovery |

Details, the boot log of every test image and the touch bring-up are in
[PORTING-rain.md](PORTING-rain.md).

## Building and booting

The kernel is built by GitHub Actions (`.github/workflows/xnu.yml`, macOS runner), which applies
`xnu/patches/` in order. Locally you need `clang`/`lld`, `mkbootimg`, `fastboot`, `python3` and `gh`.

```bash
build-rain/boot-probe.sh <actions-run-id> <patch-number> <tag>
```

That downloads the kernel from the run, packs it with the device tree (`build-rain/rain-ios.adt`)
and boot arguments (`tools/mkpayload.py`), builds the shim boot image (`shim/build.sh`) and boots
it once the phone answers in fastboot. The iOS root image is built with `build-rain/build-iosroot.sh`
from Apple's IPSW and flashed to `userdata`.

Not in git: Apple's IPSW and root image, Xiaomi's touch firmware (`novatek_ts_fw.bin`, passed in
with `mkpayload.py --blob TouchFW=...`), built payloads and boot images.

## Repository layout

| Path | What |
| --- | --- |
| `shim/` | Bare-metal boot shim: boot image v4, hand-off to a Gold core, payload loader |
| `xnu/patches/` | XNU patch series: sweet's 0001–0079, topaz's 0080–0947, **rain's 0960+** |
| `tools/` | `mkadt.py` (Apple Device Tree), `mkpayload.py` (kernel + ADT + boot args), root image tools |
| `build-rain/` | Redmi 10C build and boot scripts, device trees, touch-probe working copies |
| `PORTING-rain.md` | Redmi 10C porting log |
| `docs/UPSTREAM-topaz.md`, `NOTES.md` | Upstream documentation (Redmi Note 12 4G, Redmi Note 10 Pro) |
| `docs/UPSTREAM-topaz-RECOVERY.md` | Upstream's recovery runbook. **Redmi Note 12 only, do not run it on a Redmi 10C** |
