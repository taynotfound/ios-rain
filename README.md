# iOS 16 on the Redmi 10C (`rain`)

**Apple's XNU kernel and the iOS 16.7.16 userland of the iPhone X, booting on a Xiaomi Redmi 10C**
(`rain`, Xiaomi ROM family `fog`; Qualcomm **SM6225 "khaje"** / Snapdragon 680, 720×1650 panel, 4 GB RAM).

As of **2026-11-11** the phone boots **on its own** into the **iOS home screen**, with **touch**, volume
and lock keys, and the **Adreno GPU drawing the interface**. It is still slow (about 15 frames a second)
and has no sound or battery reading yet. This is an experiment, not a usable phone OS.

Website: **<https://taynotfound.github.io/ios-rain/>**

> It installs into the `userdata` partition and so **erases Android's user data**. The kernel image
> goes into the `boot_a` slot (or is started from RAM with `fastboot boot`); TWRP stays in `boot_b`.

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
| Boot | Works, **without a PC**: stock Xiaomi ABL boots the shim (boot image v4) from `boot_a`, which starts XNU on Gold core `0x100`. The bootloader does not count down retries for slot a, so it keeps booting iOS |
| CPU | The four Cortex-A73 cores (`cpus=4`) |
| Memory | 2 GiB for the kernel (the `0xC0000000` bank, `build-rain/rain-ios-2g.adt`); `0x90000000`–`0xC0000000` is the GPU's pool (0985) |
| Storage | Works. UFS at `0x4804000`; the iOS root is an HFS+ volume in `userdata` |
| iOS userland | Works. iPhone X 16.7.16 (20H392) root; lock screen, home screen, Settings. A fresh first boot sits on the Apple logo for 15+ minutes |
| Display | Works with `sweetfb_noshadow=1 sweetfb_fit=1`: the 1125×2436 iPhone X canvas is scaled to 720×1559 with bars above and below. The CPU does that scaling: ~66 ms a frame, the main reason it is slow. Next: the display controller's own scaler (`qseedv3lite`) |
| GPU | Works (`gpu=1 gpu_mhz=600`): Adreno 610 through Mesa freedreno (user space built by `.github/workflows/userland.yml`, installed with `build-rain/install-gpu-root.sh`) |
| Touch | Works (patches 0962–0984), also after locking and unlocking. Novatek NT36525B over SPI through GPI DMA; the driver uploads Xiaomi's firmware at boot and again whenever the panel wakes |
| Keys | Volume and lock keys work. Volume up + down for 3 s reboots into fastboot (0989, not yet tested) |
| Live log | Works: `sweetusb=2` makes the phone a USB network device that streams the kernel log (0988, see below) |
| Battery | Not yet: the gauge is not where it is on the Note 12, so iOS shows a fixed 100 %. 0989/0991 look for it on both I2C buses (not yet tested). Charging works on the charger chip's own defaults |
| Vibration | Driver written (0991, GPIO 36), not yet tested |
| Audio | Not yet. Speaker amp (AW87390) driver and firmware lookup are written (0990); the audio DSP runs Qualcomm's older APR protocol, which needs its own driver |
| Wi-Fi | Not yet |
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
| Speaker amp | Awinic AW87390 (I²C SE1 0x58) | SIA8159 |
| Audio DSP firmware | APR / Elite (`qcom,msm-audio-apr`) | AudioReach / GPR |
| Logs | Live over USB from the running kernel (TWRP has no working USB) | log ring pulled from recovery |

Details, the boot log of every test image and the touch bring-up are in
[PORTING-rain.md](PORTING-rain.md).

## Building and booting

The kernel is built by GitHub Actions (`.github/workflows/xnu.yml`, macOS runner), which applies
`xnu/patches/` in order; the GPU user space and audio plug-in by `.github/workflows/userland.yml`.
Locally you need `clang`/`lld`, `mkbootimg`, `fastboot`, `python3` and `gh`.

```bash
build-rain/boot-probe.sh <actions-run-id> <patch-number> <tag> [extra boot args]
```

That downloads the kernel from the run, packs it with the device tree and boot arguments
(`tools/mkpayload.py`, with Xiaomi's touch firmware as `--blob TouchFW=...`), builds the shim boot image
(`shim/build.sh`) and boots it once the phone answers in fastboot. To make it the default:
`fastboot flash boot_a <image> && fastboot set_active a`. To get back to TWRP: `fastboot set_active b`.

The iOS root image is built from Apple's IPSW with `build-rain/build-iosroot.sh`, gets the GPU user space
with `sudo build-rain/install-gpu-root.sh`, and is flashed with `fastboot -S 256M flash userdata`.

### Live kernel log over USB

Boot with `sweetusb=2`. The phone appears as USB device `1209:0001` and sends its log to `192.168.7.1`,
UDP port 5140. On the PC, once: a NetworkManager profile that gives the phone's link (MAC
`02:00:5e:77:ee:01`) the address `192.168.7.1/24`, and a firewall rule for UDP 5140 on that link.
Then:

```bash
build-rain/usblog.py build-rain/logs/live.txt
```

Not in git: Apple's IPSW and root image, Xiaomi's firmware files (touch, GPU, amp), built payloads and boot images.

## Repository layout

| Path | What |
| --- | --- |
| `shim/` | Bare-metal boot shim: boot image v4, hand-off to a Gold core, payload loader |
| `xnu/patches/` | XNU patch series: sweet's 0001–0079, topaz's 0080–0947, **rain's 0960+** |
| `tools/` | `mkadt.py` (Apple Device Tree), `mkpayload.py` (kernel + ADT + boot args), root image tools |
| `site/` | The project website (GitHub Pages, `.github/workflows/pages.yml`) |
| `build-rain/` | Redmi 10C build and boot scripts, device trees, touch-probe working copies |
| `PORTING-rain.md` | Redmi 10C porting log |
| `docs/UPSTREAM-topaz.md`, `NOTES.md` | Upstream documentation (Redmi Note 12 4G, Redmi Note 10 Pro) |
| `docs/UPSTREAM-topaz-RECOVERY.md` | Upstream's recovery runbook. **Redmi Note 12 only, do not run it on a Redmi 10C** |
