# iOS 16 on the Redmi 10C (`rain`)

**Apple's XNU kernel and the iOS 16.7.16 userland of the iPhone X, booting on a Xiaomi Redmi 10C**
(`rain`, Xiaomi ROM family `fog`; Qualcomm **SM6225 "khaje"** / Snapdragon 680, 720×1650 panel, 4 GB RAM).

As of **2026-11-14** the phone boots into the **iOS home screen** with **touch**, the volume and lock
keys, the **Adreno GPU drawing the interface at 36–49 frames a second**, **sound through the speaker**,
the **battery level**, **vibration**, the **clock**, **110 GB of storage** and a real **Shut Down**.
Wi-Fi and brightness do not work yet. This is an experiment, not a usable phone OS.

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
| Boot | Works, **without a PC**: stock Xiaomi ABL boots the shim (boot image v4) from `boot_a`, which starts XNU on Gold core `0x100`. The bootloader does not count down retries for slot a. `boot_a` still holds an image from 2026-11-12; today's fixes are tested with `fastboot boot` only |
| CPU | The four Cortex-A73 cores (`cpus=4`) |
| Memory | 2 GiB for the kernel (the `0xC0000000` bank, `build-rain/rain-ios-2g.adt`); GPU pool below it (0985), modem pool at `0xBC000000` (0999); 64 MB for device mappings (1008) |
| Storage | Works. UFS at `0x4804000`; the iOS root is flashed as a 16 GB HFS+ image and grows itself to all of `userdata` (~110 GB) on first boot (`hfs-grow`, a one-shot launchd job) |
| iOS userland | Works. iPhone X 16.7.16 (20H392) root; lock screen, home screen, Settings. The first boot after flashing `userdata` sits on the Apple logo for about 15 minutes. About and Date & Time work: the crash-looping `fairplayd.H2` and `bluetoothd` jobs are removed (locationd waited forever on FairPlay). Settings > General > Shut Down powers off (1011); with USB plugged in the phone starts again by itself |
| Display | Works: `sweetfb_fit=3` scales the 1125×2436 canvas to 740×1650 and cuts 10 columns off each side; `sweetfb_hwscale=1` does it in the display controller's own scaler (no CPU copy, 1006) |
| GPU | Works (`gpu=1 gpu_mhz=600`): Adreno 610 through Mesa freedreno (user space built by `.github/workflows/userland.yml`, installed with `build-rain/install-gpu-root.sh`). **36–49 fps** |
| Touch | Works (0962–0984), also after locking and unlocking. Novatek NT36525B over SPI through GPI DMA; Xiaomi's firmware uploaded at boot and on every panel wake |
| Keys | Volume and lock keys work. Volume up + down for 3 s reboots into fastboot (0989) |
| Clock | Works (1006): the PM6125 RTC plus boot arg `rtc_off` (the RTC is read-only to Linux/XNU and resets when the battery runs flat). Set the timezone by hand in Settings > General > Date & Time (automatic needs a location fix) |
| Live log | Works: `sweetusb=2` (from `fastboot boot`) or `sweetusb=3` (also powers USB up itself, for boots from `boot_a`, 1006/1007) makes the phone a USB network device that streams the kernel log |
| Battery | Works: SH366101 gauge at SE1 `0x55` (level, voltage, current, temperature; 0997–1002) |
| Charging | Works: SMB1351 charger, high-current mode on wall chargers (1001); a PC port stays at 500 mA |
| Vibration | Works (GPIO 36, 0991) |
| Audio | **Works** with `adsp=1`: rain's own ADSP firmware (APR/Elite), AFE, SoundWire codec, AW87390 amp (0990, 1003, 1008), iOS through the RedwoodAudio HAL plug-in and Apple's VirtualAudio router. `adsp_tone=N` plays a test chime at boot (1009) |
| Brightness | Driver written (1004, `sweetbl=1`), not yet tested |
| Wi-Fi | Not yet: WCN3950; topaz's modem/WLFW code is in the tree, the plan is in [docs/CONNECTIVITY-rain.md](docs/CONNECTIVITY-rain.md) |
| Cellular, Bluetooth, camera, sensors | Not started (sensors: [docs/SENSORS-rain.md](docs/SENSORS-rain.md)) |

### Next

Wi-Fi; brightness (test 1004); Bluetooth (a real driver, then bluetoothd back); the power-off button
combination; the final kernel in `boot_a` with sound, clock, USB power-up and power-off.

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
The same kernel build is a plain script, `build-rain/ci-build-xnu.sh`, for any Mac: `codemagic.yaml`
runs it on Codemagic as a fallback when GitHub Actions is down (sign in there, add this repository, start the
`xnu` workflow, download `kernel.development.sweet`). XNU needs Apple's Xcode and KDK, so some Mac has to build it.
Locally you need `clang`/`lld`, `mkbootimg`, `fastboot`, `python3` and `gh`.

```bash
build-rain/boot-probe.sh <actions-run-id | kernel file> <patch-number> <tag> [extra boot args]
```

That downloads the kernel from the run, packs it with the device tree and boot arguments
(`tools/mkpayload.py`, with Xiaomi's touch firmware as `--blob TouchFW=...`), builds the shim boot image
(`shim/build.sh`) and boots it once the phone answers in fastboot. To make it the default:
`fastboot flash boot_a <image> && fastboot set_active a`. To get back to TWRP: `fastboot set_active b`.

The iOS root image is built from Apple's IPSW with `build-rain/build-iosroot.sh`, gets the GPU user space
and audio plug-in with `sudo build-rain/install-gpu-root.sh` and the bring-up tweaks with
`sudo build-rain/apply-root-tweaks.sh` (in that order), and is flashed with `fastboot -S 256M flash userdata`.

The boot arguments in use: `sweetfb_fit=3 sweetfb_hwscale=1 gpu=1 gpu_mhz=600 sweetfb_hz=60 adsp=1
sweetusb=3 rtc_off=<seconds>` (on top of the fixed ones in `build-rain/boot-probe.sh`).

### Live kernel log over USB

Boot with `sweetusb=2` (or `sweetusb=3` from `boot_a`). The phone appears as USB device `1209:0001` and sends its log to `192.168.7.1`,
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
