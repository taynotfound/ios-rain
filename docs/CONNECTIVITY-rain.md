# Wi-Fi and Bluetooth on rain: research and plan

Written 2026-11-12. Nothing here has been run on the phone yet. Each fact is marked:

- **[repo]** read in this repo's patches or docs
- **[linux]** read in mainline Linux sources (`sm6115.dtsi`, `qrb4210-rb2.dts`, `hci_qca.c`, `btqca.c`, `ath10k/snoc.c`)
- **[woa]** from MakrSas/topaz-woa `docs/RESEARCH_wifi.md` (measured on topaz, the Note 12 4G)
- **[scion]** from the descriptions of Scion-Project/scion-kernel PRs 166 and 171 (iOS 16.5 Bluetooth on a Galaxy S9); read second-hand through a page summary, the diffs were not read
- **[guess]** my inference, to be checked

The rain device tree (`dtb/rain.dts`) and the fog vendor image are not in git and were not
available in this session, so every rain value below is "same SoC as topaz, expected equal"
until the checks in section 1 are done.

## Summary

| | Wi-Fi | Bluetooth |
| --- | --- | --- |
| Chip | WCN3950 (one chip does both). topaz's WLFW reports chip 0x4130 / family 0x4007 [woa], which is Linux's `QCA_WCN3950_SOC_ID_T` 0x40074130 [linux] | same chip, separate interface |
| Where it hangs | Inside the SoC's WLAN block at `0xC800000`, firmware runs on the modem DSP | HCI over UART, QUP0 SE3 `0x4A8C000` |
| Driver in this repo | Complete for topaz: `TopazModem` (0113-0130) + `en0`/Apple80211 (0900) | none |
| Work for rain | Boot it and fix what differs: probably hours to days | New kernel driver + iOS user-space setup: the larger job |
| First test | boot args `modem=1 wifi=1` | staged `bt=N` bring-up, section 3 |

Wi-Fi comes first: it is nearly free, it gives the phone a network (ssh, log upload without
the USB cable), and it powers the same chip rails Bluetooth needs.

## 1. Checks to run on the PC first (no phone needed)

In `ios-topaz/dtb/rain.dts`:

```bash
grep -n -A40 'icnss\|wcn3990\|bt_wcn\|btfmslim\|wlan_msa' dtb/rain.dts | less
```

Wanted, with the topaz/mainline values to compare:

| Item | Expected | Source |
| --- | --- | --- |
| WLAN block | `qcom,icnss@c800000`, 0x800000 long | [woa][linux] |
| WLAN IRQs | 12 copy-engine SPIs 358-369 (0x166-0x171) | [woa][linux] |
| WLAN SMMU stream | `apps_smmu 0x1a0 0x1` | [woa][linux] |
| MSA memory | `wlan_msa` at `0x51900000`, 1 MiB. **Check on rain**: its PIL regions are shifted against topaz (GPU zap is 0x56615000 on rain, 0x55B15000 on topaz, boot log 54) | [woa][repo] |
| WLAN rails | vdd-cx-mx (L8A), 1.8-xo (L16A 1.3 V), 1.3-rfa (L17A), 3.3-ch0 (L23A); PM6125 through RPM | [linux] rb2, [woa] |
| BT node | `qcom,wcn3990`, `qca,bt-sw-ctrl-gpio` = TLMM 87; supplies vdd-io L9A 1.8 V, xtal L16A, core L17A, pa L23A | [woa][linux] rb2 |
| BT UART | QUP0 SE3 `0x4a8c000`, SPI 330; TLMM 8 CTS, 9 RTS, 10 TX, 11 RX, function `qup3`; wake IRQ on GPIO 11 | [linux] |
| BT audio | `btfmslim` on a SLIMbus controller (SCO/A2DP offload). Not needed for data | [guess] |

In the fog vendor image (the folder `research/fw-fog/` already has touch firmware from it):

```bash
ls vendor/firmware | grep -i -E 'cmbtfw|cmnv|crbtfw|crnv|apbtfw|apnv|wlanmdsp|bdwlan'
```

- Bluetooth needs `cmbtfwXX.tlv` (patch) and `cmnvXX.bin` (NVM), XX = ROM version, 12 or 13.
  For SoC id 0x40074130 the NVM may carry a suffix: `cmnv13t.bin`; 0x40075130 → `cmnv13s.bin` [linux].
  If only `crbtfw*`/`apbtfw*` exist, the chip is a WCN3990/WCN3988 and the names change, nothing else.
- Wi-Fi board data `bdwlan.*` and `wlanmdsp.mbn` live in the **modem partition** (FAT), not in
  vendor. The 0990 FAT reader finds them by name [repo].

## 2. Wi-Fi

### What exists [repo]

`TopazModem.cpp` does the whole chain in the kernel: modem PAS boot through TrustZone
(PAS id 4) from the stock `modem_a` partition, SMEM/SMP2P, GLINK, QRTR, the host services the
modem asks for (pd-mapper, tftp, rmtfs with a read-only EFS), WLFW QMI up to mission mode,
copy engines at `0xC800000 + 0x240000`, HTC/WMI/HTT, scan, association, WPA2-PSK. Patch 0900
puts `en0` and an Apple80211 ioctl surface on top so that iOS's `wifid` drives it. `mkadt.py`
already writes the `wlan` ADT node that makes launchd start `wifid`.

Rain already has two of the three rain-specific pieces:

- 0990: files are looked up by name in the modem partition's FAT (topaz used a fixed offset
  table of the Note 12 firmware, which would be wrong here).
- 0999: the modem pool moved to `0xBC000000`, out of kernel RAM.
- `adsp=1` works on rain with this code (boot log 69), so SCM/PAS, SMEM, GLINK and QRTR are
  proven on this phone. The modem uses the same path with another PAS id.

### What can differ on rain

1. **Modem memory regions.** PAS `mem_setup` takes address and size from the firmware and the
   reserved region. ADSP needed a rain carveout size (0998). Expect the same for the modem:
   compare `modem_mem` / `pil_modem_mem` in rain.dts with the constants in `TopazModem.cpp`.
2. **MSA region address** (table above).
3. **WLAN rails.** `TopazModem` makes no RPM votes for the four WCN rails, and topaz-woa
   also reached FW_READY without them [woa][repo]. Either ABL leaves them on or the modem
   votes for them. If WLFW stays at "no FW_READY" or scans return nothing, vote L8A/L16A/L17A/L23A
   from the kernel. The audio codec patch (0145) already sends an RPM regulator request
   (`ldoa` 14, `uv`, `swen`), which is the model.
4. **Board data.** WLFW reports a board id; the driver asks for `bdwlan.bNN` / `bdwlan.NNN`
   and falls back to `bdwlan.bin` [repo]. topaz reported 0xFF [woa]. With the wrong board file
   the radio works badly (weak signal, missing channels) rather than not at all.
5. **EFS.** rmtfs serves `modemst1/2`, `fsg`, `fsc` read-only [repo]. These partitions hold
   the IMEI and RF calibration and were never backed up on this phone (PORTING-rain.md, "Not
   done"). The driver never writes them; keep it that way.
6. **A/B slot.** iOS boots from slot a now. The driver reads `modem_a` by default; boot arg
   `modem_part=` overrides [repo]. Check both slots hold a modem image.
7. **MAC address.** Find out what `en0` reports; without a provisioned address the chip may
   use a default. Stock Android keeps it in `persist` (`wlan_mac.bin`) [guess].

### Test order

1. Kernel 1002 or later, USB live log on (`sweetusb=2`), add `modem=1 modem_watch=1`. Expect:
   PAS supported/init/mem_setup/auth OK, modem READY over SMP2P, QRTR services (about 40).
   No `wifi=1` yet.
2. Add `wifi=1`. Expect WLFW: IND_REGISTER, HOST_CAP, MSA_INFO, MSA_READY, CAP (chip/board
   id line: write it down here), BDF download, FW_READY.
3. Scan results in the log, then iOS Settings → Wi-Fi lists networks, join a WPA2 network.
4. Known limits inherited from topaz: WPA2-PSK only, no power save, legacy rates [repo][woa].

`sweetbusy=N` exists because `wifid` once hung the boot waiting for IOKit to quiesce
(0127/0128) [repo]; keep it in mind if boot stalls with `wifi=1`.

## 3. Bluetooth

Nobody upstream has done this (ios-sweet, ios-topaz, HackiOS on the POCO: all "no Bluetooth").
The one precedent is Scion's Galaxy S9 port, which got to "Settings → Bluetooth on, scan
works" on 2026-11-03, with a Broadcom chip [scion]. Its iOS-side findings carry over; its
chip driver does not.

Two halves: (A) get the WCN3950 to speak plain HCI over the UART, entirely in the kernel;
(B) hand that byte stream to iOS's `BlueTool` and `bluetoothd`.

### A. Chip bring-up in the kernel (new file, e.g. `iokit/Kernel/arm/RainBT.cpp`)

Hardware steps, following Linux `hci_qca.c` for `QCA_WCN3950` [linux]:

1. **Rails**: vddio (L9A 1.8 V), vddxo (L16A), vddrf (L17A), vddch0 (L23A) through RPM.
   Three of the four are shared with Wi-Fi, so with `wifi=1` running they are probably on
   already; L9A may be an always-on 1.8 V rail. Test first without votes, with Wi-Fi up.
2. **UART**: QUP0 SE3 in GENI UART mode at 115200 8N1. The repo has GENI SE code for I2C
   (SE1/SE2) and SPI (SE0, the Novatek touch driver, including the clock/RCG and SMMU traps
   found in touch stages A-C). SE3 needs the UART protocol: check `GENI_FW_REVISION` says
   UART (ABL may not have loaded UART firmware into SE3; the touch work found SE0 already
   had SPI firmware), then TLMM 8-11 to function `qup3`, `GCC_QUPV3_WRAP0_S3_CLK` on.
   FIFO mode with polling is enough to start; interrupt SPI 330 later.
3. **Power pulse**: flow control off, send the single byte `0xFC` at 115200, wait 100 ms.
   (Power off: byte `0xC0` at 2400 baud.) The chip does not answer HCI before this pulse,
   even with rails on.
4. **Version**: vendor command `0xFC00` sub-command `0x19` (EDL patch version request);
   the reply gives SoC id, ROM version and patch version. Print it: this is the first proof
   of life, comparable to the touch chip-ID stage.
5. **Baud rate up** (optional at first): QCA vendor baud command, then switch the host side;
   3.2 Mbaud is what Linux uses for this family. Stay at 115200 until the rest works.
6. **Firmware**: download `cmbtfwXX.tlv` then `cmnvXX.bin` with `0xFC00`/`0x1E` TLV segments
   (243 bytes each; most segments are sent without waiting for an event, per the TLV
   header's download mode, see `qca_tlv_check_data` and `qca_tlv_send_segment`). The
   BD address is tag 2 inside the NVM file; either patch it in the buffer or send
   `0xFC14` afterwards.
7. `HCI_Reset`, then standard HCI works: Read Local Version, Read BD_ADDR, LE scan.
8. **IBS** (in-band sleep): the chip may send the bytes `0xFD` (wake indication), `0xFE`
   (sleep indication) and expect `0xFC` (wake ack) outside H4 framing. These must be
   answered or stripped in the kernel and never passed to iOS. Simplest: clear the IBS /
   deep-sleep tags (17 and 27) in the NVM before download, as Linux does when IBS is off.

Firmware files come from the fog vendor image. Put them in the root next to the GPU files
(`/usr/share/firmware/topaz/`, installed by `build-rain/install-gpu-root.sh`) or embed them
with `mkpayload.py --blob` as was done for the touch firmware. Not in git.

Suggested staging, same style as the touch port, one boot each, results over the USB log:

| `bt=` | Does | Success line |
| --- | --- | --- |
| 1 | read SE3 firmware/protocol registers, TLMM 8-11, GPIO 87 state; change nothing | register dump |
| 2 | UART init, internal loopback | 16 of 16 bytes |
| 3 | power pulse + version request | SoC id 0x4007x130, ROM version |
| 4 | TLV + NVM download, HCI_Reset, Read Local Version, Read BD_ADDR | manufacturer 0x001D (Qualcomm), address |
| 5 | one LE scan from the kernel | advertisers listed |
| 6 | keep the chip up and expose it to iOS (part B) | |

Stage 5 proves the radio without any iOS involvement, which separates chip problems from iOS
problems (Scion did the same: its kernel scan found 21 LE devices before iOS saw any) [scion].

### B. iOS side

What iOS expects, as reported by Scion for iOS 16.5 on the same `D22AP` identity [scion]
(we run 16.7.16; re-check each point against our binaries with `strings`):

1. **A `bluetooth` service/node with a real address.** `bluetoothd` looks up `marconi-bt` or
   `bluetooth`; without `local-mac-address` it logs "No bluetooth on this device" or "Not a
   valid MAC address (all 00s)" and exits. Properties: `local-mac-address` (6 bytes, display
   order), `transport-speed`; `transport-encoding` absent means UART. We can write this
   node in `tools/mkadt.py` next to the existing `wlan` node; if `bluetoothd` reads the
   IORegistry rather than the ADT, publish an IOService as Scion did.
2. **The UART is a kernel control socket, not a tty.** BlueTool's `device -D` opens a
   `PF_SYSTEM`/`SYSPROTO_CONTROL` stream socket named `com.apple.uart.bluetooth`. The kernel
   side registers it with `ctl_register` and `CTL_FLAG_REG_SOCK_STREAM` (a datagram control
   fails with EPROTOTYPE). It must answer `TIOCGETA` (termios with `CS8|CREAD|CLOCAL|CRTSCTS`
   and the speed) and accept `TIOCSETA/W/F`; reads and writes carry raw H4 bytes
   (`ctl_enqueuedata` towards the client). `kern_control` is part of open-source XNU, so
   this is ordinary kernel code.
3. **bluetoothd's transport** is chosen by an environment variable in its launchd job:
   `BT_HCI_TRANSPORT=H4:POSIX:com.apple.uart.bluetooth,<baud>`. Plain `H4` crashes it; the
   default is Apple's `com.apple.uart.sk.bluetooth` (a skywalk channel we do not have).
4. **BlueTool must not load Apple's firmware.** It runs `/etc/bluetool/D22AP.boot.script` and
   `.init.script` only when MobileGestalt `ReleaseType` is `Vendor` or `Internal`; otherwise
   it picks a Broadcom firmware by product hash. Scion injects a small library into BlueTool
   that answers `ReleaseType` and uses scripts containing just `device -D -S` and `hci info`.
   We have the injection path already (`topaz_inject`, 0919/0947, `userland/gpu/topaz_inject.c`).
5. Power, reset and wake requests from iOS go to device nodes that do not exist; the kernel
   keeps the chip powered. The Bluetooth switch cannot turn the chip off (acceptable).

### The part with no precedent: Broadcom commands on a Qualcomm chip

The iPhone X has a Broadcom Bluetooth chip on a UART, and `bluetoothd` is built for it. The
S9 also has a Broadcom chip, so Scion never had to deal with this. After HCI_Reset,
`bluetoothd` will send Broadcom vendor commands (opcodes `0xFCxx`: sleep mode, PCM/I2S
setup, TX power tables, Apple's own LE extensions). On a WCN3950 the same opcodes mean
something else: `0xFC00` is Qualcomm's patch download, `0xFC0B` its NVM access.

Plan: a filter inside the control socket's write path.

- Standard commands (OGF 1-8) pass through unchanged.
- Vendor commands (OGF 0x3F) are never sent to the chip. The kernel answers each with a
  Command Complete, status 0, and logs the opcode and parameters.
- From the log, find which vendor commands need a return payload (version or feature
  queries) and give them plausible Broadcom-shaped answers; find any that `bluetoothd`
  treats as fatal when "unsupported".
- Also possible: `Read Local Version` may need the manufacturer field to say Broadcom
  (0x000F) if `bluetoothd` refuses other vendors. Rewrite in the read path only if needed.

Unknown until tried: how many such commands there are and whether any carry behaviour iOS
depends on (LE scanning and pairing use standard HCI, so the basics should survive) [guess].
Getting a list beforehand is cheap: `strings` and a disassembler on `bluetoothd` and
`BlueTool` from the 16.7.16 root on the PC.

### Bluetooth audio

Calls and music over Bluetooth are a separate project. On this platform SCO and A2DP offload
run through the ADSP over SLIMbus (`btfmslim`), and rain's ADSP speaks APR/Elite, for which
no driver exists yet (boot log 64, 69). A2DP could instead go over HCI ACL in software, which
is how `bluetoothd` normally sends it, but the phone has no working speaker path either.
Target for now: on/off, scan, pairing, LE (keyboards, data), in that order.

### Coexistence

One chip and one 2.4 GHz antenna path for both radios. The firmware arbitrates when both
sides are configured; on Android the WLAN driver sends BT coex parameters through WMI.
Expect Wi-Fi throughput to drop during Bluetooth scans. Not a first-order problem.

## 4. Order of work

1. Section 1 checks on the PC (rain.dts nodes, firmware file names). 30 minutes.
2. Wi-Fi: `modem=1`, then `wifi=1`, fix regions/rails as the log says.
3. Bluetooth A, stages `bt=1` to `bt=5`. Each stage is one kernel build and one boot.
4. Bluetooth B: `bluetooth` node, control socket, launchd job environment, BlueTool
   injection and scripts, then the vendor-command filter driven by the log.

## 5. Risks

- **EFS partitions** (`modemst1/2`, `fsg`, `fsc`, `persist`) are unbacked. All access stays
  read-only. A backup needs a recovery with working USB or `dd` from a booted Linux.
- The modem boot is the first thing on rain that starts the cellular DSP. It does not
  transmit on cellular without further QMI setup, and none is planned.
- UART SE3 may lack GENI UART firmware from ABL (stage `bt=1` answers that). Loading SE
  firmware from the kernel is possible (Linux `qcom_geni_se` does it on newer kernels) but
  is extra work.
- The vendor-command filter is open-ended.

## Sources

- This repo: patches 0113-0130, 0145, 0900, 0990, 0998, 0999; `docs/UPSTREAM-topaz.md`; `tools/mkadt.py`
- Linux mainline: `drivers/bluetooth/hci_qca.c`, `btqca.c`, `btqca.h`; `drivers/net/wireless/ath/ath10k/snoc.c`; `arch/arm64/boot/dts/qcom/sm6115.dtsi`, `qrb4210-rb2.dts`
- https://github.com/MakrSas/topaz-woa `docs/RESEARCH_wifi.md`, `docs/AGENT_BRIEF_drivers.md`
- https://github.com/Scion-Project/hackios-xnu-kernel/pull/166 and /pull/171 (served as Scion-Project/scion-kernel), https://github.com/Scion-Project/hakios-starlte/pull/44
- https://github.com/seemoo-lab/internalblue/blob/master/doc/ios.md (BlueTool, UART vs PCIe iPhones)
- https://naehrdine.blogspot.com/2021/02/internalblue-perfect-bluetooth-research.html
- https://wiki.postmarketos.org/wiki/Xiaomi_Redmi_10C_(xiaomi-fog) (Wi-Fi/BT untested there)
