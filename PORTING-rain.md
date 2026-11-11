# iOS on Redmi 10C (`rain`): porting plan

Base: [MakrSas/ios-topaz](https://github.com/MakrSas/ios-topaz) @ `e477291` (Redmi Note 12 4G, SM6225 "khaje").
Hardware source: Xiaomi stock firmware `miui_FOGGlobal_V14.0.9.0.TGEMIXM` →
`vendor_boot` dtb #08 (Khaje SoC) + `dtbo` #34 ("RAIN KHAJE IDP nopmi", board-id 0x60022),
merged into `ios-topaz/dtb/rain.dts` (gitignored).

## Same as topaz (verified in the device tree)

| Item | rain | topaz |
| --- | --- | --- |
| SoC | khaje, msm-id 0x206 | khaje (SM6225) |
| GICv3 | 0xf200000 | 0xf200000 |
| UFS | 0x4804000 (fastboot: `variant: SM_ UFS`) | 0x4804000 |
| Splash framebuffer | 0x5c000000, 0xf00000 | 0x5c000000, 0xf00000 |
| Gold cores (MPIDR) | 0x100–0x103 | 0x100–0x103 |
| Power controller | RPM (rpm-smd), not RPMh | RPM |
| Vol+ | PMIC GPIO5 | PMIC GPIO5 |

So the shim, MMU, GIC, timer, UFS, ARMv8.0 work (patch 0081) and the Gold-core boot path should carry over unchanged.

## Different, needs work

1. **Screen: 720×1650** (topaz 1080×2400). Three possible panels in the DT (nt36528 Helitai, ft8006s Xinli,
   nt36525b BOE), all 720×1650 video mode; ABL picks one. Hard-coded 1080/2400 in patch 0114 and the
   framebuffer code. The iPhone X canvas (1125×2436) is wider than the panel, so it must be scaled
   (×0.64 → 720×1559), not just centred/cropped. **First milestone only needs the console stride/size right.**
2. **Touch: Novatek NT36xxx over SPI**, QUP0 SE0 `spi@4a80000` (IRQ 0x147), chip at CS0, 9.6 MHz,
   reset TLMM GPIO 86, IRQ TLMM GPIO 80, coords 0–719 × 0–1649.
   Topaz has FocalTech FT5452 over **I²C** on QUP0 SE2. Needs a new SPI-GENI path and the Novatek protocol
   (patches 0083–0090 are the model to follow). Some rain units may ship a FocalTech chip on the same bus
   (`compatible` lists both).
3. **RAM size**: memory node is filled in by ABL at boot. The default `mkadt.py` range (0xb0000000 + 0x4d000000)
   should exist on any 3 GB+ unit; `--highmem` assumes 4 GB at 0x1_40000000 and must not be used blindly.
4. **Logs**: the TWRP on this phone has no working USB, so topaz's "pull log ring from recovery" path is unavailable.
   Need a working recovery for rain, or use the on-screen console only, or port a USB live log.
5. **Wi-Fi / modem / GPU / audio**: not checked yet. Later.

## Boot log

| # | Date | Image | Result |
| --- | --- | --- | --- |
| 1 | 2026-11-07 | shim only, 720×1650, SCALE 3 (be66bb1) | ABL accepted boot v4; red console drawn, a8r8g8b8 OK; text cut off at the right |
| 2 | 2026-11-07 | shim only, SCALE 2 (d8d5349) | All readable. Silver MIDR 51AF8014 (no 16K) → PSCI CPU_ON 0x100 OK → Gold MIDR 411FD090 (A73), tg16=1. GICD 0x12, GICR 0xf380000; vtimer 30 irqs/300 ms, SGI1 10/10; GICR+ 0x101–0x103 ready. CPUFREQ max 2400 MHz. `SPEED silver=0x7B9DA gold=0x1D4C34`: gold ~3.8× slower than silver in the loop (open question, not blocking). LOADER: no payload (expected) |

| 3 | 2026-11-07 | shim + XNU (CI run 42414358196, Xcode 16.0, no chained fixups, plain unslid pointers), ADT 720×1650, `cpus=1 rd=ufsroot0 wdt=-1 sweettouch=0`, no ufsrw | **XNU runs.** Console scrolls: device-tree platform expert, AKS stub, IOWatchdog, CPUFREQ gold 2400 MHz, I2C1 SE1 ready, AMFI, CPU 1/1 (mpidr 0x100), BSD banner, MAC framework, battery static fallback, FB DSI0 + MDP VIG0 direct scanout from 0x5c000000 stride 2880, AMFI XNL, security policy loaded. Memory, GIC, timer and framebuffer values are right. Last line in photo: "Security policy loaded: Guarded vnode policy" |

| 4 | 2026-11-07 | as #3 + `sweetchg=0 io=0x4` | Identical screen, same last line, no extra io= output. In both #3 and #4 the last line sits on the bottom console row (51 rows × 32 px; 18 px red strip below). `SWEET FB: console on shadow frame` (patch 0097) never printed, so the console still draws into the uncached boot framebuffer. Hypothesis: first console scroll on the device-mapped framebuffer hangs (topaz's 75-row screen never scrolls during boot) |
| 5 | 2026-11-07 | as #3 + `sweetchg=0`, console height faked to 800 px (25 rows), not committed | Console scrolled fine many times before AppleCLCD; froze right after `MDP VIG0 … direct scanout ready`. With the patched source (CI buildtree, `research/bt`): AppleCLCD::start never reaches `console double buffered` nor `vsync … the panel`. Between them: 3 physically contiguous 4.75 MB allocations below 4 GiB + 1 more, memcpy from the boot FB, `sweet_console_retarget` (takes the putc lock), flipFrom (plain copy + MDP register writes, no waits). Other thread keeps printing until it needs a scroll, then blocks too. Suspect: contiguous allocation blocking with only 1.2 GiB (default mkadt range 0xb0000000+0x4d000000) given to XNU |
| 6 | 2026-11-07 | shim only + `/memory` bank print (f684b8f) | DDR (MiB): 0x400+0x3DD, 0xC00+0x800, 0x800+0x400 → 0x40000000+989 MiB, 0x80000000+1 GiB, 0xC0000000+2 GiB (4 GB phone). 0x80000000–0x140000000 is one 3 GiB range; no fixed reserved-memory above 0x63900000 in rain.dts |
| 7 | 2026-11-07 | as #4 without `io=`, ADT `MIDSIZE=0xc0000000 mkadt.py --midmem` (dram 0x80000000 + 3 GiB) | Memory took effect (3883 buffer headers vs 1528), same freeze: one more line, wrapped to the top row, then nothing. Not a memory shortage |
| 8 | 2026-11-07 | kernel with patch 0960 (step1–7 IOLogs in AppleCLCD::start, `sweetfb_noshadow=1` skip), CI 42416957974, 3 GiB ADT | step1 (fsz 4752000), step2 (flip buffers 0x839b4000 / 0x83e4…), step3 (fade buffer) on the bottom rows; first photo also shows `step4 shadow 0xfffffe4ce4a08000, copying boot frame` wrapped onto the top row (user reads step3 as last; second photo crops the top). Hang is the fCons contiguous allocation or the memcpy from the boot FB surface (`fPixels`: sweet_surface_create_physical(0x5c000000), write-combined map). No step5 |
| 9 | 2026-11-07 | same kernel, `sweetfb_noshadow=1` (image `build-rain/shim-xnu-rain-6-noshadow.img`) | **MILESTONE 1 DONE.** Past AppleCLCD; reaches root mount: `BSD root: ufsroot0, major 1, minor 0` (UFS driver reads userdata on rain), `hfs_ValidateHFSPlusVolumeHeader: unknown Volume Signature : 41dd` (Android data, as expected), `cannot mount root, errno = 19`, `mount(N) failed`, retries every ~5 s (mount(5)…mount(8) seen), no panic. `SWEET INTF1: 317–318 frames in 5 s` → panel at ~63.5 Hz, display pipeline alive. Read-only: nothing written |

| 10 | 2026-11-07 | userdata flashed with iOS 16.7.16 root (`fastboot -S 256M flash userdata ios16-root.img`, 47/47 chunks, 318 s; image SHA-256 c1cf335f…47e6). Boot `shim-xnu-rain-ios1.img`: kernel 0960, 3 GiB ADT with D22AP /product, `cpus=1 rd=ufsroot0 unentitled_ios_sim_launch=1 ignition_level=1 wdt_test=no_user_checkin ufsrw=1 rootrw=1 launchd_unsecure_cache=1 wdt=-1 sweettouch=0 sweetchg=0 sweetfb_noshadow=1` | **Apple logo on the 10C screen** (white logo centred, console text gone). Root mounted from userdata and iOS userland started far enough to switch to the boot logo. Faint ghost shapes on the panel. Waiting to see if it gets past the logo |

| 10b | 2026-11-07 | same boot #10, left running (the `-v` image was built but never booted) | **MILESTONE 2: iOS 16 lock screen on the Redmi 10C**, after well over 10 minutes on the Apple logo (first-boot setup on one core). Lock-screen clock ("4:16") on the blue default wallpaper, drawn by iOS through AppleCLCD even with `sweetfb_noshadow=1`. Picture is the centre 720×1650 window of the 1125×2436 iPhone X canvas (date line and status bar cropped above, clock cut by the notch) — canvas offsets still assume a 1080 px panel. Buttons: user reports iOS "notices" key presses but nothing reacts yet. Touch off (`sweettouch=0`, no Novatek driver) |

| 12 | 2026-11-08 | kernel 0961 (CI 42423414706), `+ sweetfb_fit=1` | **Fit works**: whole lock screen (status bar, date, clock, camera button, home bar) scaled to 720×1559 with 45 px bars |
| 13 | 2026-11-08 | shim panel probe (3f23475) | `PANEL c3q_35_02_0a_fhdp_video` = "nt36525b video mode dsi boe panel": **Novatek NT36525B TDDI touch**, SPI on QUP0 SE0 `spi@4a80000`, reset GPIO 86, IRQ GPIO 80 |

**Touch facts (Xiaomi driver `drivers/input/touchscreen/nt36525b_spi` in github.com/r0ddty/kernel_xiaomi_fog, copy in `research/nt36525b/`, GPL, not committed):** BOE vendor → firmware `novatek_ts_fw.bin` loaded by the host on **every boot** (`BOOT_UPDATE_FIRMWARE 1`); SPI mode 0, 8-bit; register address high bit marks write (`SPI_WRITE_MASK(a) a|0x80`). Firmware file has to come from the stock vendor partition.
| 14 | 2026-11-08 | kernel 0962 (CI 42426923800), `raintouch=1 sweettouchdelay=2 -v` + fit boot args | Panic `RAIN TOUCH A: stage 'nvt reset' err=-4 xfers=0`: gcc vote 0x5fe0, S0 RCG cfg 0x1, cbcr 0x80004220 (off at first read; wait for on passed); **se0 fw=0x102 (protocol 1 = SPI, loaded by ABL)**, status 0, hwtx 0x20102964 / hwrx 0x20204800 (32-bit FIFO, 16/32 words); gpio0-3 0x3 → 0x84 (qup0, 6 mA, no pull). **First SPI command (3-byte TX_ONLY page select) never reported M_CMD_DONE within ~20 ms.** Probe ran after SpringBoard started (touch init is late), console still visible with `-v` |

| 15 | 2026-11-08 | kernel 0963 (CI 42427721183), same args | `RAIN TOUCH A2: 'nvt reset' err=-4 xf=0 cbcr=0x4220 \| FAIL op=1 n=3 irq=0 st=0x41 txfs=0 rxfs=0 ios=0x5 sirq=0 \| CFG mclk=0x101 sel=0 word=0x4 cpha=0 cpol=0 demux=0 trans=0 cgc=0x7f dma=0 pack=0x7f8fe`. Command accepted (GENI_STATUS M_CMD_ACTIVE), TX FIFO drained, **no M IRQ bit ever** (no done, no error); every SPI config register reads back as written. Suspect: SE0 serial clock not really running (wrong gate register watched, or RCG root off) |

| 16 | 2026-11-08 | kernel 0964 (CI 42428335983), same args | `RAIN TOUCH A3: 'nvt reset' err=-2 retry=-2 xf=0 \| CLK cmd=0 cmd2=0x2 cfg=0x1 b144=0x4220 b138=0 vote=0x5fe0 \| SE2 cmd=0 cfg=0x1 \| FAIL op=1 n=3 irq=0 st=0x41 txfs=0 ios=0x5`. S0 RCG root ON, same config as working SE2, branch 0x1f144 running: **not a clock problem**. First command still hangs at 200 ms; M cancel does not free the engine (next command -2 busy). Next suspect: GENI_IF_DISABLE_RO (0x64) FIFO_IF_DISABLE set on SE0 → FIFO writes ignored, SE0 usable only through GPI DMA (DT gives it `dmas`). |

| 17 | 2026-11-08 | kernel 0965 (CI 42429044274), same args | `RAIN TOUCH A4: 'se0 fw' err=-15 \| IF ifdis=0x1 fw=0x102 fws=0x102 force=0 st=0`. **Confirmed: SE0 has FIFO_IF_DISABLE set — GPI DMA only.** Probe stopped before sending anything (engine idle). Touch needs a GPI DMA path (transfer/event rings, channel alloc/start for SE0 TX/RX, CONFIG0 + GO + DMA TREs, doorbell, completion event) like Linux spi-geni-qcom's GENI_GPI_DMA mode; no topaz/HackiOS code for GPI exists. |

| 18 | 2026-11-08 | kernel 0966 (CI 42430216869), same args | **Phone reset with no panic and came up in TWRP.** Something below XNU (hyp/TZ) reset the SoC during the GPI bring-up or the first GPI transfer; prime suspect: GPI DMA through SMMU stream 0xf6 not mapped for us. TWRP USB still not enumerating, so the sweetlog ring could not be read. Next: per-step console breadcrumbs with pauses (0967) |

| 19 | 2026-11-08 | kernel 0967 (CI 42430986430), same args | Last breadcrumb `RAIN B: 2 se0 gpi mode`, then reset (no panic; user also saw an unrelated `sig 9 driverkitd` crash line). Step 1 (GPI map, contiguous alloc, **reads** of GPII 0 ch/ev contexts) passed. Reset is inside step 2: SE0 GPI-mode writes (0xe1c=0, M_IRQ_EN=0, DMA_MODE_EN=1, GSI_EVENT_EN\|=0xf) or the first GPI register **write** (GPII 0 INTSET=1). Next: one breadcrumb per write (0968) |

| 20 | 2026-11-08 | kernel 0968 (CI 42432630938), same args + `raintouchstop=3` | Apple logo, then panel **fully off**, no reboot within 3 min, no panic visible; user hard-reset. Reading: iOS kept running and turned the display off (a halted panic could not do that) → the touch thread most likely hung inside steps 1–3, but a panic drawn into the boot framebuffer after iOS took over the scanout would also be invisible. Also: B2's "last visible line" was unreliable (iOS frames cover the console once SpringBoard draws). Next (0969): end each probe by rebooting into fastboot (IMEM restart reason 0x77665500 + PSCI SYSTEM_RESET), so the outcome is readable without the screen: fastboot = reached stop point, other reboot = crash, no reboot = hang |

| 21 | 2026-11-08 | kernel 0969 (CI 42433766292), `raintouchstop=3 raintouchfb=1` | Landed in **TWRP** (watcher's "fastboot after 77 s" was the user's manual restart) |
| 22 | 2026-11-08 | same kernel, `raintouchstop=1 raintouchfb=1` (control: step 1 known safe) | Landed in **TWRP** untouched → the IMEM "bootloader" restart reason is **not** honoured when set from inside XNU (shim-only probes did return to fastboot), and any reset lands in TWRP. Consistent reading of #19–#21: **steps 1–3 safe** (GPI map + alloc + reads, SE0 IRQ routing off, SE0 DMA mode); the unstopped run crashed in steps 4–12. Signal from now on: panic ending (no `raintouchfb`) → stop point reached = phone stays on, no reboot; crash = reboot into TWRP. Bisect next: `raintouchstop=7` |

| 23 | 2026-11-08 | same kernel, `raintouchstop=7` (panic ending) | Stayed on, no reboot (panel off) → **steps 1–7 safe** (+ SE0 GSI events, GPII 0 INTSET, event ring alloc + context + doorbell). Crash is in 8–12 (channel alloc 8/9, start 10/11, first DMA doorbell 12). Next: `raintouchstop=10` |

| 24 | 2026-11-08 | same kernel, `raintouchstop=10` | Stayed on, no reboot → **steps 1–10 safe** (ch0/ch1 alloc + context, ch0 start). Crash in 11 (ch1 start) or 12 (first DMA doorbell). Next: `raintouchstop=11` |

| 25 | 2026-11-08 | same kernel, `raintouchstop=11` | Stayed on, no reboot → **GPI bring-up complete and safe (steps 1–11)**: SE0 in GPI mode, event ring, ch0/ch1 allocated and started. **The SoC reset comes from step 12, the first DMA doorbell** (GPI fetching our TREs from RAM). Matches the DT: GPI behind apps SMMU, `iommus = <&apps_smmu 0xf6 0>`, Android maps it in `qcom,iommu-dma-addr-pool = <0x100000 0x100000>`. Under XNU nothing programs the SMMU for SID 0xf6 → fault → reset. Next: SMMU stream setup (bypass or a mapping) for SID 0xf6 |

| 26 | 2026-11-08 | kernel 0970 (CI 42435412954), no stop, `sweettouchdelay=0 raintouchpause=0` | **Rebooted into TWRP.** Either the SMMU identity-bank writes for SID 0xf6 (step 12) reset the SoC, or the first doorbell (step 13) still faults with the bank in place. Next: same kernel, `raintouchstop=12` with pause 0 (panic visible before iOS draws, shows `smmu=`) |

| 27 | 2026-11-08 | same kernel, `raintouchstop=12 raintouchpause=0 sweettouchdelay=0` | **Panic visible.** All steps 1–12 printed; `TopazModem: smmu: sCR0 002e0406 ID0 4c017e32 ID1 50000030: 50 SMRs, 48 CBs, page 1000 x64`; `GPI0 identity (sid f6 mask 0) CB4 SMR5: SMR 800000f6 S2CR 00000004 SCTLR 000000e0 -> OK`; `smmu=1`; ev 0x10100000 (allocated), go0 0x1020000a / go1 0x10200102 (both channels started, SPI, out/in); ring pa 0x848d4000. So SMMU programming is safe, yet #26 still reset at the first doorbell with this bank in place. sCR0 has USFCFG=1 (unmatched streams fault) and GFRE/GFIE set: suspect a GPI access with another SID → global fault → reset. Next (0971): clear sCR0 GFRE/GFIE before the doorbell and capture sGFSR/sGFSYNR*/sGFAR and the CB FSR/FAR |

| 28 | 2026-11-08 | kernel 0971 (CI 42436207478), no stop, pause 0, delay 0 | **No reset — first GPI DMA ran.** `cr0 0x2e0406->0x2e0400` (GFRE/GFIE off accepted), `gfsr=0` (no global fault, no foreign SID), **ch0 rp0 = wp0 = 0x848d4030** (GSI fetched all 3 TREs), **evrp 0x848d4810** (one event written). `err=-32` was a false positive in my check: `cbfsr=0x400` is only FSR.FORMAT (bits 10:9 = 0b10, AArch64), not a fault; cbfar/cbfsynr0 stale. The probe broke out before reading the event (`ev n=0`). Fix (0972): fault masks CB FSR 0xc00001fe, sGFSR 0x800001ff. Open: #26 (GFIE on) reset — whether keeping GFRE/GFIE off is required stays to be seen |

| 29 | 2026-11-08 | kernel 0972 (CI 42436819713), no stop, pause 0, delay 0 | **Five GPI SPI writes completed, no faults, no reset**: `'done' err=-31 xf=5`, 5 transfer-complete events (`w3=0x220003`, code 2 EOT, len 3), SMMU clean (`gfsr=0 cbfsr=0`). TX ring rp 0x110 / wp 0x120: the engine stopped at the DMA TRE of the first **full-duplex read**. Linux gpi.c: duplex GO = CHAIN\|LINK (mine: CHAIN only); only the TX DMA TRE gets IEOT. Fix (0973): LINK on duplex GO, no IEOT on the RX DMA TRE, duplex done = TX event + RX ring RP reaching WP |

| 30 | 2026-11-08 | kernel 0973 (CI 42486361671) | `'done' err=0 xf=8`, 10 events, no faults: duplex now completes, but `id 00 …` |
| 31 | 2026-11-08 | kernel 0974 (CI 42487161862, + Novatek eng reset 0x5a→0x7fff80) | `xf=10`, 12 events, `id` and `old` all 00 |
| 32 | 2026-11-08 | kernel 0975 (CI 42487862032, loopback + 0xa5 fill) | `xf=11`, **14 events**, `id`, `old` and **`lb` all a5**, pins 0x84 ×4. RX buffer untouched when copied. 14 events = 8 TX-only ×1 + 3 duplex ×2 → **the RX channel posts its own transfer-complete event (chid 1)**; my wait ended on the TX event + RX ring RP (which moves when the TRE is fetched, before data lands), so the copy ran early. Fix (0976): duplex done = TX event (chid 0) **and** RX event (chid 1) |
| 33 | 2026-11-08 | kernel 0976 (CI 42488745988, wait for RX event chid 1) | `err=0 xf=11 n=14`, RX event `rx=0x1220003` (XFER_COMPLETE, chid 1, status 3), `imm=0`; `id`/`old`/`lb` still all a5. RX completes but the buffer is unchanged, so 0977 reads it through an uncached `ml_io_map` alias (cache vs. DMA never writing) and reports the RX event TRE pointer and length/code |
| 34 | 2026-11-08 | kernel 0977 (CI 42489841903, uncached RX alias) | `uc=1`; **uncached and cached views both a5**. RX event: TRE ptr 0x84ad4420 (RX ring entry 2), w2 0x2000008 = length 8, code 2 (EOT). GPI event-ring writes land in the same block, RX data does not → suspect the QUP0 wrapper stream (SID 0xe3) writes RX data under a loader-left SMMU mapping. 0978: record SMR for 0xe3, identity bank for it, scan all CBs for faults |
| 35 | 2026-11-08 | kernel 0978 (CI 42490772811, identity bank for QUP0 SID 0xe3) | **RX works.** `lb 11 22 33 44 55 66 77 88` (loopback echoes TX), **`id 0b 00 00 25 65 03` = NT36525B trim ID** (Linux trim table {0x0B,0xFF,0xFF,0x25,0x65,0x03}, mask 1,0,0,1,1,1). SID 0xe3 had no SMR (5 valid SMRs, slot 255) and no fault was recorded: its writes were silently dropped. Cached view `ff ff 0b 00` = the last read (2 dummy bytes, then 0b 00), so cache maintenance was fine all along |
| 36 | 2026-11-08 | kernel 0979 (CI 42492125916) + payload `--blob TouchFW=novatek_ts_fw.bin` | C10: rings wrap, events via uncached alias, TX chunks 4 KiB; firmware download (hw-crc flow, 19 partitions, 0x13f02 bytes, host-tested against the blob), fw info, 30 s point-buffer poll |
| 37 | 2026-11-08 | C10 boot 2 (16:04:06) | First Apple logo, then black and never back: **no panic, no reboot** (boot 1 reached the logo too, outcome not seen). Unknown where it stopped: download hang, poll loop, or the panic not drawn. Next: raintouchfb=1 or breadcrumbs per C10 stage, shorter poll |
| 38 | 2026-11-11 | kernel 0980 (C11) | Reading of #37: a failed download would panic within seconds on the text console (as C9 did); logo first, black later = the 30 s poll ended and the panic fired after iOS owned the display. So the download probably worked. C11 keeps running after the download and feeds touches to `sweet_hid_touch` (x·3/2, (y−45)·2400/1559); panics only on bring-up failure or `raintouchpanic=1`. Also fixes a C10 off-by-one in the point-buffer offset |
| 39 | 2026-11-11 | C11 boot (17:15:01, CI 43053502074) | Reaches the lock screen, no early panic (so bring-up reported no error); **swipe does nothing**. Next: same kernel with `raintouchpanic=1` to read the firmware state |
| 40 | 2026-11-11 | same kernel, `raintouchpanic=1` (17:23:15) | **Firmware is up**: `parts=19 wrote=0x13f02 err=0 rst a0 06`, `INFO 0d f2 x18 y32 max 720 1650` (fw 0x0d, 18×32 sensor grid), xf=95, no SMMU faults. Open: does the point buffer show fingers, or does iOS ignore the HID events. 0981: `raintouchpanic=2` panics at the first finger with panel + HID coordinates |
| 41 | 2026-11-11 | kernel 0981 (CI 43058211406), `raintouchpanic=2` (17:37:01) | **The driver sees fingers.** `TOUCH polls=17 downs=2 panel 560,1065 … raw 0a 23 42 09 01 01` (id 1, status 2 = move, x 0x230, y 0x429), xf=129, no faults. So C11's "no reaction" is on the HID / iOS side |
| 42 | 2026-11-11 | same kernel, run mode + `sweetfb_console=1 sweetlog=HID,RAIN` (17:42:03) | `sweetfb_console` has no effect on rain (iOS is shown, no text). Lock screen is **live**: clock runs, "swipe up to open" animates, **volume and lock keys work** (HID path to backboardd is fine). Touch still does nothing |
| 43 | 2026-11-11 | kernel 0982 (CI 43061792849), `raintouchpanic=3` (17:55:14) | **Clean single-finger tracking.** `T3 frames=300 empty=137 maxfingers=1 ids=0x2 id1 333,1013 -> 442,887 hid down=117 up=3`; first frame has one record (`0a 14 3f d5 01 01`). No ghost contact: the two contacts in #41 were real. Console showed `SWEET HID: touch` lines. So the driver hands clean events to SweetHIDTouch; why iOS does not react on the lock screen is still open |
| 44 | 2026-11-11 | kernel 0983 (CI 43063511499), run mode (18:04:42) | **TOUCH WORKS in iOS.** Swipe up opens the lock screen. Cause of the dead lock screen confirmed by the fix: the TDDI chip loses its RAM firmware when iOS takes the panel; the 1 s liveness check + reload (0983) brings it back |

**Touch firmware found:** stock vendor (fog Global V14.0.9.0, OTA SHA-256 verified) `/vendor/firmware/novatek_ts_fw.bin`, 118,784 bytes, SHA-256 `234f437f…4b86`, in `research/fw-fog/` (proprietary, not committed). Same folder: `novatek_ts_hlt_fw.bin` (Helitai/nt36528) and `focaltech_ts_fw_xinli.bin` (Xinli/ft8006s) for the other two panels.
**Topaz touch structure (CI buildtree, `research/bt/xnu/bsd/dev/sweet_touch.c`):** GENI SE register helpers, TLMM pin setup (`touch_gpio`), clocks (`touch_clocks`), SE init, a reader thread, events to iOS via `sweet_hid_touch(finger, down, x, y)` (SweetHID.cpp). Plan: a rain Novatek path with the same shape on QUP0 SE0 in SPI mode.
**Touch stages:** A) SE0 SPI bring-up + chip/trim ID read (no firmware); B) firmware upload + checksum; C) event read → `sweet_hid_touch`, panel→logical through the fit transform (x·25/16, (y−45)·25/16).
**Audio facts:** codec WCD937x + Bolero, same as topaz; speaker amp is Awinic AW87xxx (topaz: SIA8159) → speaker needs a new amp driver; headphone path may work with `adsp=1`.
**Wi-Fi:** topaz `modem=1 wifi=1` (modem PAS boot from `modem_a` via TrustZone, WLFW); untested on rain.

Flash notes: ABL does not report `max-download-size` and rejects a 3.4 GB chunk → `-S 256M`; re-sparsing the img2simg file failed ("Error reading sparse file") → flash the raw .img; ABL's fastboot stops answering after aborted or long transfers → Vol-+Power restart before the next command. A flaky cable caused repeated USB re-enumeration (device number climbing).

**Open issue:** with shadow/flip buffers enabled, AppleCLCD::start hangs at the fCons allocation or the memcpy from the boot FB surface (step 3/4 of patch 0960). `sweetfb_noshadow=1` is a workaround; iOS frames will need the flip buffers later.

## Milestone 2 (next): iOS root in userdata

**Source:** Apple CDN, `iPhone10,3,iPhone10,6_16.7.16_20H392_Restore.ipsw`, 5.75 GiB, still signed, SHA-256 starts `ece1194f7e233e91` (ipsw.me API):
`https://updates.cdn-apple.com/2025FallFCS/fullrestores/122-77006/1158E502-D76D-4226-AA06-ED5E30CD1111/iPhone10,3,iPhone10,6_16.7.16_20H392_Restore.ipsw`
Disk images inside (uncompressed sizes, read from the zip directory only): 087-86698-044.dmg 4.4 GiB (has .mtree/.root_hash: likely the OS root), 087-86593-047.dmg 2.6 GiB (likely SystemOS cryptex with the dyld cache), 087-86622-047 / 087-86719-047 ~100 MiB, 087-86715-047 10 MiB. Which is OS/SystemOS/AppOS must be confirmed from BuildManifest.plist (D22AP, Erase identity). Members are deflated in the zip, so downloading the whole IPSW (and checking its SHA-256) is simpler than partial fetches.

**Missing tools on this PC:** `apfs-fuse`, `hfsprogs` (mkfs.hfsplus / fsck.hfsplus), `lzfse` (for DeviceTree.d22ap.im4p → `research/ipsw-x/d22ap.adt`; mkadt copies `/product` from it, which MobileGestalt needs). All three are AUR packages here (`yay` is installed); installing needs sudo and is the user's call. Present: rsync, img2simg, fastboot. Disk: 357 GB free on /home.

**Steps:**
1. User installs `hfsprogs apfs-fuse-git lzfse` (AUR; PKGBUILDs not reviewed by me).
2. Download the IPSW (5.75 GiB) and verify SHA-256 (needs user's go-ahead; Apple's licence covers Apple hardware only).
3. Read BuildManifest → OS / SystemOS / AppOS dmg names; extract DeviceTree.d22ap → `research/ipsw-x/d22ap.adt`.
4. Review `tools/sweet-root-tweaks.py` fully, then user runs (sudo): `mkiosroot.sh OS SystemOS AppOS ios16-root.img 16G`, mount, `sweet-root-tweaks.py`, unmount, `img2simg`.
5. ADT again (midmem 3 GiB, now with /product); payload with topaz's iOS boot args incl. `ufsrw=1 rootrw=1` (first boot that can write) and `sweetfb_noshadow=1`.
6. In fastboot: check `partition-size:userdata` ≥ 16 GiB, then **`fastboot flash userdata ios16-root.simg` (destructive: erases the Android data; explicit OK right before)**.
7. `fastboot boot` → expect Apple logo, first boot ~4 min.

**Risks:** with `sweetfb_noshadow=1` the flip buffers do not exist, and iOS frames may not reach the panel (AppleCLCD presents through them); the step 3/4 hang may have to be fixed first. The iPhone X canvas (1125×2436) is still laid out for a 1080 px panel (patches 0061/0114 hard-code 1080/2400): expect a cropped or misplaced picture until that is ported. Touch stays off (Novatek SPI driver not written).

Build notes: shim builds with clang/lld (wrapper adds `-fno-jump-tables`); kernel CI moved to `macos-15` (macos-14 runners retired).
Kernel boot args for milestone 1 must NOT include `ufsrw=1`/`rootrw=1`: every UFS write path (incl. the sweetlog ring) is gated by `ufsrw`.

## Milestone 1: kernel console on the 10C screen

- Fork ios-topaz (needs user's GitHub account; kernel builds on GitHub macOS runners).
- Change only framebuffer geometry (720×1650, stride 720×4) in the shim and patches 0010/0012/0114; touch off (`sweettouch=0`).
- `fastboot boot` the shim. Success = kernel text visible, ideally reaching "waiting for root device".
- Nothing is written to the phone in this milestone.

## Milestone 2+: iOS root in userdata, then touch

- Download iPhone X iOS 16.7.16 (20H392) IPSW from Apple (user's call: Apple's licence covers Apple hardware only).
- Build HFS+ root and `fastboot flash userdata` (erases the already-inaccessible data).
- Novatek SPI touch driver.

## Not done / unverified

- No partition backups (persist, modemst1/2, fsg, fsc): TWRP USB is broken. Topaz never writes those partitions.
- The phone's bootloader reports `rain`; Xiaomi's firmware is labelled `fog`. The dtbo has a dedicated RAIN overlay, so the
  hardware description matches, but which of the three panels this unit has is unknown until it boots.
- None of ios-topaz's scripts or patches have been run or fully read yet.
