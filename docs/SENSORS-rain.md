# Motion sensors on rain (accelerometer; "gyro")

Written 2026-11-12. Research and one probe patch (1005); no sensor data read yet.

## What the phone has

- **Probably no gyroscope chip.** Spec sites disagree, but the detailed listings and user
  reports say accelerometer, proximity and light only, no gyro, no compass
  ([Mobility Arena](https://mobilityarena.com/xiaomi-redmi-10c), GSMArena user thread). Android's
  "gyroscope" on such phones is computed from the accelerometer.
  Check on the PC: `ls vendor/etc/sensors/config/` in the fog vendor image. The `*.json` file
  names are the chips (for example `sc7a20`, `stk3x3x`); a gyro would show as `bmi160`,
  `lsm6ds*`, `icm4*` or similar.
- So the realistic target is the **accelerometer**: screen rotation, shake, tilt in games.
  Apps that need a real gyro (AR, some games) will not work on this hardware under any OS.

## Where the accelerometer hangs

Not on a bus the main CPU owns. The I2C scans (boot log 66) found no accelerometer on QUP0
SE1/SE2. On this SoC family sensors belong to the **ADSP's sensor hub** (SSC): the chip
driver runs inside the ADSP firmware, in the protection domain `msm/adsp/sensor_pd`, which
`TopazAudio`'s pd-mapper already announces.

Android reaches it through two user-space pieces. Their open-source equivalents on mainline
Linux phones are **hexagonrpcd** and **libssc**:

1. **A file server for the DSP** (Android `sscrpcd`, mainline `hexagonrpcd`). The sensor
   domain reads its configuration ("registry": `vendor/etc/sensors/config/*.json`,
   `sns_reg_config`, and calibration under `persist/sensors/registry`) by calling *back* into
   the main CPU over **FastRPC** (`apps_std` open/read/stat calls). The GLINK channel is
   there on rain: `fastrpcglink-apps-dsp` (boot log 69). Without this server the sensor
   domain most likely starts no chip driver.
2. **A sensor client** (Android's sensor HAL, mainline `libssc`): QMI service 400
   ("sns client") over QRTR, with protobuf messages: look a sensor up by type ("accel") to get
   its id, send an enable request with a sample rate, then receive events with three floats
   in m/s².

## Plan

| Step | What | Size |
| --- | --- | --- |
| 0 | Patch 1005: log line when service 400 appears. Boot `adsp=1` and read the log. If it appears with no file server, step 2 can be tried first | done, untested |
| 1 | FastRPC listener in the kernel on `fastrpcglink-apps-dsp`: attach to the sensor domain, answer `apps_std` file calls from files copied into the iOS root (for example `/usr/share/firmware/rain/sensors/`). Model: hexagonrpcd `listener.c`, `apps_std.c`; Linux `drivers/misc/fastrpc.c` for the message layout | large (comparable to the modem host services, patch 0121) |
| 2 | QMI client for service 400 with hand-packed protobuf (three message shapes), model: libssc | medium |
| 3 | iOS side: an IOHIDEventService that sends `IOHIDEvent::accelerometerEvent` (the in-kernel HID code already has it), at about 50 Hz. backboardd derives rotation from these | small, but which service properties CoreMotion wants on an iPhone X identity is unknown |

Step 3 can be developed first with **fake data** (gravity pointing down the screen, or
switched by the volume keys): it shows whether iOS rotates before any DSP work is done.

## Risks and unknowns

- FastRPC on this SoC passes buffers by physical address through an SMMU context bank owned
  by the ADSP; the driver must map its buffers for that stream, as was needed for touch DMA
  and audio (`topaz_smmu_identity`).
- The registry needs a writable directory in the Android design; a read-only answer may or
  may not satisfy the firmware.
- Sensor power rails are switched by the ADSP itself through RPM; if a rail is gated by a
  main-CPU vote on Android, it has to be found in rain.dts.
