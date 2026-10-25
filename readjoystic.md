# Joystick calibration and HEAD receiver handoff

This note records the HAND joystick calibration and the remaining checks for
the HEAD (ESP32 A) firmware. The calibrated HAND values are normalized control
values; HEAD must not treat them as raw ADC readings or apply the HAND ADC
calibration a second time.

## HAND pins and measured calibration

HAND joystick connections, as currently configured:

- X signal: GPIO 1
- Y signal: GPIO 2
- Switch: GPIO 14
- ADC resolution: 12-bit

The measured center values supplied for the joystick are:

```text
X_CENTER = 2800
Y_CENTER = 2780
```

Configured directional thresholds:

```text
X_LEFT_THRESHOLD  = 2000
X_RIGHT_THRESHOLD = 3500
Y_BACK_THRESHOLD  = 2000
Y_FRONT_THRESHOLD = 3500
JOY_DEADZONE      = 12
```

These settings are in `hand/include/config.h`. The HAND reads the raw ADC
values, normalizes each axis piecewise around its center, clamps to -100..100,
then applies the deadzone:

- X: raw 2000 -> -100, 2800 -> 0, 3500 -> +100.
- Y: raw 2000 -> +100, 2780 -> 0, 3500 -> -100. Y is inverted in HAND to
  preserve the existing control direction.
- Outputs with absolute value <= 12 become 0.

Values outside the configured thresholds clamp to the corresponding full
scale. Examples before deadzone:

| Raw X | Raw Y | HAND control `(x,y)` |
|---:|---:|---:|
| 2800 | 2780 | `(0,0)` |
| 2000 | 2780 | `(-100,0)` |
| 3500 | 2780 | `(+100,0)` |
| 2800 | 2000 | `(0,+100)` |
| 2800 | 3500 | `(0,-100)` |

The `raw=(...)` values in HAND diagnostics are the ADC readings. The
`value=(...)` values, `HAND STATE joystick=(...)`, and transmitted packet
fields are normalized controls. Calibration changes the latter, not the raw
ADC reading.

## HAND-to-HEAD packet contract

HAND sends a packed `V3ControlPacket` (type 3) over ESP-NOW every 50 ms:

- `magic = 0xB808`
- `mode = 0` means HEAD servo control; `mode = 1` means BODY control
- `x` and `y` are signed 8-bit normalized values in -100..100
- `zoom` is 0..100
- `sequence` increments for each control packet
- ESP-NOW channel is configured as channel 1

The packet definition is in `hand/shared/protocol.h` and
`baymaxhead/shared/protocol.h`. Keep packing, field order, type, magic, and
size identical. The calibration constants belong on HAND only: HEAD receives
already-normalized signed values.

## What the current serial evidence means

The HAND has previously printed:

```text
JOYSTICK STATE mode=HEAD raw=(0,0) value=(-100,100)
HAND STATE ... joystick=(-100,100)
```

This means the HAND ADC inputs were reading zero at that time. Since zero is
below both configured low thresholds, the calibrated conversion correctly
clamps X to -100 and the inverted Y to +100. If this remains the HAND output,
the root cause is still at the ADC/input/wiring/runtime level; changing the
HEAD calibration cannot correct it.

The user also observed raw values around `(2828,2803)` while moving the stick
and full-scale values near `(4095,4095)` in an earlier capture. Those ADC
readings did change, so the signals were observed working during that test.
At the measured center `(2800,2780)`, normalized output should now be close to
`(0,0)`. Re-test after upload and check both `raw` and `value`.

HAND's button press toggles HEAD/BODY mode. Confirm the mode is HEAD before
expecting the HEAD servo path to move.

## HEAD firmware checks

HEAD receiver code in `baymaxhead/src/main.cpp` should:

1. Accept only a type-3 packet with the expected packed size and `magic`.
2. Copy `mode`, signed `x`/`y`, and packet timestamp; increment
   `rxControlCount`.
3. Use the received signed values directly. Do not re-center or normalize
   these packet fields as if they were raw ADC values.
4. In mode 0, use nonzero values to update pan/tilt targets while the packet
   age is below `CONTROL_TIMEOUT_MS` (currently 600 ms).
5. Report receiver diagnostics separately from HAND diagnostics.

For one controlled test, add/enable a throttled HEAD log on each changed
control value (or at about 5-10 Hz) containing:

```text
HEAD CONTROL RX mode=<0|1> x=<signed> y=<signed> seq=<n> age=<ms>
```

Also inspect the existing one-second `RADIO` and `HEAD STATE` lines:

- If `rxControlCount` does not increase, troubleshoot ESP-NOW channel,
  registration, sender/receiver power, packet length, magic, and type.
- If the count increases and HEAD `x/y` equal HAND `value=(x,y)`, the radio
  control path works; investigate mode, timeout, servo limits, and servo
  hardware if motion is absent.
- If HAND `raw` is at its measured center and HAND `value` is near `(0,0)`,
  HEAD should receive near `(0,0)`, not `(-100,100)`.
- If HAND's `value` is `(-100,100)` and HEAD receives the same, HEAD is
  faithfully receiving the bad/extreme HAND input; return to HAND ADC and
  wiring diagnosis.

The HEAD source currently expects mode 0 and applies the received values as a
fraction of `JOY_SERVO_MAX_STEP_DEG` per 20 ms update. Check the HEAD's
`controlAge` stays below 600 ms and confirm servo power, shared ground, PWM
pins, and mechanical linkage independently.

## Status packet caution

A HAND capture previously showed a type-5 status line with fields such as
`ch=191`, `echo=65536us`, large receive counters, and a nonsensical control
age, while sensor packets were sometimes received. Those values are not
plausible for the configured channel 1 and may indicate a status packet
layout/version mismatch or corrupted/stale parsing. Before relying on type-5
fields, verify the HAND and HEAD are both running firmware built from matching
`shared/protocol.h` definitions (including the 66-byte `HeadStatusPacket`)
and that the receiver validates the exact packet size, magic, and type.
Do not infer joystick reception from a malformed status line; use the HEAD
`RADIO`/`HEAD STATE` serial diagnostics as well.

## Relevant source files

- HAND calibration: `hand/include/config.h`
- HAND pin mapping: `hand/include/pins.h`
- HAND ADC normalization and packet send: `hand/src/main.cpp`
- HAND packet contract: `hand/shared/protocol.h`
- HEAD receive, telemetry, and servo control: `baymaxhead/src/main.cpp`
- HEAD thresholds/timeouts: `baymaxhead/include/config.h`
- HEAD packet contract: `baymaxhead/shared/protocol.h`
