# HEAD firmware handoff: sensor, laser, and Action controls

This note is for the person maintaining the HEAD (ESP32 A) firmware. It records
what changed on both HAND and HEAD for diagnostics, what the connected HAND
reported, and which controls are implemented versus display-only.

## Projects and shared contract

- HAND firmware that was built/flashed:
  `~/Documents/PlatformIO/Projects/doom-world/hand`
- HEAD firmware inspected:
  `~/Documents/PlatformIO/Projects/baymaxhead`
- Camera firmware inspected:
  `~/Documents/PlatformIO/Projects/doomcamv4`
- HAND/HEAD shared ESP-NOW packet definitions are in
  `shared/protocol.h` in the `doom-world` project and `shared/protocol.h` in
  `baymaxhead`. Keep their field order, packing, IDs, magic value, and channel
  in sync.

HEAD source links:

- [HEAD main.cpp](/Users/gaurav/Documents/PlatformIO/Projects/baymaxhead/src/main.cpp)
- [HEAD config.h](/Users/gaurav/Documents/PlatformIO/Projects/baymaxhead/include/config.h)
- [HEAD pins.h](/Users/gaurav/Documents/PlatformIO/Projects/baymaxhead/include/pins.h)
- [HEAD protocol.h](/Users/gaurav/Documents/PlatformIO/Projects/baymaxhead/shared/protocol.h)
- [HAND main.cpp](/Users/gaurav/Documents/PlatformIO/Projects/doom-world/hand/src/main.cpp)
- [HAND protocol.h](/Users/gaurav/Documents/PlatformIO/Projects/doom-world/shared/protocol.h)

## What changed for diagnosis

The HAND and HEAD now share an acknowledged auxiliary-command and status
protocol. Both builds compile successfully. The updated HAND is flashed; the
updated HEAD still needs to be flashed for end-to-end acknowledgements.

- HAND prints sensor distance, validity, packet age, and a one-second HEAD
  status snapshot with current sensor, laser, LED, camera power, buzzer, servo
  targets, joystick mode/position, control age, and radio counters.
- Every HAND auxiliary command (camera power, head LED, laser, both servo
  targets, buzzer) includes a sequence number. HEAD returns `AuxAckPacket`
  type 6 after accepting the command. HAND retries the same idempotent command
  at most twice if no acknowledgement arrives; it reports a timeout if still
  unconfirmed.
- HEAD also broadcasts `HeadStatusPacket` type 5 once per second, including
  sensor echo duration, software output states, servo targets, joystick/control
  age, radio channel/counters, and the last accepted auxiliary command. HAND
  marks status stale/offline if no fresh status packet arrives.
- `ESP_OK` from the HAND send call means only that its local send request was
  accepted. A matching `HEAD ACK` or `HEAD STATUS CONFIRMED` is evidence that
  HEAD firmware accepted that sequence. It is not electrical/mechanical
  readback of the laser, camera power rail, LED, buzzer, or servo position.
- Camera HTTP photo/record/flash/gallery actions print their result and
  elapsed time on HAND serial; camera Wi-Fi connection transitions are logged.
- The HAND build setting routes `Serial` to UART0 (`ARDUINO_USB_CDC_ON_BOOT=0`)
  for the connected USB-to-UART adapter at 115200 baud.
- HAND now prints `JOYSTICK INPUT mode=... raw=(x,y) value=(x,y)` when a
  joystick axis changes by at least 5 normalized units or the HEAD/BODY mode
  changes. `raw` is the 12-bit ADC reading (center is expected near 2048);
  `value` is the normalized, dead-zoned value sent in control packets. It also
  prints `JOYSTICK STATE` with raw and normalized values once per second, so a
  stationary or disconnected input remains visible after opening the monitor.
  `JOYSTICK TX ERROR` means HAND's local ESP-NOW send call failed. These logs
  identify the input/send stage; HEAD receiver status still requires the
  updated HEAD firmware.
- The earlier capture showed `raw=(0,0)`, but the latest user test shows both
  joystick ADC channels responding: raw readings reached `(4095,4095)` and
  changed to approximately `(2828,2803)` during movement. The joystick axis
  wiring and HAND ADC reads are therefore working. The normalized values
  (`(99,-99)` at full-scale and around `(38,-36)` at the intermediate sample)
  follow the current fixed-center conversion; validate center readings and
  direction mapping before tuning/calibrating it.
- The latest capture also showed `SENSOR RX: no HEAD SensorPacket received yet`
  and `HEAD STATUS: WAITING`. WAITING is expected until the updated HEAD
  firmware broadcasts type 5, but the missing legacy sensor packets mean the
  HAND did not receive HEAD telemetry during this capture either. Check that
  HEAD is powered and running, and inspect its serial log and ESP-NOW channel.
  `cameraWiFi=DISCONNECTED` is the HAND's camera Wi-Fi state; by itself it
  does not diagnose the ESP-NOW link.

The diagnostic build compiled and uploaded successfully to the connected
ESP32-S3. These are the sensor lines captured from that device:

```text
SENSOR RX: seq=6245 distance=-1.0cm valid=no age=88ms
SENSOR RX: seq=6254 distance=-1.0cm valid=no age=6ms
SENSOR RX: seq=6262 distance=-1.0cm valid=no age=45ms
```

The exact sequence values change over time. The important result is that the
sequence advances and packet age stays low: the HAND **is receiving regular
HEAD sensor packets**. The distance is `-1.0` and `valid=no`, so the HEAD
firmware is currently reporting the ultrasonic measurement as invalid. The
DIS screen therefore correctly displays `--`; this is not evidence of a
missing HAND screen implementation.

With the new HAND firmware but before flashing the updated HEAD, its serial
monitor reports `HEAD STATUS: WAITING`: the installed HEAD firmware does not
broadcast the new type 5 status packet. WAITING alone does not show whether
the radio link is down. If HAND also reports `SENSOR RX: no HEAD SensorPacket
received yet`, as in the latest test, it is not receiving even the older
telemetry; check HEAD power, operation, and ESP-NOW channel before diagnosing
the ultrasonic sensor. Flash the updated HEAD before expecting ACKs or status
details.

The actual `doom-world/hand/src/main.cpp` already had other uncommitted gallery
preview changes when this diagnostic work began. Those gallery changes were
preserved and are unrelated to this HEAD investigation.

## What the HEAD currently implements

### Ultrasonic distance

The HEAD source reads an HC-SR04 and broadcasts `SensorPacket` telemetry:

- TRIG: GPIO 27; ECHO: GPIO 35.
- ECHO must be reduced to a safe 3.3 V logic level before it reaches the ESP32.
- The HEAD attempts a reading and sends telemetry every 100 ms (10 Hz).
- `pulseIn()` waits for an echo for at most 25,000 microseconds.
- Readings outside 2–400 cm, or readings with no echo, are sent as invalid
  (`distanceCm = -1`, `valid = 0`).
- The HAND accepts packets with the exact shared packet size, and displays a
  distance only when a recent packet is valid and within 2–400 cm.

Next checks on the HEAD:

1. Confirm the sensor has a common ground with the HEAD and is powered correctly.
2. Check the TRIG/ECHO wiring against the pin map and confirm the ECHO divider
   is installed and functioning. Never feed a 5 V ECHO signal directly into an
   ESP32 GPIO.
3. Confirm an obstacle is within the sensor's usable range and not angled or
   too soft/small to reflect ultrasound.
4. Temporarily log the raw echo duration and validity on the HEAD serial output.
   A zero duration means `pulseIn()` timed out; a nonzero duration rejected by
   the code is likely out of the configured distance range.
5. Check whether the HEAD's `RADIO ch=...` diagnostic remains on channel 1 and
   whether its `txFail` counter stays unchanged.

Do not change the HAND's valid-distance test just to hide `--`: the current
HEAD is explicitly reporting invalid measurements. Fix the sensor signal or
the HEAD's measurement/validity logic first, then verify that the HAND logs a
positive distance with `valid=yes`.

### Laser

The laser path is implemented in both source projects:

1. On the HAND Action screen, clicking the selected laser item toggles its
   local state and sends a packed `V3AuxCommand` with `target = AUX_LASER` (3),
   `type = 4`, `magic = 0xB808`, and `state = 0|1`.
2. The HEAD checks that packet format and target, stores `laserEnabled`, and
   applies the value to GPIO 23 in its main loop.
3. HAND and HEAD both use ESP-NOW on channel 1.

The HAND and HEAD protocol definitions now include `AuxAckPacket` (type 6) and
`HeadStatusPacket` (type 5). All three copies of the shared protocol must stay
compatible in field order and packing. The HAND resends the same auxiliary
command up to two times while waiting for its matching sequence acknowledgement.
HEAD status can confirm the last accepted sequence if the individual ACK packet
was lost.

When the user clicks the laser, compare the HAND's `AUX TX`/`HEAD ACK` lines
with the HEAD's `AUX ACK` and `RADIO` lines:

- `rx` should increase for received packets.
- `aux` should increase for accepted current-format auxiliary packets.
- `rejected` should not increase for valid HAND auxiliary packets.
- These counters are cumulative; the matching sequence in `HEAD ACK` or
  `HEAD STATUS CONFIRMED` identifies the specific accepted command.

If the HAND logs repeated `AUX RETRY` then `HEAD ACK TIMEOUT`, check that the
updated HEAD firmware is running, both devices are on channel 1, and the HEAD
`rx`/`aux` counters increase. If the matching ACK says ON/OFF but the physical
laser does not change, the protocol path is working; inspect GPIO 23, transistor
polarity, laser supply, and common ground. The ACK confirms software state,
not optical output.

There is a separate HEAD failsafe inconsistency worth resolving: config defines
`LASER_FAILSAFE_MS = 1500` and the callback updates `lastLaserCmdMs`, but the
loop does not use either to turn the laser off. The current source comment
mentions a heartbeat failsafe, but no periodic HAND laser heartbeat is sent.
Decide the intended safety behavior before implementing it; a timeout that
automatically turns the laser off must not be described as a heartbeat unless
the HAND really sends periodic laser refresh commands.

### Servo control from Action

**Servo control is implemented in firmware; the `SERVO C` display is not just
placeholder artwork.** Actual movement still depends on servos being connected
to the configured pins and an adequate external supply.

- HEAD pan servo signal: GPIO 18.
- HEAD tilt servo signal: GPIO 19.
- The HEAD attaches both outputs using 50 Hz LEDC PWM and initializes them to
  the 90-degree center target.
- The HAND's Action screen has a `SERVO C` selection. Clicking it enters servo
  editing, forces HAND mode to HEAD, and sends current targets for `AUX_SERVO_1`
  and `AUX_SERVO_2`.
- Rotate the encoder to change the selected target in 5-degree increments.
  The HAND sends the corresponding auxiliary servo target each step.
- Click while editing to select S-2 after S-1; click again after S-2 to exit
  servo editing.
- The HEAD maps servo target IDs 4 and 5 to pan/tilt, clamps their values to
  the configured angle limits, and moves toward targets in its loop.
- The joystick also moves the HEAD servos when HAND mode is HEAD. This is a
  separate, continuously transmitted joystick-control path.

The HEAD uses an external 5 V servo rail with a shared ground; do not power
servos from an ESP32 GPIO. Check the actual servo mechanical limits and linkage
before relying on the configured 10–170 degree pan and 20–160 degree tilt
ranges. If Action servo clicks/encoder changes fail, inspect the HEAD `aux`
counter and verify target IDs plus the `value` byte. A software build alone
does not prove the servos or power wiring work on the robot.

### Microphone / voice control

**Microphone input is not implemented.** The AI/J.A.R.V.I.S. page draws a
microphone icon and a `NOT ACTIVE` label, but the HAND source has no microphone
input pin, audio driver, recording logic, speech recognition, or AI connection.
The Action screen's buzzer control is an output command; it is not a
microphone.

No microphone feature should be added to the HEAD unless that is the intended
architecture. A real microphone feature would require choosing the board that
owns the microphone, wiring/interface, audio capture implementation, and a
defined processing or transmission path. Until those exist, treat the
microphone icon as static UI artwork.

## Build and verify after HEAD changes

The new ACK/status packet definitions must match in these files:

- `baymaxhead/shared/protocol.h`
- `doom-world/shared/protocol.h`
- This repository's `shared/protocol.h`

Do not change one side alone: old HAND/HEAD builds will not understand the new
type 5 status or type 6 ACK packets. Compile-time checks enforce a 10-byte
`AuxAckPacket` and a 66-byte `HeadStatusPacket` in each shared header. Rebuild
and flash both boards for an end-to-end test.

Build the HEAD separately from its project directory:

```sh
cd ~/Documents/PlatformIO/Projects/baymaxhead
pio run
```

Build and flash the HAND from its project directory too:

```sh
cd ~/Documents/PlatformIO/Projects/doom-world/hand
pio run
pio run -t upload
pio device monitor -b 115200
```

Flash HEAD using its own connected port and platform configuration. Do not
assume the HAND USB port belongs to HEAD. After both boards run the new
firmware, HAND serial should show lines similar to:

```text
HEAD ACK MATCH target=LASER state=1 value=0 seq=...
HEAD STATUS online ch=1 ... sensor=INVALID -1.0cm echo=0us
  laser=ON ... imu=READY ... pan=90 tilt=90 ... aux=... rejected=0
```

When no matching ACK arrives, HAND logs `AUX RETRY` and eventually
`HEAD ACK TIMEOUT`. `HEAD STATUS: OFFLINE` means the HEAD status packet has not
arrived recently. On HEAD serial, `AUX ACK` reports its attempted ACK
transmission, `RADIO` prints channel/counters, and `HEAD STATE` prints the
local sensor/output/servo/control/IMU state plus ultrasonic echo duration.

`HEAD ACK MATCH` means the HEAD firmware accepted and applied the software
state for that command. It cannot prove electrical output, light emission,
servo shaft movement, actual camera power, or audible sound because there are
no physical feedback sensors for those. Compare the software status with
careful hardware inspection. The HAND USB-to-UART adapter was previously
detected as `/dev/cu.usbmodem5AE60622391` on macOS; port names may change after
reconnection.

Joystick control packets are sent continuously rather than individually
acknowledged. The periodic status reports latest joystick mode/axes, control
age, and cumulative receive count so a stale or missing control stream is
visible without flooding serial output.

For a HAND-side joystick test, open its serial port at 115200 baud, move one
axis at a time, and look for `JOYSTICK INPUT`. The latest user test confirmed
the ADC values change with movement, so there is no current reason to change
the X/Y pin constants. A centered axis should read near 2048 raw and 0
normalized (allowing for calibration offset). Confirm the center and each
direction before adjusting calibration/scaling in the HAND source. If raw
values stop changing in a future test, check joystick power (3.3 V), shared
ground, and signal connections: X/GPIO 1 and Y/GPIO 2. Do not feed 5 V into the
ESP32 ADC pins. If the input changes but HEAD does not respond, first restore
HEAD telemetry; then flash the updated HEAD and check its `control` receive
count, mode, axes, and control age.

## Short status summary

| Feature | Firmware status | Current observation |
|---|---|---|
| DIS ultrasonic distance | HEAD telemetry and HAND display are implemented | HAND receives HEAD packets, but latest readings are `-1.0 cm`, `valid=no` |
| DIS compass heading | Not wired end-to-end | Static graphic; heading is not in `SensorPacket` |
| Action laser | HAND command and HEAD GPIO output are implemented | Needs a click plus comparison of HAND send log and HEAD counters/output |
| Action servo control | HAND target editing and HEAD PWM/target handling are implemented | Verify servos, power, grounds, and mechanical range on hardware |
| Microphone/voice | Not implemented | AI-screen icon/art only; Action buzzer is unrelated |
| Laser failsafe | Config/comment exist, actual timeout is not in loop | Decide and implement intended safe behavior deliberately |
