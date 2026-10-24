# DOOM WORLD V4 — PlatformIO Folder & File Arrangement

Place all projects inside a single root folder named **`doom-world/`**. Because `hand/`, `head/`, and `body/` all use the `-I../shared` build flag in their `platformio.ini`, keeping `shared/` at the same level as the project folders ensures every board automatically finds `protocol.h`.

---

## 1. Complete Directory Tree

```text
doom-world/
│
├── README.md                                  # Main system documentation
├── WIRING.md                                  # Hardware GPIO pinout reference
├── DOOM_CONTROLLER_CONTEXT.md                 # System integration & protocol contract
│
├── shared/                                    # Shared across HAND, HEAD, and BODY
│   └── protocol.h                             # ESP-NOW packets, magic (0xB808), AUX IDs
│
├── hand/                                      # PROJECT 1: DOOM Hand Controller (ESP32-S3)
│   ├── platformio.ini                         # Board: esp32-s3-devkitc-1
│   ├── include/
│   │   ├── pins.h                             # Hand TFT, Encoder, Joystick, Buzzer, LEDs
│   │   ├── config.h                           # Wi-Fi AP credentials, URLs, timeouts
│   │   ├── theme.h                            # Orange/Black RGB565 color palette
│   │   ├── Org_01.h                           # Custom font (Boot, Header, Action, DIS)
│   │   ├── FreeMonoBold9pt7b.h                # Custom font (Iron Man Idle Screen)
│   │   ├── Animation.h                        # 15-frame boot animation array
│   │   ├── image_IRON_bits.h                  # Screen 1: Boot Iron Man bitmap
│   │   ├── image__251eed8d212d7f77bedb8a31e6d70126_1_bits.h  # Screen 8: Idle bitmap
│   │   ├── image_Auth_bits.h                  # Screen 5: Action screen icon
│   │   ├── image_camera_bits.h                # Screen 2 & 3: Camera icon
│   │   ├── image_check_big_bits.h             # Screen 2, 4, 5: Link checkmark icon
│   │   ├── image_DolphinCommon_bits.h         # Screen 4: DIS screen dolphin art
│   │   ├── image_DolphinSuccess_bits.h        # Screen 6: Navigation screen dolphin art
│   │   ├── image_EviWaiting1_bits.h           # Screen 2: HEAD mode icon
│   │   ├── image_im_03909002_bits.h           # Screen 7: J.A.R.V.I.S. AI graphic
│   │   ├── image_location_bits.h              # Screen 2 & 6: Navigation pin icon
│   │   ├── image_MHz_bits.h                   # Screen 4: MHz label bitmap
│   │   ├── image_microphone_recording_bits.h  # Screen 7: Microphone icon
│   │   ├── image_music_radio_broadcast_bits.h # Screen 2: DIS radio tower icon
│   │   ├── image_music_record_1_bits.h        # Screen 2: Decorative dot icon
│   │   ├── image_music_record_bits.h          # Screen 2: Decorative dot icon
│   │   ├── image_mute_hover_bits.h            # Screen 7: Mute speaker icon
│   │   ├── image_Ok_btn_pressed_bits.h        # Screen 2: BODY mode icon
│   │   ├── image_Pin_attention_dpad_bits.h    # Screen 4: Compass center D-pad icon
│   │   ├── image_Power_hvr_bits.h             # Screen 3: Camera power toggle icon
│   │   ├── image_RFIDBigChip_bits.h           # Screen 2: M-Active / SD Gallery chip icon
│   │   ├── image_satellite_dish_bits.h        # Screen 6: Satellite dish icon
│   │   ├── image_Scanning_short_bits.h        # Screen 4: Radar scanning art
│   │   ├── image_SDQuestion_bits.h            # Screen 2: SD card icon
│   │   ├── image_vol_ac_text_bits.h           # Screen 5: Volume button icon
│   │   └── image_wifi_75_bits.h               # Screen 3: Live Wi-Fi signal icon
│   └── src/
│       └── main.cpp                           # Hand V4 main firmware
│
├── head/                                      # PROJECT 2: Head Node — ESP32 (A)
│   ├── platformio.ini                         # Board: esp32dev (includes -I../shared)
│   ├── include/
│   │   ├── pins.h                             # Servos (18,19), HC-SR04 (27,35), MPU (21,22),
│   │   │                                      # Laser (23), LED (4), CamPwr (16), Buzzer (17)
│   │   └── config.h                           # Servo limits, 10cm alarm threshold, timeouts
│   └── src/
│       └── main.cpp                           # Head ESP32 (A) main firmware
│
├── cam/                                       # PROJECT 3: Camera Node — ESP32-CAM
│   ├── platformio.ini                         # Board: esp32cam (huge_app.csv + PSRAM)
│   ├── include/
│   │   ├── pins.h                             # OV2640 camera pins + Flash LED (GPIO 4)
│   │   └── config.h                           # SoftAP (DOOM_CAM, Ch 1) & 60KB JPEG limits
│   └── src/
│       └── main.cpp                           # ESP32-CAM HTTP + SD Gallery server
│
└── body/                                      # PROJECT 4: Body Node — ESP32 (B) (Next Step)
    ├── platformio.ini                         # Board: esp32dev (includes -I../shared)
    ├── include/
    │   ├── pins.h                             # Left (25,26,27) & Right (13,32,33) motors
    │   └── config.h                           # PWM frequency, deadzone, failsafe timeout
    └── src/
        └── main.cpp                           # Differential drive motor firmware
```

---

## 2. Step-by-Step Setup Sequence for the `head/` Folder

To set up the **Head ESP32 (A)** project right now in VS Code / PlatformIO:

1. **Ensure `shared/protocol.h` exists** one level above `head/` (`doom-world/shared/protocol.h`).
2. **Create the `head/` folders:**
   * `doom-world/head/`
   * `doom-world/head/include/`
   * `doom-world/head/src/`
3. **Place the 4 Head files in this exact order:**
   * `doom-world/head/platformio.ini` *(from "Head ESP32 (A) PlatformIO Config")*
   * `doom-world/head/include/pins.h` *(from "Head ESP32 (A) Pin Mapping")*
   * `doom-world/head/include/config.h` *(from "Head ESP32 (A) Configuration")*
   * `doom-world/head/src/main.cpp` *(from "Head ESP32 (A) Main Firmware")*
4. **Open a terminal inside `doom-world/head/` and run:**
   ```bash
   pio run
   pio run -t upload
   ```