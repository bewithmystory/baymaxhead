/*
 * DOOM WORLD - HEAD ESP32 (A) Firmware (V4)
 *
 * Hardware Subsystems (per WIRING.md):
 *   - Pan/Tilt Servos (GPIO 18, 19) using hardware 50 Hz LEDC PWM
 *   - HC-SR04 Ultrasonic Sensor (TRIG 27, ECHO 35) -> 10 Hz ESP-NOW telemetry + <10cm alarm
 *   - MPU6500 IMU (SDA 21, SCL 22) -> Yaw heading integration & health check
 *   - Laser Diode Transistor (GPIO 23) -> AUX_LASER (target=3) + LaserCommand (type=2)
 *   - Head LED Transistor (GPIO 4) -> AUX_HEAD_LED (target=2)
 *   - ESP32-CAM Power Transistor (GPIO 16) -> AUX_CAMERA_POWER (target=1)
 *   - Speaker/Buzzer (GPIO 17) -> AUX_VOLUME (target=6) + obstacle warning tone
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

#include "pins.h"
#include "config.h"
#include "protocol.h"

// Legacy 9-byte V3AuxCommand layout (without value byte) so both V3 and V4 Hand firmware work seamlessly
typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint8_t type;
  uint8_t target;
  uint8_t state;
  uint32_t sequence;
} LegacyV3AuxCommand;

// Spinlock for thread-safe ISR-to-loop communication
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

// ------------ Runtime State ------------
volatile int8_t   joyX = 0;
volatile int8_t   joyY = 0;
volatile uint8_t  joyMode = 0; // 0 = HEAD, 1 = BODY
volatile uint32_t lastControlMs = 0;
volatile uint32_t lastControlSequence = 0;

volatile bool     laserEnabled = false;
volatile uint32_t lastLaserCmdMs = 0;
volatile bool     headLedEnabled = false;
volatile bool     camPowerEnabled = true;
volatile bool     buzzerEnabled = true; // Toggled via AUX_VOLUME (target=6)
volatile uint8_t  buzzerVolume = 50;
volatile uint32_t confirmationBeepUntilMs = 0;
volatile uint8_t  lastAcceptedTarget = 0;
volatile uint8_t  lastAcceptedState = 0;
volatile uint8_t  lastAcceptedValue = 0;
volatile uint32_t lastAcceptedSeq = 0;

volatile float    servoPanTarget = SERVO_CENTER_DEG;
volatile float    servoTiltTarget = SERVO_CENTER_DEG;

float currentPanDeg  = SERVO_CENTER_DEG;
float currentTiltDeg = SERVO_CENTER_DEG;

// Telemetry & MPU6500 state
uint32_t telemetrySeq = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastServoStepMs = 0;
uint32_t lastMpuStepMs = 0;
uint32_t lastDiagnosticsMs = 0;
uint32_t lastControlLogMs = 0;
int8_t lastLoggedJoyX = 0;
int8_t lastLoggedJoyY = 0;
uint8_t lastLoggedJoyMode = 0;
bool controlLogInitialized = false;
float    lastDistanceCm = -1.0f;
bool     lastDistanceValid = false;

uint32_t rxPacketCount = 0;
uint32_t rxControlCount = 0;
uint32_t rxAuxCount = 0;
uint32_t rxLegacyCount = 0;
uint32_t rxRejectedCount = 0;
uint32_t telemetryTxFailureCount = 0;

bool     mpuReady = false;
float    headingYawDeg = 0.0f;
float    gyroZBias = 0.0f;

// Broadcast address to send SensorPacket back to DOOM Hand
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t lastHeadStatusMs = 0;

// =====================================================
// SERVO PWM DRIVER (ESP32 Arduino Core 3.x LEDC API)
// =====================================================

uint32_t angleToDuty(float angleDeg) {
  float clamped = constrain(angleDeg, 0.0f, 180.0f);
  float us = SERVO_MIN_US + (clamped / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  // 50 Hz period = 20,000 us; 14-bit resolution = 16384 ticks
  return (uint32_t)((us / 20000.0f) * 16383.0f);
}

void writeServos(float panDeg, float tiltDeg) {
  ledcWrite(SERVO_PAN_PIN, angleToDuty(panDeg));
  ledcWrite(SERVO_TILT_PIN, angleToDuty(tiltDeg));
}

void initServos() {
  ledcAttach(SERVO_PAN_PIN, SERVO_FREQ_HZ, 14);
  ledcAttach(SERVO_TILT_PIN, SERVO_FREQ_HZ, 14);
  writeServos(SERVO_CENTER_DEG, SERVO_CENTER_DEG);
}

// =====================================================
// AUXILIARY OUTPUTS & BUZZER
// =====================================================

void applyHardwareOutputs() {
  bool laser, led, camPwr;
  portENTER_CRITICAL(&rxMux);
  laser  = laserEnabled;
  led    = headLedEnabled;
  camPwr = camPowerEnabled;
  portEXIT_CRITICAL(&rxMux);

  digitalWrite(LASER_PIN,    laser  ? HIGH : LOW);
  digitalWrite(HEAD_LED_PIN, led    ? HIGH : LOW);
  digitalWrite(CAM_POWER_PIN, camPwr ? CAM_PWR_ON_LEVEL : CAM_PWR_OFF_LEVEL);
}

void updateHeadBuzzer(bool obstacleNear) {
  static uint32_t lastBeepToggleMs = 0;
  static bool beepPhase = false;

  bool volOn;
  uint32_t beepUntilMs;
  portENTER_CRITICAL(&rxMux);
  volOn = buzzerEnabled;
  beepUntilMs = confirmationBeepUntilMs;
  portEXIT_CRITICAL(&rxMux);

  if (!volOn) {
    digitalWrite(HEAD_BUZZER_PIN, LOW);
    return;
  }

  if (obstacleNear) {
    // Rapid warning pulse when ultrasonic detects an obstacle < 10 cm
    if (millis() - lastBeepToggleMs >= 120) {
      lastBeepToggleMs = millis();
      beepPhase = !beepPhase;
      digitalWrite(HEAD_BUZZER_PIN, beepPhase ? HIGH : LOW);
    }
  } else if ((int32_t)(beepUntilMs - millis()) > 0) {
    digitalWrite(HEAD_BUZZER_PIN, HIGH);
  } else {
    digitalWrite(HEAD_BUZZER_PIN, LOW);
  }
}

// =====================================================
// HC-SR04 ULTRASONIC SENSOR
// =====================================================

float readUltrasonicCm(bool &validOut) {
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(ULTRA_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG_PIN, LOW);

  unsigned long durationUs = pulseIn(ULTRA_ECHO_PIN, HIGH, ULTRA_TIMEOUT_US);
  if (durationUs == 0) {
    validOut = false;
    return -1.0f;
  }

  float cm = (durationUs * 0.0343f) * 0.5f;
  if (cm >= 2.0f && cm <= 400.0f) {
    validOut = true;
    return cm;
  }

  validOut = false;
  return -1.0f;
}

// =====================================================
// MPU6500 IMU (I2C Yaw Heading Tracking)
// =====================================================

bool initMPU6500() {
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, 400000);
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Wake up MPU6500
  if (Wire.endTransmission() != 0) {
    return false;
  }

  // Configure Gyroscope to +/- 250 dps (sensitivity = 131 LSB/dps)
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();

  // Calibrate Z-axis gyro bias at startup (keep head still for ~200 ms)
  long sumZ = 0;
  int samples = 50;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(0x47); // GYRO_ZOUT_H
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU6500_ADDR, 2) == 2) {
      int16_t rawZ = (Wire.read() << 8) | Wire.read();
      sumZ += rawZ;
    }
    delay(4);
  }
  gyroZBias = (float)sumZ / (float)samples;
  return true;
}

void updateMPU6500Heading() {
  if (!mpuReady) return;
  uint32_t now = millis();
  if (lastMpuStepMs == 0) {
    lastMpuStepMs = now;
    return;
  }
  float dt = (now - lastMpuStepMs) * 0.001f;
  lastMpuStepMs = now;

  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x47); // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return;

  if (Wire.requestFrom(MPU6500_ADDR, 2) == 2) {
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    float dpsZ = ((float)rawZ - gyroZBias) / 131.0f;
    if (fabsf(dpsZ) > 0.6f) { // Deadband against stationary drift
      headingYawDeg += dpsZ * dt;
      while (headingYawDeg < 0.0f)   headingYawDeg += 360.0f;
      while (headingYawDeg >= 360.0f) headingYawDeg -= 360.0f;
    }
  }
}

// =====================================================
// ESP-NOW COMMUNICATION
// =====================================================

void setupBroadcastPeer() {
  if (esp_now_is_peer_exist(broadcastAddress)) return;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddress, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_err_t result = esp_now_add_peer(&peer);
  Serial.printf("ESP-NOW broadcast peer: %s (%d)\n", esp_err_to_name(result), result);
}

void sendSensorTelemetry(float distanceCm, bool valid) {
  SensorPacket pkt;
  pkt.sequence   = ++telemetrySeq;
  pkt.distanceCm = valid ? distanceCm : -1.0f;
  pkt.valid      = valid ? 1 : 0;
  if (esp_now_send(broadcastAddress, (uint8_t *)&pkt, sizeof(pkt)) != ESP_OK) {
    portENTER_CRITICAL(&rxMux);
    telemetryTxFailureCount++;
    portEXIT_CRITICAL(&rxMux);
  }
}

void sendAuxAck(uint8_t target, uint8_t state, uint8_t value, uint32_t sequence) {
  AuxAckPacket ack;
  ack.magic = DOOM_MAGIC;
  ack.type = 6;
  ack.target = target;
  ack.state = state;
  ack.value = value;
  ack.sequence = sequence;
  esp_now_send(broadcastAddress, (uint8_t *)&ack, sizeof(ack));
}

void sendHeadStatus() {
  HeadStatusPacket status = {};
  status.magic = DOOM_MAGIC;
  status.type = 5;
  status.version = 1;

  portENTER_CRITICAL(&rxMux);
  status.laser = laserEnabled ? 1 : 0;
  status.headLed = headLedEnabled ? 1 : 0;
  status.camPower = camPowerEnabled ? 1 : 0;
  status.buzzer = buzzerEnabled ? 1 : 0;
  status.buzzerVolume = buzzerVolume;
  status.joyMode = joyMode;
  status.joyX = joyX;
  status.joyY = joyY;
  status.lastTarget = lastAcceptedTarget;
  status.lastState = lastAcceptedState;
  status.lastValue = lastAcceptedValue;
  status.lastSeq = lastAcceptedSeq;
  status.panTarget = servoPanTarget;
  status.tiltTarget = servoTiltTarget;
  status.distanceCm = lastDistanceCm;
  status.distanceValid = lastDistanceValid ? 1 : 0;
  status.echoUs = 0;
  status.controlAgeMs = (lastControlMs == 0) ? 0xFFFFFFFFu : (millis() - lastControlMs);
  status.rxPacketCount = rxPacketCount;
  status.rxControlCount = rxControlCount;
  status.rxAuxCount = rxAuxCount;
  status.rxLegacyCount = rxLegacyCount;
  status.rxRejectedCount = rxRejectedCount;
  status.telemetryTxFailureCount = telemetryTxFailureCount;
  status.radioChannel = WIFI_CHANNEL;
  status.reserved = 0;
  portEXIT_CRITICAL(&rxMux);

  esp_now_send(broadcastAddress, (uint8_t *)&status, sizeof(status));
}

void handleAuxTarget(uint8_t target, uint8_t state, uint8_t value, uint32_t sequence) {
  bool outputStateChanged = false;
  portENTER_CRITICAL_ISR(&rxMux);
  switch (target) {
    case AUX_CAMERA_POWER:
      camPowerEnabled = (state != 0);
      break;
    case AUX_HEAD_LED:
      outputStateChanged = headLedEnabled != (state != 0);
      headLedEnabled = (state != 0);
      break;
    case AUX_LASER:
      outputStateChanged = laserEnabled != (state != 0);
      laserEnabled = (state != 0);
      lastLaserCmdMs = millis();
      break;
    case AUX_SERVO_1:
      servoPanTarget = constrain((float)value, (float)SERVO_PAN_MIN_DEG, (float)SERVO_PAN_MAX_DEG);
      break;
    case AUX_SERVO_2:
      servoTiltTarget = constrain((float)value, (float)SERVO_TILT_MIN_DEG, (float)SERVO_TILT_MAX_DEG);
      break;
    case AUX_VOLUME:
      buzzerEnabled = (state != 0 && value > 0);
      buzzerVolume  = value;
      if (!buzzerEnabled) confirmationBeepUntilMs = 0;
      break;
    default:
      break;
  }

  if (outputStateChanged) {
    confirmationBeepUntilMs = millis() + OUTPUT_CONFIRM_BEEP_MS;
  }

  lastAcceptedTarget = target;
  lastAcceptedState = state;
  lastAcceptedValue = value;
  lastAcceptedSeq = sequence;
  portEXIT_CRITICAL_ISR(&rxMux);
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  portENTER_CRITICAL_ISR(&rxMux);
  rxPacketCount++;
  portEXIT_CRITICAL_ISR(&rxMux);

  if (!data || len < 4) {
    portENTER_CRITICAL_ISR(&rxMux);
    rxRejectedCount++;
    portEXIT_CRITICAL_ISR(&rxMux);
    return;
  }

  // 1. Check for the packed V3ControlPacket (type = 3, magic = 0xB808).
  if (len == sizeof(V3ControlPacket)) {
    V3ControlPacket ctrl;
    memcpy(&ctrl, data, sizeof(ctrl));
    if (ctrl.magic == DOOM_MAGIC && ctrl.type == 3) {
      portENTER_CRITICAL_ISR(&rxMux);
      joyMode       = ctrl.mode;
      joyX          = ctrl.x;
      joyY          = ctrl.y;
      lastControlMs = millis();
      lastControlSequence = ctrl.sequence;
      rxControlCount++;
      portEXIT_CRITICAL_ISR(&rxMux);
      return;
    }
  }

  // 2. Check for V4 V3AuxCommand (10 bytes, with value byte)
  if (len == sizeof(V3AuxCommand)) {
    V3AuxCommand aux;
    memcpy(&aux, data, sizeof(aux));
    if (aux.magic == DOOM_MAGIC && aux.type == 4) {
      handleAuxTarget(aux.target, aux.state, aux.value, aux.sequence);
      sendAuxAck(aux.target, aux.state, aux.value, aux.sequence);
      portENTER_CRITICAL_ISR(&rxMux);
      rxAuxCount++;
      portEXIT_CRITICAL_ISR(&rxMux);
      return;
    }
  }

  // 3. Check for Legacy V3AuxCommand (9 bytes, without value byte)
  if (len == sizeof(LegacyV3AuxCommand)) {
    LegacyV3AuxCommand auxLegacy;
    memcpy(&auxLegacy, data, sizeof(auxLegacy));
    if (auxLegacy.magic == DOOM_MAGIC && auxLegacy.type == 4) {
      uint8_t legacyValue = (auxLegacy.state != 0) ? 50 : 0;
      handleAuxTarget(auxLegacy.target, auxLegacy.state, legacyValue, auxLegacy.sequence);
      sendAuxAck(auxLegacy.target, auxLegacy.state, legacyValue, auxLegacy.sequence);
      portENTER_CRITICAL_ISR(&rxMux);
      rxLegacyCount++;
      portEXIT_CRITICAL_ISR(&rxMux);
      return;
    }
  }

  // 4. Check for Legacy LaserCommand (4 bytes, type = 2)
  if (len == sizeof(LaserCommand)) {
    LaserCommand lcmd;
    memcpy(&lcmd, data, sizeof(lcmd));
    if (lcmd.type == 2 && lcmd.magic == DOOM_MAGIC) {
      portENTER_CRITICAL_ISR(&rxMux);
      bool newLaserState = (lcmd.laserOn != 0);
      if (laserEnabled != newLaserState) {
        confirmationBeepUntilMs = millis() + OUTPUT_CONFIRM_BEEP_MS;
      }
      laserEnabled   = newLaserState;
      lastLaserCmdMs = millis();
      rxLegacyCount++;
      portEXIT_CRITICAL_ISR(&rxMux);
      return;
    }
  }

  portENTER_CRITICAL_ISR(&rxMux);
  rxRejectedCount++;
  portEXIT_CRITICAL_ISR(&rxMux);
}

// =====================================================
// SETUP & MAIN LOOP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== DOOM HEAD ESP32 (A) STARTING ===");

  // Initialize Transistor & Buzzer Outputs
  pinMode(LASER_PIN, OUTPUT);
  pinMode(HEAD_LED_PIN, OUTPUT);
  pinMode(CAM_POWER_PIN, OUTPUT);
  pinMode(HEAD_BUZZER_PIN, OUTPUT);

  digitalWrite(LASER_PIN, LOW);
  digitalWrite(HEAD_LED_PIN, LOW);
  digitalWrite(CAM_POWER_PIN, CAM_PWR_ON_LEVEL); // Power on ESP32-CAM by default
  digitalWrite(HEAD_BUZZER_PIN, LOW);

  // Initialize Ultrasonic Pins
  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  pinMode(ULTRA_ECHO_PIN, INPUT);

  // Initialize Servos & MPU6500
  initServos();
  mpuReady = initMPU6500();
  Serial.println(mpuReady ? "MPU6500 ready" : "MPU6500 not detected (continuing)");

  // Configure Wi-Fi Station on fixed Channel 1 for ESP-NOW coexistence
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_err_t channelResult = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.printf("Wi-Fi channel %u: %s (%d)\n", WIFI_CHANNEL,
                esp_err_to_name(channelResult), channelResult);

  Serial.print("HEAD ESP32 (A) MAC: ");
  Serial.println(WiFi.macAddress());
  lastLaserCmdMs = millis();

  esp_err_t nowResult = esp_now_init();
  if (nowResult == ESP_OK) {
    esp_now_register_recv_cb(onDataRecv);
    setupBroadcastPeer();
    Serial.println("ESP-NOW initialized on Channel 1");
  } else {
    Serial.printf("FATAL: ESP-NOW init failed: %s (%d)\n",
                  esp_err_to_name(nowResult), nowResult);
  }
}

void loop() {
  uint32_t now = millis();

  if (now - lastDiagnosticsMs >= 1000) {
    lastDiagnosticsMs = now;
    uint32_t packets, controls, aux, legacy, rejected, txFailures;
    portENTER_CRITICAL(&rxMux);
    packets = rxPacketCount;
    controls = rxControlCount;
    aux = rxAuxCount;
    legacy = rxLegacyCount;
    rejected = rxRejectedCount;
    txFailures = telemetryTxFailureCount;
    portEXIT_CRITICAL(&rxMux);
    uint8_t primaryChannel = 0;
    wifi_second_chan_t secondaryChannel = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primaryChannel, &secondaryChannel);
    Serial.printf("RADIO ch=%u rx=%lu ctrl=%lu aux=%lu legacy=%lu rejected=%lu txFail=%lu\n",
                  primaryChannel, (unsigned long)packets, (unsigned long)controls,
                  (unsigned long)aux, (unsigned long)legacy, (unsigned long)rejected,
                  (unsigned long)txFailures);
  }

  uint8_t controlMode;
  int8_t controlX, controlY;
  uint32_t controlSequence, controlAge;
  bool hasControl;
  portENTER_CRITICAL(&rxMux);
  controlMode = joyMode;
  controlX = joyX;
  controlY = joyY;
  controlSequence = lastControlSequence;
  hasControl = lastControlMs != 0;
  controlAge = hasControl ? (now - lastControlMs) : 0xFFFFFFFFu;
  portEXIT_CRITICAL(&rxMux);

  bool controlChanged = !controlLogInitialized ||
                        controlMode != lastLoggedJoyMode ||
                        controlX != lastLoggedJoyX ||
                        controlY != lastLoggedJoyY;
  if (hasControl && controlChanged && now - lastControlLogMs >= 100) {
    lastControlLogMs = now;
    lastLoggedJoyMode = controlMode;
    lastLoggedJoyX = controlX;
    lastLoggedJoyY = controlY;
    controlLogInitialized = true;
    Serial.printf("HEAD CONTROL RX mode=%u x=%d y=%d seq=%lu age=%lu\n",
                  (unsigned)controlMode, (int)controlX, (int)controlY,
                  (unsigned long)controlSequence, (unsigned long)controlAge);
  }

  // 1. Laser heartbeat failsafe: turn off laser if no refresh within 1500 ms
  if (laserEnabled && (now - lastLaserCmdMs) > LASER_FAILSAFE_MS) {
    laserEnabled = false;
  }
  applyHardwareOutputs();

  // 2. Update Servo Pan/Tilt at 50 Hz (every 20 ms)
  if (now - lastServoStepMs >= 20) {
    lastServoStepMs = now;

    int8_t x, y;
    uint8_t mode;
    uint32_t ctrlAge;
    float panTgt, tiltTgt;

    portENTER_CRITICAL(&rxMux);
    x       = joyX;
    y       = joyY;
    mode    = joyMode;
    ctrlAge = (lastControlMs == 0) ? 999999UL : (now - lastControlMs);
    panTgt  = servoPanTarget;
    tiltTgt = servoTiltTarget;
    portEXIT_CRITICAL(&rxMux);

    // If joystick is in HEAD mode (mode == 0) and link is active, nudge target angles
    if (mode == 0 && ctrlAge < CONTROL_TIMEOUT_MS && (x != 0 || y != 0)) {
      panTgt  += ((float)x / 100.0f) * JOY_SERVO_MAX_STEP_DEG;
      tiltTgt += ((float)y / 100.0f) * JOY_SERVO_MAX_STEP_DEG;
      panTgt   = constrain(panTgt,  (float)SERVO_PAN_MIN_DEG,  (float)SERVO_PAN_MAX_DEG);
      tiltTgt  = constrain(tiltTgt, (float)SERVO_TILT_MIN_DEG, (float)SERVO_TILT_MAX_DEG);

      portENTER_CRITICAL(&rxMux);
      servoPanTarget  = panTgt;
      servoTiltTarget = tiltTgt;
      portEXIT_CRITICAL(&rxMux);

    }

    // Smoothly interpolate toward target angles
    currentPanDeg  += (panTgt  - currentPanDeg)  * 0.35f;
    currentTiltDeg += (tiltTgt - currentTiltDeg) * 0.35f;
    writeServos(currentPanDeg, currentTiltDeg);
  }

  // 3. Update MPU6500 Yaw Heading
  updateMPU6500Heading();

  // 4. Sample Ultrasonic Sensor & Broadcast Telemetry at 10 Hz (every 100 ms)
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    bool valid = false;
    float cm = readUltrasonicCm(valid);
    lastDistanceCm = cm;
    lastDistanceValid = valid;

    sendSensorTelemetry(cm, valid);
  }

  // 5. Trigger Head Buzzer Alarm if valid obstacle is closer than 10 cm
  bool obstacleClose = (lastDistanceValid && lastDistanceCm > 0.0f && lastDistanceCm < OBSTACLE_ALARM_CM);
  updateHeadBuzzer(obstacleClose);

  // 6. Publish periodic HEAD status packet for controller health monitoring
  if (now - lastHeadStatusMs >= 1000) {
    lastHeadStatusMs = now;
    sendHeadStatus();
  }
}