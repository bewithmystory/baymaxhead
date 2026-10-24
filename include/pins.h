#ifndef HEAD_PINS_H
#define HEAD_PINS_H

// =====================================================
// HEAD ESP32 (A) Pin Mapping (from WIRING.md)
// =====================================================

// Pan & Tilt Servos (5V rail powered, common GND with Head ESP32)
#define SERVO_PAN_PIN     18
#define SERVO_TILT_PIN    19

// HC-SR04 Ultrasonic Sensor (ECHO via 5V -> 3.3V divider)
#define ULTRA_TRIG_PIN    27
#define ULTRA_ECHO_PIN    35

// MPU6500 I2C Bus
#define MPU_SDA_PIN       21
#define MPU_SCL_PIN       22

// Transistor-Switched Outputs
#define LASER_PIN         23
#define HEAD_LED_PIN       4
#define CAM_POWER_PIN     16

// Head Speaker / Buzzer
#define HEAD_BUZZER_PIN   17

#endif // HEAD_PINS_H