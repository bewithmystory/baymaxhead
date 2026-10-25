#ifndef HEAD_CONFIG_H
#define HEAD_CONFIG_H

#include <Arduino.h>

// =====================================================
// HEAD ESP32 (A) Operational Configuration
// =====================================================

// Logic level that turns the ESP32-CAM power transistor ON
#define CAM_PWR_ON_LEVEL          HIGH
#define CAM_PWR_OFF_LEVEL         LOW

// Servo Pulse & Angle Limits (50 Hz standard hobby servos)
#define SERVO_FREQ_HZ             50
#define SERVO_MIN_US              500
#define SERVO_MAX_US              2400
#define SERVO_PAN_MIN_DEG         10
#define SERVO_PAN_MAX_DEG         170
#define SERVO_TILT_MIN_DEG        20
#define SERVO_TILT_MAX_DEG        160
#define SERVO_CENTER_DEG          90

// Maximum servo speed in degrees per 20ms step when driven by joystick
#define JOY_SERVO_MAX_STEP_DEG    2.5f

// Ultrasonic Telemetry & Obstacle Alarm Thresholds
#define TELEMETRY_INTERVAL_MS     100   // 10 Hz SensorPacket broadcast to HAND
#define OBSTACLE_ALARM_CM         10.0f // Beep alarm when valid distance < 10 cm
#define ULTRA_TIMEOUT_US          25000 // ~400 cm max echo wait

// Failsafe Timeouts
#define CONTROL_TIMEOUT_MS        600   // Stop joystick servo motion if link drops
#define LASER_FAILSAFE_MS         1500  // Auto-shutoff laser if heartbeat stops

// MPU6500 I2C Address
#define MPU6500_ADDR              0x68

#endif // HEAD_CONFIG_H