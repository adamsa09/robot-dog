#pragma once
#include <stdint.h>

// PCA9685 Config
constexpr int I2C_SDA           = 21;
constexpr int I2C_SCL           = 22;
constexpr int SERVO_US_MIN      = 537;   // SERVO_MIN=110 counts @ 50 Hz → 537 µs
constexpr int SERVO_US_MAX      = 2930;  // SERVO_MAX=600 counts
constexpr int EASE_SPEED_DPS    = 60;    // degrees/sec (0–180 API scale)
constexpr int NUM_CHANNELS      = 16;

// Geometry constants
constexpr float L2              = 115.707f;
constexpr float L3              = 115.000f;

constexpr int NUM_LEGS          = 4;

// Leg definitions
constexpr int LEG_FL                = 0; // PCA9685 PINS: 0, 1
constexpr int LEG_FR                = 1; // PCA9685 PINS: 2, 3
constexpr int LEG_RL                = 2; // PCA9685 PINS: 4, 5
constexpr int LEG_RR                = 3; // PCA9685 PINS: 6, 7

constexpr float legPhaseOffset[NUM_LEGS] = {
  0.0f,  // FL
  0.5f,  // FR
  0.5f,  // RL
  0.0f   // RR
};

// Servo calibration structs
struct JointCalibration {
    float slope;
    float intercept;
};

struct LegCalibration {
    JointCalibration hip;
    JointCalibration knee;
};

// Gait config
constexpr int GAIT_PERIOD_MS = 1000;
constexpr int NUM_KEYS = 4;

struct FootKey {
  float phase;
  float x;
  float z;
};


struct Coord {
  float x;
  float z;
};

