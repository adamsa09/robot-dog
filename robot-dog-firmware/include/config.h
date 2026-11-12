#pragma once
#include <stdint.h>

// PCA9685 Config
constexpr int I2C_SDA           = 21;
constexpr int I2C_SCL           = 22;
constexpr int SERVO_US_MIN      = 537;   // SERVO_MIN=110 counts @ 50 Hz → 537 µs
constexpr int SERVO_US_MAX      = 2930;  // SERVO_MAX=600 counts
constexpr int EASE_SPEED_DPS    = 60;    // degrees/sec (0–180 API scale)
constexpr int NUM_CHANNELS      = 16;

// Geometry constants -- belt driven
constexpr float belt_L2              = 115.707f;
constexpr float belt_L3              = 115.000f;

// Geometry constants -- Parallelogram driven
constexpr float L1 = 35;
constexpr float L2 = 42.178;
constexpr float L3 = 49.497;
constexpr float L4 = 35;
constexpr float L5 = 35;
constexpr float L6 = 42.178;
constexpr float L7 = 115.718;
constexpr float L8 = 115.718;
constexpr float L9 = 35;
constexpr float L10 = 108.55;

// If +1: servo rotates CCW on input increase; If -1: Servo rotates CW on input increase
// TODO Add sigma values into jointcalibration.
constexpr int sigmas2 = 1; 
constexpr int sigmas3 = 1;


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

// TODO: Should I use coord struct in here instead of x and z variables?
struct FootKey {
  float phase;
  float x;
  float z;
};


struct Coord {
  float x;
  float z;
};

