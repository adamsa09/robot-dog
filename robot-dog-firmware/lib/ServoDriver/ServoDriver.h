#pragma once
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "config.h"   // SERVO_US_MIN, SERVO_US_MAX, I2C_SDA, I2C_SCL

extern Adafruit_PWMServoDriver pca9685;

void servoDriverBegin();
void writeServo(uint8_t channel, float physDeg);
