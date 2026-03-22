#include "ServoDriver.h"

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40, Wire);

void servoDriverBegin() {
  Wire.begin(I2C_SDA, I2C_SCL);
  pca9685.begin();
  pca9685.setOscillatorFrequency(25000000);
  pca9685.setPWMFreq(50);
  delay(10);
}

void writeServo(uint8_t channel, float physDeg) {
  physDeg = constrain(physDeg, 0.0f, 270.0f);
  uint16_t us = (uint16_t)(SERVO_US_MIN + (physDeg / 270.0f) * (SERVO_US_MAX - SERVO_US_MIN));
  pca9685.writeMicroseconds(channel, us);
}
