#define USE_PCA9685_SERVO_EXPANDER
#include <ServoEasing.hpp>
#include <Wire.h>
#include <Arduino.h>
#include "config.h"

// ─── One ServoEasing instance per channel, lazily attached ───────────────────
ServoEasing* servos[NUM_CHANNELS];
bool         attached[NUM_CHANNELS];

String inputLine = "";

// ─── Helpers ─────────────────────────────────────────────────────────────────
// ServoEasing is 0–180°; remap physical 0–270° to that scale
int physToApi(int physDeg) {
  return map(physDeg, 0, 270, 0, 180);
}

// Attach and home a channel the first time it's commanded
void ensureAttached(uint8_t ch) {
  if (attached[ch]) return;

  servos[ch]->attach(ch, 0, SERVO_US_MIN, SERVO_US_MAX);
  servos[ch]->setEasingType(EASE_CUBIC_IN_OUT);
  servos[ch]->easeTo(0, EASE_SPEED_DPS);   // blocking home to 0° on init
  attached[ch] = true;

  Serial.print("Ch ");
  Serial.print(ch);
  Serial.println(" initialized.");
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  for (int i = 0; i < NUM_CHANNELS; i++) {
    servos[i]  = new ServoEasing(0x40);
    attached[i] = false;
  }

  // Only need to verify I2C once against any instance
  if (servos[0]->InitializeAndCheckI2CConnection(&Serial)) {
    Serial.println("PCA9685 not found — check wiring/address!");
    while (true);
  }

  Serial.println("Ready. Format: <ch>,<angle>  (e.g. 0,90  or  3,180)");
}

// ─── Serial parsing ──────────────────────────────────────────────────────────
void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      inputLine.trim();
      if (inputLine.length() > 0) {
        int commaIdx = inputLine.indexOf(',');

        if (commaIdx < 0) {
          Serial.println("Format: <ch>,<angle>  e.g. 0,90");
        } else {
          int ch    = inputLine.substring(0, commaIdx).toInt();
          int angle = inputLine.substring(commaIdx + 1).toInt();

          if (ch < 0 || ch >= NUM_CHANNELS) {
            Serial.println("Invalid channel. Use 0-15.");
          } else if (angle < 0 || angle > 270) {
            Serial.println("Invalid angle. Use 0-270.");
          } else {
            ensureAttached((uint8_t)ch);
            servos[ch]->startEaseTo(physToApi(angle), EASE_SPEED_DPS, START_UPDATE_BY_INTERRUPT);
            Serial.print("Ch ");
            Serial.print(ch);
            Serial.print(" → ");
            Serial.print(angle);
            Serial.println("°");
          }
        }
      }
      inputLine = "";

    } else {
      inputLine += c;
      if (inputLine.length() > 12) inputLine = "";  // guard: "15,270" = 6 chars
    }
  }
}

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
  handleSerial();
  // Interrupt handles all active servo updates; loop stays non-blocking
}
