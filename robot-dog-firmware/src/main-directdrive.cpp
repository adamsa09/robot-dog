#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <ps5Controller.h>
#include <Arduino.h>
#include "config.h"
#include "Calibration.h"
#include "IK.h"
#include "Gait.h"
#include "ServoDriver.h"


/* TODO
- Make deadzone for sticks, not set their zeroes. More accurate.
- Adapt joint channel identification for when 3 DOF is implemented
*/

// ----- CONSTANTS -----
const int LStickXZero = 0;
const int LStickYZero = -2;

const int RStickXZero = 1;
const int RStickYZero = -3;


bool isHome;
unsigned long lastHomeRead;
const unsigned long homeReadDelay = 5000;
bool pendingHome;
bool pendingStand;

const float HOME_THETA2 = 0;
const float HOME_THETA3 = 130;

const float STAND_X = 50;
const float STAND_Z = 200;

float speedFactor = 1.0f;


FootKey footKeyPoints[NUM_LEGS][NUM_KEYS];


// ----- STATE VARIABLES -----
float gaitPhase = 0.0f;


unsigned long currentTimeMs;
unsigned long lastTimeMs;


// ----- HELPERS -----

void onConnect() {
  Serial.println("[i] Connected to controller");
}

void onDisconnect() {
  Serial.println("[i] Disconnected from controller");
}

float stickToSpeed(int stickValue) {
  return map(stickValue, LStickYZero, 127, 0, 3);
}

void homeAllLegs() {
  for (int i = 0; i < NUM_LEGS; i++) {
    float theta2, theta3;

    int hipChannel = i * 2;
    int kneeChannel = i * 2 + 1;

    writeServo(hipChannel, HOME_THETA2);
    writeServo(kneeChannel, HOME_THETA3);
  }
}


void standAllLegs() {
  for (int i = 0; i < NUM_LEGS; i++) {
    float theta2, theta3;
    IK(STAND_X, STAND_Z, &theta2, &theta3, legCalibrations[i]);

    int hipChannel = i * 2;
    int kneeChannel = i * 2 + 1;

    writeServo(hipChannel, theta2);
    writeServo(kneeChannel, theta3);
  }
}

void interpretController() {
  speedFactor = stickToSpeed(ps5.LStickY());

  if (ps5.Triangle() == 1 && millis() - lastHomeRead >= homeReadDelay) {
    if (isHome == false) {
      pendingHome = true;
    } else {
      pendingStand = true;
    }
    lastHomeRead = millis();
  }
}

// ----- SETUP / LOOP -----

void setup() {
  for (int i = 0; i < NUM_LEGS; i++) {
    footKeyPoints[i][0] = { 0.00f, 50.0f, 200.0f };
    footKeyPoints[i][1] = { 0.25f, 50.0f, 155.0f };
    footKeyPoints[i][2] = { 0.50f, -50.0f, 155.0f };
    footKeyPoints[i][3] = { 0.75f, -50.0f, 200.0f };
  }

  Serial.begin(115200);
  
  servoDriverBegin();

  lastTimeMs = millis();
  Serial.println("[*] Ready.");

  ps5.attach(interpretController);
  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisconnect);
  ps5.begin("14:3A:9A:2A:90:35");


  Serial.println("[i] Ready");

  while (ps5.isConnected() == false) {
    Serial.println("[i] Waiting for PS5 Controller");
    delay(300);
  }
}


void loop() {
  if (pendingHome == true) {
    pendingHome = false;
    homeAllLegs();
    isHome = true;
  }

  if (pendingStand == true) {
    pendingStand = false;
    standAllLegs();
    isHome = false;
  }

  if (!isHome) {
    currentTimeMs = millis();
    unsigned long dt = currentTimeMs - lastTimeMs;
    lastTimeMs = currentTimeMs;

    float phaseDelta = ((float)dt / GAIT_PERIOD_MS) * speedFactor;
    gaitPhase = wrapPhase(gaitPhase + phaseDelta);

    for (int i = 0; i < NUM_LEGS; i++) {
      float legPhase = wrapPhase(gaitPhase + legPhaseOffset[i]);

      Coord foot = getFootPositionForPhase(i, legPhase, footKeyPoints);

      float theta2, theta3;
      IK(foot.x, foot.z, &theta2, &theta3, legCalibrations[i]);

      int hipChannel = i * 2;
      int kneeChannel = i * 2 + 1;

      writeServo(hipChannel, theta2);
      writeServo(kneeChannel, theta3);
    }
  }
}
