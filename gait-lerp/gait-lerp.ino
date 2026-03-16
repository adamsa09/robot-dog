#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>


// ----- CONSTANTS -----
const float L2 = 115.707f;
const float L3 = 115.000f;


static const int I2C_SDA = 21;
static const int I2C_SCL = 22;
static const int SERVO_US_MIN = 537;
static const int SERVO_US_MAX = 2930;
static const int NUM_CHANNELS = 16;


const int NUM_LEGS = 4;


const int LEG_FL = 0;
const int LEG_FR = 1;
const int LEG_RL = 2;
const int LEG_RR = 3;


const int GAIT_PERIOD_MS = 1000;
const float speedFactor = 1.0f;


// Trot: Diagonals in-sync
float legPhaseOffset[NUM_LEGS] = {
  0.0f,  // FL
  0.5f,  // FR
  0.5f,  // RL
  0.0f   // RR
};


struct FootKey {
  float phase;
  float x;
  float z;
};


struct Coord {
  float x;
  float z;
};


const int NUM_KEYS = 4;
FootKey footKeyPoints[NUM_LEGS][NUM_KEYS];


// RED
static const float HIP_SERVO_CALIB_SLOPE     = 0.7784726794f;
static const float HIP_SERVO_CALIB_INTERCEPT = 7.656682028f;


// BLUE
static const float KNEE_SERVO_CALIB_SLOPE     = 0.8825572f;
static const float KNEE_SERVO_CALIB_INTERCEPT = 1.600947f;


// ----- STATE VARIABLES -----
float gaitPhase = 0.0f;


unsigned long currentTimeMs;
unsigned long lastTimeMs;


// PCA9685 driver instance (address 0x40)
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40, Wire);


// ----- HELPERS -----

// Convert physical servo degrees (0–270°) to microseconds and write to channel.
// Uses writeMicroseconds() so the Adafruit library handles count calculation
// internally via the oscillator frequency you set in setup().
void writeServo(uint8_t channel, float physDeg) {
  physDeg = constrain(physDeg, 0.0f, 270.0f);
  uint16_t us = (uint16_t)(SERVO_US_MIN + (physDeg / 270.0f) * (SERVO_US_MAX - SERVO_US_MIN));
  pca9685.writeMicroseconds(channel, us);
}


// Determine theta2 (hip servo) and theta3 (knee servo) based on coordinates (x, z)
void IK(float x, float z, float* theta2, float* theta3) {
  float L23    = sqrtf(x * x + z * z);
  float theta23 = acosf((L2 * L2 + L3 * L3 - L23 * L23) / (2.0f * L2 * L3));

  *theta3 = KNEE_SERVO_CALIB_SLOPE * (180.0f - degrees(theta23)) + KNEE_SERVO_CALIB_INTERCEPT;

  float alpha1 = atanf(x / z);
  float gamma1 = asinf((sinf(theta23) * L3) / L23);

  *theta2 = HIP_SERVO_CALIB_SLOPE * (90.0f - (degrees(gamma1) + degrees(alpha1))) + HIP_SERVO_CALIB_INTERCEPT;
}


float wrapPhase(float phase) {
  if (phase >= 1.0f) phase -= 1.0f;
  else if (phase < 0.0f) phase += 1.0f;
  return phase;
}


float linterp(float a, float b, float alpha) {
  return a + alpha * (b - a);
}


Coord getFootPositionForPhase(int legIndex, float legPhase) {
  FootKey* keys = &footKeyPoints[legIndex][0];

  int i = 0, j = 0;
  float phaseStart, phaseEnd;

  if (legPhase < keys[0].phase || legPhase >= keys[NUM_KEYS - 1].phase) {
    i = NUM_KEYS - 1;
    j = 0;
    phaseStart = keys[i].phase;
    phaseEnd   = keys[j].phase + 1.0f;
  } else {
    for (int k = 0; k < NUM_KEYS - 1; k++) {
      if (legPhase >= keys[k].phase && legPhase < keys[k + 1].phase) {
        i = k;
        j = k + 1;
        phaseStart = keys[i].phase;
        phaseEnd   = keys[j].phase;
        break;
      }
    }
  }

  float effectivePhase = legPhase;
  if (phaseEnd > 1.0f && legPhase < keys[0].phase) {
    effectivePhase = legPhase + 1.0f;
  }

  float alpha  = (effectivePhase - phaseStart) / (phaseEnd - phaseStart);
  float xInterp = linterp(keys[i].x, keys[j].x, alpha);
  float zInterp = linterp(keys[i].z, keys[j].z, alpha);

  return { xInterp, zInterp };
}


// ----- SETUP / LOOP -----

void setup() {
  for (int i = 0; i < NUM_LEGS; i++) {
    footKeyPoints[i][0] = { 0.00f,  50.0f, 200.0f };
    footKeyPoints[i][1] = { 0.25f,  50.0f, 155.0f };
    footKeyPoints[i][2] = { 0.50f, -50.0f, 155.0f };
    footKeyPoints[i][3] = { 0.75f, -50.0f, 200.0f };
  }

  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("[i] Initializing PCA9685");
  pca9685.begin();

  pca9685.setOscillatorFrequency(27000000);
  pca9685.setPWMFreq(50);  // Standard 50 Hz for analog servos

  delay(10);

  lastTimeMs = millis();
  Serial.println("[*] Ready.");
}


void loop() {
  currentTimeMs = millis();
  unsigned long dt = currentTimeMs - lastTimeMs;
  lastTimeMs = currentTimeMs;

  /*
  const int GAIT_PERIOD_MS = 1000;
  const float speedFactor = 1.0f;
  */

  float phaseDelta = ((float)dt / GAIT_PERIOD_MS) * speedFactor;
  gaitPhase = wrapPhase(gaitPhase + phaseDelta);

  for (int i = 0; i < NUM_LEGS; i++) {
    float legPhase = wrapPhase(gaitPhase + legPhaseOffset[i]);

    Coord foot = getFootPositionForPhase(i, legPhase);

    float theta2, theta3;
    IK(foot.x, foot.z, &theta2, &theta3);

    int hipChannel  = i * 2;
    int kneeChannel = i * 2 + 1;

    writeServo(hipChannel,  theta2);
    writeServo(kneeChannel, theta3);
  }
}
