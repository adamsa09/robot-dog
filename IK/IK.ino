#define USE_PCA9685_SERVO_EXPANDER
#include <ServoEasing.hpp>
#include <Wire.h>
#include <math.h>

const float HOME_X = 31.746;
const float HOME_Z = 60.599;

const float L1 = 115.707f;
const float L2 = 115.000f;

static const int I2C_SDA = 21;
static const int I2C_SCL = 22;
static const int SERVO_US_MIN = 537;
static const int SERVO_US_MAX = 2930;
static const int EASE_SPEED_DPS = 60;
static const int NUM_CHANNELS = 16;


// RED
static const float HIP_SERVO_CALIB_SLOPE = 0.7784726794;
static const float HIP_SERVO_CALIB_INTERCEPT = 7.656682028;

// BLUE
static const float KNEE_SERVO_CALIB_SLOPE = 0.8825572;
static const float KNEE_SERVO_CALIB_INTERCEPT = 1.600947;

ServoEasing* servos[NUM_CHANNELS];
bool attached[NUM_CHANNELS];

String inputLine = "";

int physToApi(int physDeg) {
  return map(physDeg, 0, 270, 0, 180);
}

void ensureAttached(uint8_t ch) {
  if (attached[ch]) return;

  servos[ch]->attach(ch, 0, SERVO_US_MIN, SERVO_US_MAX);
  servos[ch]->setEasingType(EASE_CUBIC_IN_OUT);
  servos[ch]->setSpeed(EASE_SPEED_DPS);
  attached[ch] = true;

  Serial.print("Ch ");
  Serial.print(ch);
  Serial.println(" initialized.");
}

void IK(float x, float z, float* theta2, float* theta3) {
  float L12 = sqrtf(x * x + z * z);

  float theta12 = acos((L1 * L1 + L2 * L2 - L12 * L12) / (2 * L1 * L2));

  *theta3 = KNEE_SERVO_CALIB_SLOPE * (180 - degrees(theta12)) + KNEE_SERVO_CALIB_INTERCEPT;

  float alpha1 = atan(x / z);

  float gamma1 = asin((sin(theta12) * L2) / L12);

  *theta2 = HIP_SERVO_CALIB_SLOPE * (90 - (degrees(gamma1) + degrees(alpha1))) + HIP_SERVO_CALIB_INTERCEPT;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  for (int i = 0; i < NUM_CHANNELS; i++) {
    servos[i] = new ServoEasing(0x40);
    attached[i] = false;
  }

  if (servos[0]->InitializeAndCheckI2CConnection(&Serial)) {
    Serial.println("PCA9685 not found — check wiring/address!");
    while (true) {}
  }

  Serial.println("Ready. Format: x,z");
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    // Too many nested if statements. fix.
    if (c == '\n' || c == '\r') {
      inputLine.trim();
      if (inputLine.length() > 0) {
        int commaIdx = inputLine.indexOf(',');
        if (commaIdx < 0 && inputLine != "home" || inputLine != "zero") {
          Serial.println("Format: x,z");
        } else {
          float theta2, theta3;
          if (inputLine == "home") {
            IK(HOME_X, HOME_Z, &theta2, &theta3);
          } else {
            float x = inputLine.substring(0, commaIdx).toFloat();
            float z = inputLine.substring(commaIdx + 1).toFloat();

            IK(x, z, &theta2, &theta3);
          }

          if (isnan(theta2) || isnan(theta3)) {
            Serial.println("Invalid.");
          } else {
            float hipServo = theta2;
            float kneeServo = theta3;

            ensureAttached(0);
            ensureAttached(1);

            int hipTargetApi = physToApi((int)hipServo);
            int kneeTargetApi = physToApi((int)kneeServo);

            ServoEasing::ServoEasingArray[0]->setEaseTo(hipTargetApi);
            ServoEasing::ServoEasingArray[1]->setEaseTo(kneeTargetApi);

            setEaseToForAllServosSynchronizeAndStartInterrupt();

            Serial.print("Hip: ");
            Serial.print(hipServo);
            Serial.print(" ----- Knee: ");
            Serial.println(kneeServo);
          }
        }
      }
      inputLine = "";
    } else {
      inputLine += c;
      if (inputLine.length() > 12) inputLine = "";
    }
  }
}

void loop() {
  handleSerial();
}
