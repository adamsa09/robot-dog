#define USE_PCA9685_SERVO_EXPANDER
#include <ServoEasing.hpp>
#include <Wire.h>
#include <math.h>

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

struct Angles {
  float theta2;
  float theta3;
};

int physToApi(int physDeg) {
  return map(physDeg, 0, 270, 0, 180);
}

void ensureAttached(uint8_t ch) {
  if (attached[ch]) return;

  Serial.print("[i] Attaching servo ");
  Serial.println(ch);
  servos[ch]->attach(ch, 0, SERVO_US_MIN, SERVO_US_MAX);
  servos[ch]->setEasingType(EASE_CUBIC_IN_OUT);
  servos[ch]->setSpeed(EASE_SPEED_DPS);
  attached[ch] = true;

  Serial.print("[i] Ch ");
  Serial.print(ch);
  Serial.println(" initialized.");
}

Angles IK(float x, float z) {
  float L12 = sqrtf(x * x + z * z);

  float theta12 = acos((L1 * L1 + L2 * L2 - L12 * L12) / (2 * L1 * L2));

  float theta3 = KNEE_SERVO_CALIB_SLOPE * (180 - degrees(theta12)) + KNEE_SERVO_CALIB_INTERCEPT;

  float alpha1 = atan(x / z);

  float gamma1 = asin((sin(theta12) * L2) / L12);

  float theta2 = HIP_SERVO_CALIB_SLOPE * (90 - (degrees(gamma1) + degrees(alpha1))) + HIP_SERVO_CALIB_INTERCEPT;

  return { theta2, theta3 };
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println("[i] Initializing PCA9685 Connection");
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("[i] Attaching servos to ServoEasing");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    servos[i] = new ServoEasing(0x40);
    attached[i] = false;
  }

  if (servos[0]->InitializeAndCheckI2CConnection(&Serial)) {
    Serial.println("PCA9685 not found — check wiring/address!");
    while (true) {}
  }

  Serial.println("[*] Ready.");
}

void loop() {
  ensureAttached(0);
  ensureAttached(1);

  float positions[4][2] = {
    {46.10, 48.68},
    {28.83, 81.20},
    {56.67, 81.20},
    {67.95, 48.68}
  };

  int coordinates[4][2] = {
    {50, 200},
    {50, 155},
    {-50, 155},
    {-50, 200}
  };

  for (int i = 0; i < 4; i++) {
    Angles angles = IK(coordinates[i][0], coordinates[i][1]);
    
    int hipApi = physToApi((int)angles.theta2);
    int kneeApi = physToApi((int)angles.theta3);
    
    ServoEasing::ServoEasingArray[0]->setEaseTo(hipApi);
    ServoEasing::ServoEasingArray[1]->setEaseTo(kneeApi);
    setEaseToForAllServosSynchronizeAndStartInterrupt();
    
    delay(EASE_SPEED_DPS * 3); // Wait for movement to complete
  }
}
