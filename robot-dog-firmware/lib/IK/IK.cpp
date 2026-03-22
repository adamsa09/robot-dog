#include "IK.h"
#include <Arduino.h>
#include <math.h>
#include "config.h"

void IK(float x, float z, float* theta2, float* theta3, const LegCalibration& cal) {
  float L23 = sqrtf(x * x + z * z);
  float theta23 = acosf((L2 * L2 + L3 * L3 - L23 * L23) / (2.0f * L2 * L3));

  *theta3 = cal.knee.slope * (180.0f - degrees(theta23)) + cal.knee.intercept;

  float alpha1 = atanf(x / z);
  float gamma1 = asinf((sinf(theta23) * L3) / L23);

  *theta2 = cal.hip.slope * (90.0f - (degrees(gamma1) + degrees(alpha1))) + cal.hip.intercept;
}
