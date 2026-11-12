#include "IK.h"
#include <Arduino.h>
#include <math.h>
#include "config.h"

void IK(float x, float z, float* theta2, float* theta3, const LegCalibration& cal) {
  float belt_L23 = sqrtf(x * x + z * z);
  float theta23 = acosf((belt_L2 * belt_L2 + belt_L3 * belt_L3 - belt_L23 * belt_L23) / (2.0f * belt_L2 * belt_L3));

  *theta3 = cal.knee.slope * (180.0f - degrees(theta23)) + cal.knee.intercept;

  float alpha1 = atanf(x / z);
  float gamma1 = asinf((sinf(theta23) * belt_L3) / belt_L23);

  *theta2 = cal.hip.slope * (90.0f - (degrees(gamma1) + degrees(alpha1))) + cal.hip.intercept;
}

static float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

bool IKParallelogram(float x, float z, float servo2_x, float servo2_z, float* theta2, float* theta3, const LegCalibration& cal) {
  float delta_x = x - servo2_x;
  float delta_z = z - servo2_z;

  float r_squared = delta_x*delta_x + delta_z*delta_z;

  float r_min = fabsf(L8-L10);
  float r_max = L8 + L10;

  if (r_squared < r_min*r_min || r_squared > r_max*r_max) {
    return false;
  }

  float c = (r_squared - L8*L8 - L10*L10) / (2.0f * L8 * L10);
  c = fmaxf(-1.0f, fminf(1.0f, c)); // clamp

  float psi = -acosf(c);

  float phi_8 = atan2f(delta_z, delta_x) - atan2f(L10 * sinf(psi), L8 + L10 * cosf(psi));
  float phi_1 = phi_8 + psi - (float)M_PI_2;

  float d1 = wrap180(degrees(phi_1) - 90.0f);
  float d2 = wrap180(degrees(phi_8) + 90.0f);


  float t2 = (135.0f + sigmas2 * d1);
  float t3 = (135.0f + sigmas3 * d2);

  if (t2 < 0.0f || t2 > 270.0f || t3 < 0.0f || t3 > 270.0f) return false;

  *theta2 = t2;
  *theta3 = t3;
  return true;
}