#pragma once

#include "config.h"

void IK(float x, float z, float* theta2, float* theta3, const LegCalibration& cal);
bool IKParallelogram(float x, float z, float servo2_x, float servo2_z, float* theta2, float* theta3, const LegCalibration& cal);