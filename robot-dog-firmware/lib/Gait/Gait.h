#pragma once
#include <stdint.h>
#include "config.h"   // for NUM_LEGS, NUM_KEYS

float wrapPhase(float phase);
float linterp(float a, float b, float alpha);
Coord getFootPositionForPhase(int legIndex, float legPhase, FootKey footKeyPoints[][NUM_KEYS]);
