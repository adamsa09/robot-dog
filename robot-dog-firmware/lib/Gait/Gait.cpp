#include "Gait.h"

float wrapPhase(float phase) {
  if (phase >= 1.0f) phase -= 1.0f;
  else if (phase < 0.0f) phase += 1.0f;
  return phase;
}

float linterp(float a, float b, float alpha) {
  return a + alpha * (b - a);
}

Coord getFootPositionForPhase(int legIndex, float legPhase, FootKey footKeyPoints[][NUM_KEYS]) {
  FootKey* keys = &footKeyPoints[legIndex][0];

  int i = 0, j = 0;
  float phaseStart, phaseEnd;

  if (legPhase < keys[0].phase || legPhase >= keys[NUM_KEYS - 1].phase) {
    i = NUM_KEYS - 1;
    j = 0;
    phaseStart = keys[i].phase;
    phaseEnd = keys[j].phase + 1.0f;
  } else {
    for (int k = 0; k < NUM_KEYS - 1; k++) {
      if (legPhase >= keys[k].phase && legPhase < keys[k + 1].phase) {
        i = k;
        j = k + 1;
        phaseStart = keys[i].phase;
        phaseEnd = keys[j].phase;
        break;
      }
    }
  }

  float effectivePhase = legPhase;
  if (phaseEnd > 1.0f && legPhase < keys[0].phase) {
    effectivePhase = legPhase + 1.0f;
  }

  float alpha = (effectivePhase - phaseStart) / (phaseEnd - phaseStart);
  return { linterp(keys[i].x, keys[j].x, alpha), linterp(keys[i].z, keys[j].z, alpha) };
}
