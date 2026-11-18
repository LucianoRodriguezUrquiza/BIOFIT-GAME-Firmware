#pragma once
#include <stdint.h>
#include <math.h>

// BIOFIT v2.2.8 EXPERIMENTAL — per-ankle evidence, not ground-contact sensing.
// The canonical v2.2.7 UP detector and its outputs remain unchanged.
// Mark an UP as "new-cycle qualified" only after negative vertical phase
// followed by a stable, low-motion interval. A held elevated foot can still
// be stationary: this is deliberately called EVIDENCE, not confirmed stance.
struct BiofitPhaseEvidence
{
  static constexpr uint32_t MinAfterUpMs = 340;
  static constexpr uint32_t QuietDwellMs = 80;
  static constexpr float DownThresholdG = -0.115f;
  static constexpr float QuietVerticalG = 0.065f;
  static constexpr float QuietActivityG = 0.075f;
  static constexpr float QuietGyroDps = 23.0f;
  static constexpr float QuietNormErrorG = 0.095f;

  uint32_t lastUpMs = 0;
  uint32_t quietStartMs = 0;
  bool sawNegativePhase = false;
  bool quietCounting = false;
  bool ready = true; // fresh calibration starts ready for the first rise
  uint32_t qualified = 0, unqualified = 0;

  void Reset()
  {
    lastUpMs=0;
    quietStartMs=0;
    sawNegativePhase=false;
    quietCounting=false;
    ready=true;
  }

  void Observe(uint32_t nowMs, float upG, float gyroDps,
               float activityG, float accelNormG, float gravityNormG)
  {
    if(ready) return;
    if(!isfinite(upG) || !isfinite(gyroDps) || !isfinite(activityG) ||
       !isfinite(accelNormG) || !isfinite(gravityNormG)) return;

    if(upG <= DownThresholdG) sawNegativePhase = true;
    bool hasMinimumTime=(uint32_t)(nowMs-lastUpMs)>=MinAfterUpMs;
    bool quiet=fabsf(upG)<=QuietVerticalG &&
               activityG<=QuietActivityG &&
               gyroDps<=QuietGyroDps &&
               fabsf(accelNormG-gravityNormG)<=QuietNormErrorG;
    if(!hasMinimumTime || !sawNegativePhase || !quiet)
    {
      quietCounting=false;
      return;
    }
    if(!quietCounting)
    {
      quietStartMs=nowMs;
      quietCounting=true;
      return;
    }
    if((uint32_t)(nowMs-quietStartMs)>=QuietDwellMs) ready=true;
  }

  bool OnUp(uint32_t nowMs)
  {
    const bool evidence=ready;
    if(evidence) ++qualified; else ++unqualified;
    ready=false;
    sawNegativePhase=false;
    quietCounting=false;
    lastUpMs=nowMs;
    return evidence;
  }
};
