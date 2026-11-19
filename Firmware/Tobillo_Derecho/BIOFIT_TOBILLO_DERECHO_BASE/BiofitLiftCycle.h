#pragma once
#include <stdint.h>
#include <math.h>

// v2.2.9 EXP: causal onset channel, independent of legacy UP/shortV.
// Ordered acceleration lobes define a control cycle, NOT absolute motion/contact.
// Short signed integral supports onset; rearm never requires its drift-sensitive zero.
struct BiofitLiftCycle {
  enum State : uint8_t { Unreferenced, Ready, Rising, Returning };
  enum Reason : uint8_t { Anchoring, Armed, Tracking, Onset, ReturnVeto,
    RearmedQuiet, RearmedValley, DataGap, Horizon, Invalid, LateOrWeak, MinGap };
  static constexpr float StartG=.045f, OnsetG=.12f, EndG=.035f;
  static constexpr float MinVelocity=.020f;
  static constexpr uint32_t OnsetDwellMs=8, OnsetWindowMs=160;
  static constexpr uint32_t InitialQuietMs=160, QuietMs=60, ValleyMs=24;
  static constexpr uint32_t MinCycleMs=400, HorizonMs=2500;
  State state=Unreferenced;
  Reason reason=Anchoring;
  float velocity=0, minVelocity=0, biasG=0, priorG=0;
  float negativeArea=0, brakeArea=0; // g*s: lobe impulse evidence, no absolute velocity
  uint32_t lastMs=0, startMs=0, quietSince=0, evidenceSince=0, lastOnsetMs=0;
  bool haveSample=false, quietCounting=false, evidenceCounting=false;
  bool emitted=false, brakeSeen=false, haveOnset=false;
  uint32_t onsets=0, returnSamples=0, rearms=0, faults=0;

  void Reset() {
    state=Unreferenced;reason=Anchoring;velocity=minVelocity=biasG=priorG=0;negativeArea=brakeArea=0;
    haveSample=quietCounting=evidenceCounting=emitted=brakeSeen=haveOnset=false;
    lastMs=startMs=quietSince=evidenceSince=lastOnsetMs=0;
    // Lifetime diagnostics preserved across auto-rest/calibration.
  }
  bool Dwell(bool condition,uint32_t ms,uint32_t duration) {
    if(!condition) {quietCounting=false;return false;}
    if(!quietCounting) {quietCounting=true;quietSince=ms;}
    return (uint32_t)(ms-quietSince)>=duration;
  }
  void Anchor(Reason why) {
    state=Ready;reason=why;velocity=minVelocity=0;negativeArea=brakeArea=0;
    emitted=brakeSeen=quietCounting=evidenceCounting=false;priorG=0;++rearms;
  }
  bool Observe(uint32_t ms,float upG,float gyro,float activity,float normError) {
    if(!isfinite(upG)||!isfinite(gyro)||!isfinite(activity)||!isfinite(normError)||
       fabsf(upG)>16.f||gyro>900.f||activity<0||gyro<0) {
      Reset();reason=Invalid;++faults;return false;
    }
    uint32_t delta=ms-lastMs;lastMs=ms;
    if(!haveSample) {haveSample=true;priorG=upG;return false;}
    if(delta==0) return false;
    if(delta>30) {Reset();lastMs=ms;haveSample=true;reason=DataGap;++faults;return false;}
    float dt=delta*.001f;
    bool stationary=fabsf(upG)<=.05f && gyro<=20.f && activity<=.06f && fabsf(normError)<=.08f;
    if(state==Unreferenced) {
      reason=Anchoring;
      if(stationary) biasG += .04f*(upG-biasG);
      if(Dwell(stationary,ms,InitialQuietMs)) Anchor(Armed);
      return false;
    }
    // QMI8658 is configured at +/-8g, +/-512dps per axis. Observed norm
    // peaks exceed 6g: do NOT erase cycle history at a normal landing impact.
    float a=fmaxf(-3.f,fminf(3.f,upG-biasG));
    if(state==Ready) {
      if(fabsf(a)<StartG) {
        if(stationary) biasG += .01f*(upG-biasG);
        reason=Armed;return false;
      }
      startMs=ms;velocity=minVelocity=0;priorG=0;negativeArea=brakeArea=0;
      state=Rising;reason=Tracking; // accumulate negative evidence; do not latch on one noisy sample
      quietCounting=evidenceCounting=emitted=brakeSeen=false;
    }
    velocity += .5f*(priorG+a)*9.80665f*dt;priorG=a;
    minVelocity=fminf(minVelocity,velocity);
    if((uint32_t)(ms-startMs)>HorizonMs) {
      Reset();lastMs=ms;haveSample=true;reason=Horizon;++faults;return false;
    }
    if(a<-.06f) negativeArea += -a*dt;
    if(state==Rising && negativeArea>=.004f) {state=Returning;quietCounting=false;}
    if(state==Returning) {
      reason=ReturnVeto;++returnSamples;
      if(negativeArea>=.004f && a>=.08f) brakeArea += a*dt;
      if(brakeArea>=.004f) brakeSeen=true;
      // Consume the positive return/braking lobe before rearming at its valley.
      // Areas reject isolated spikes; no 80ms gyro-quiet or integrated-zero gate.
      bool valley=brakeSeen && a<=EndG; // falling lobe, not a stationary/contact test
      if(Dwell(valley,ms,ValleyMs)) {Anchor(RearmedValley);return false;}
      return false;
    }
    // A stable elevated pause may anchor zero velocity, never "ground contact".
    // A subsequent downward-first motion still enters Returning and cannot emit.
    if(Dwell(stationary && fabsf(velocity)<=.06f,ms,QuietMs)) {
      Anchor(RearmedQuiet);return false;
    }
    if(emitted) {reason=Tracking;return false;}
    if((uint32_t)(ms-startMs)>OnsetWindowMs) {reason=LateOrWeak;return false;}
    bool candidate=a>=OnsetG && activity>=.035f && velocity>=MinVelocity;
    if(!candidate) {evidenceCounting=false;return false;}
    if(!evidenceCounting) {evidenceCounting=true;evidenceSince=ms;}
    if((uint32_t)(ms-evidenceSince)<OnsetDwellMs) return false;
    if(haveOnset && (uint32_t)(ms-lastOnsetMs)<MinCycleMs) {reason=MinGap;return false;}
    emitted=true;haveOnset=true;lastOnsetMs=ms;++onsets;reason=Onset;return true;
  }
};
