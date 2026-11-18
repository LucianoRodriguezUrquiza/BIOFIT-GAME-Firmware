// Host-only replay of the EXACT production phase-evidence helper.
// No claim of biomechanical contact detection or Android performance.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "../../Firmware/Tobillo_Izquierdo/BIOFIT_TOBILLO_IZQUIERDO_BASE/BiofitPhaseEvidence.h"

static void sample(BiofitPhaseEvidence &p, uint32_t t, float up, float gyro=4.f, float activity=.01f)
{
  p.Observe(t,up,gyro,activity,.981f,.98f);
}
int main()
{
  BiofitPhaseEvidence p;
  assert(p.ready);
  assert(p.OnUp(1000)); // first physical rise after calibration
  for(uint32_t t=1100;t<=1500;t+=10)sample(p,t,+.18f);
  assert(!p.ready);
  assert(!p.OnUp(1550)); // braking with positive acceleration alone is NOT a cycle
  sample(p,1650,-.18f,40.f,.2f);
  for(uint32_t t=1700;t<=1980;t+=10)sample(p,t,+.02f,45.f,.1f);
  assert(!p.ready); // gyro still moving, no rearm
  for(uint32_t t=2000;t<2080;t+=10)sample(p,t,.01f);
  assert(!p.ready); // 80ms dwell not completed
  sample(p,2080,.01f);
  assert(p.ready);
  assert(p.OnUp(2130)); // genuinely separate new rise
  assert(!p.ready);
  // Discontinuous stability must restart dwell, not accumulate two short pauses.
  sample(p,2500,-.2f,90.f,.3f);
  for(uint32_t t=2510;t<=2560;t+=10)sample(p,t,.01f);
  sample(p,2570,.6f,80.f,.5f);
  for(uint32_t t=2580;t<=2630;t+=10)sample(p,t,.01f);
  assert(!p.ready);
  for(uint32_t t=2640;t<=2670;t+=10)sample(p,t,.01f);
  assert(p.ready);
  assert(p.OnUp(2690));
  // Elapsed guards uint32_t rollover.
  p.Reset();
  assert(p.OnUp(0xFFFFFF00u));
  sample(p,0xFFFFFF50u,-.2f);
  for(uint32_t k=0;k<=500;k+=10)sample(p,(uint32_t)(0xFFFFFF00u+k), k<340 ? -.2f:.0f);
  assert(p.ready); // 160ms stable after min interval across wrap
  assert(p.OnUp(260u));
  assert(p.qualified==5 && p.unqualified==1);
  std::puts("PASS: real fw228 helper -- negative-phase gate, contiguous quiet dwell, gyro rejection, rollover, one rise/cycle");
}
