#include <cassert>
#include <cstdio>
#include <vector>
#include "../../Firmware/Tobillo_Izquierdo/BIOFIT_TOBILLO_IZQUIERDO_BASE/BiofitLiftCycle.h"
struct Harness {
 BiofitLiftCycle c;uint32_t t=0;std::vector<uint32_t> events;
 void step(float a,int duration,float gyro=0) {for(int n=0;n<duration;n+=5){t+=5;if(c.Observe(t,a,gyro,fabsf(a),fabsf(a)))events.push_back(t);}}
 void rest(int ms=200) {step(0,ms);}
 void cycle(int pause=100) {step(.3f,150);step(-.3f,150);rest(pause);step(-.3f,150);step(.3f,150);rest(pause);}
};
int main(){
 Harness h;h.rest();for(int n=0;n<30;n++){auto before=h.events.size();h.cycle();assert(h.events.size()==before+1);}assert(h.events.size()==30);
 std::printf("PASS 30 balanced rise/return cycles at 800ms: 30 onsets, no return events; first latency=%ums\n",h.events[0]-200);
 Harness held;held.rest();held.step(.3f,150);held.step(-.3f,150);held.rest(1000);auto before=held.events.size();held.step(-.3f,150);held.step(.3f,150);held.rest();assert(held.events.size()==before&&before==1);
 Harness down;down.rest();down.step(-.35f,100);down.rest(400);down.step(.35f,100);down.rest();assert(down.events.empty());
 Harness spike;spike.rest();spike.step(.6f,5);spike.rest(300);assert(spike.events.empty());
 Harness quiet;quiet.rest(10000);assert(quiet.events.empty());
 Harness slow;slow.rest();slow.step(.10f,150);slow.step(-.10f,150);slow.rest();assert(slow.events.empty());
 Harness gap;gap.rest();gap.step(.3f,10);gap.t+=100;gap.step(.3f,5);assert(gap.c.faults==1&&gap.events.empty());gap.rest();gap.cycle();assert(gap.events.size()==1);
 Harness wrap;wrap.t=0xffffff00u;wrap.rest();wrap.cycle();assert(wrap.events.size()==1);
 Harness bad;bad.rest();bad.c.Observe(bad.t+5,NAN,0,0,0);assert(bad.c.state==BiofitLiftCycle::Unreferenced&&bad.c.faults==1);
 // No exponential leak: coasting downward may look quiet but retains veto.
 Harness coast;coast.rest();coast.step(-.3f,150);coast.rest(1000);assert(coast.c.state==BiofitLiftCycle::Returning&&coast.c.velocity<-.4f);coast.step(.3f,150);assert(coast.events.empty());
 // Gyro residual blocks old v228 quiet gate; dynamic return uses signed history.
 Harness active;active.rest();for(int n=0;n<5;n++){active.step(.3f,150,30);active.step(-.3f,300,30);active.step(.3f,150,30);active.step(0,200,30);}assert(active.events.size()==5);
 Harness impact;impact.rest();impact.step(.3f,150);impact.step(-.3f,300);impact.step(6.f,10);impact.step(.3f,140);impact.rest(100);assert(impact.events.size()==1&&impact.c.faults==0);
 std::puts("PASS held pause, down/brake, spikes, weak motion, quiet, data gap, rollover, invalid, gyro-active return");
}
