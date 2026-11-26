#include <cassert>
#include <iostream>
struct Event{uint8_t type;uint32_t ms;float up;};
std::vector<Event> events;
void pushEvent(uint8_t t,uint32_t ms,float,float up,uint8_t){events.push_back({t,ms,up});}
uint32_t nowMs=0;
void sample(float z,float wy=0){nowMs+=5;ImuSample s={};s.a[2]=z;s.w[1]=wy;s.ms=nowMs;s.us=nowMs*1000;processSample(s,.005f);}
void rest(int n){for(int i=0;i<n;i++)sample(1);}
int count(uint8_t t){int n=0;for(auto e:events)if(e.type==t)++n;return n;}
void cycle(){for(int i=0;i<30;i++)sample(1.3f);for(int i=0;i<60;i++)sample(.7f);for(int i=0;i<30;i++)sample(1.3f);rest(40);}
void boot(){cfg=defaults();diagnosticControl=BiofitDiag::Control{};diagnosticTrace.Freeze();diagnosticVerticalValid=false;nowMs=0;events.clear();autoRestEnabled=true;beginCalibration(0);rest(200);assert(calibrated);rest(450);}
void profile(uint16_t mask,uint16_t token){assert(diagnosticControl.Request(mask,token,nowMs));rest(100);assert(diagnosticControl.active==mask&&diagnosticControl.state==0);}
int main(){
  boot();profile(1,1);autoRestEnabled=false;
  for(int j=0;j<100;j++){for(int i=0;i<60;i++)sample(1.f+.22f*i/59.f);for(int i=0;i<30;i++)sample(.75f);rest(100);}
  assert(count(3)>80);for(auto e:events)if(e.type==3)assert(e.up>=.11999f);
  std::cout<<"PASS threshold-aligned real processSample, UP="<<count(3)<<"\n";
  boot();profile(8,1);autoRestEnabled=false;for(int j=0;j<30;j++)cycle();assert(count(4)==0&&count(3)>0&&liftCycle.onsets>0);
  std::cout<<"PASS isolated LIFT transmission; legacy retained\n";
  boot();profile(4,1);autoRestEnabled=false;for(int i=0;i<12000;i++)sample(1,.5f);
  assert(fabsf(fUp)<.02f);assert(count(3)==0);
  std::cout<<"PASS induced low-energy gyro bias corrected, no UP\n";
  boot();profile(2,1);autoRestEnabled=false;
  det.upArmed=false;det.downSeen=false;det.lastUpMs=nowMs-1000;fUp=0;shortV=0;fGyro[1]=15;
  diagnosticTrace.Start();for(int i=0;i<100;i++)sample(1.01f,15);diagnosticTrace.Freeze();
  bool extra=false;for(uint16_t i=0;i<diagnosticTrace.count;++i){BiofitDiag::Sample v;assert(diagnosticTrace.Read(diagnosticTrace.generation,i,v));extra|=(v.events&32)!=0;}
  assert(det.upArmed&&extra);assert(count(3)==0);
  std::cout<<"PASS supervised rearm on bounded valley without forced impulse\n";
  boot();profile(15,1);diagnosticTrace.Start();autoRestEnabled=false;
  for(int j=0;j<100;j++){cycle();}
  diagnosticTrace.Freeze();
  BiofitDiag::Sample t;assert(diagnosticTrace.count==1024);assert(diagnosticTrace.Read(diagnosticTrace.generation,1023,t)&&t.mask==15&&t.ms==nowMs);
  assert(count(3)>0&&count(4)==0);std::cout<<"PASS combined prolonged replay and circular trace\n";
  rest(100);profile(0,2);events.clear();for(int j=0;j<30;j++)cycle();assert(count(4)>0&&count(3)>0);
  std::cout<<"PASS return to default, both channels available\n";
}
