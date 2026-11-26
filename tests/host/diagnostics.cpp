#include "../../Firmware/Tobillo_Izquierdo/BIOFIT_TOBILLO_IZQUIERDO_BASE/BiofitDiagnostic.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
using namespace BiofitDiag;
int main(){
  uint16_t a[3]={};
  assert(ParseFields("15,65535",a,2)&&a[0]==15&&a[1]==65535);
  for(const char* bad:{"","1,","1,2,","-1,2","1,65536","1,2x","1,2,3","1,,2","1, 2"})assert(!ParseFields(bad,a,2));
  Control c;assert(c.active==0);assert(!c.Request(16,1,0));assert(!c.Request(1,0,0));
  assert(c.Request(15,1,100));assert(c.Request(15,1,110));assert(!c.Request(0,1,110));assert(!c.Request(0,2,110));
  assert(!c.Observe(99,true));assert(c.state==1); // older in-flight sample
  assert(!c.Observe(200,true));assert(!c.Observe(449,true)); // gap does not count as dwell
  assert(!c.Observe(450,false));
  for(uint32_t t=500;t<750;t+=10)assert(!c.Observe(t,true));
  assert(c.Observe(750,true)&&c.active==15&&c.state==0);
  assert(c.Request(0,2,800));assert(!c.Observe(10800,true)&&c.state==2&&c.active==15);
  assert(c.Request(0,3,0xffffff00u));for(uint32_t i=0;i<250;i+=10)assert(!c.Observe(0xffffff10u+i,true));
  assert(c.Observe(10u,true)&&c.active==0);
  assert(EmissionFloor(.09f,0)==.09f&&EmissionFloor(.09f,1)==.12f&&EmissionFloor(.2f,1)==.2f);
  Rearm r;assert(!r.Observe(100,false,100,260,0,.04f,15,0));
  assert(!r.Observe(300,false,300,260,0,.04f,15,0));
  assert(!r.Observe(350,false,350,260,.2f,.04f,15,0));
  assert(!r.Observe(400,false,400,260,0,.04f,15,0));
  for(uint32_t t=410;t<460;t+=10)assert(!r.Observe(t,false,t,260,0,.04f,15,0));
  assert(r.Observe(460,false,460,260,0,.04f,15,0));
  assert(!r.Observe(461,false,461,260,0,.04f,15,.2f));
  float u[3]={.2f,0,sqrtf(.96f)},v[3]={0,0,1};
  float before=u[0];for(int i=0;i<400;i++)assert(CorrectVertical(u,v,1,1,0,0,.005f));
  assert(fabsf(u[0])<before*.2f&&fabsf(u[2]-1)<.001f);
  float old=u[0];assert(!CorrectVertical(u,v,1.2f,1,0,0,.005f)&&u[0]==old);
  assert(!CorrectVertical(u,v,1,1,80,0,.005f));assert(!CorrectVertical(u,v,1,1,0,0,NAN));
  Trace<4> tr;Sample s;assert(!tr.Read(0,0,s));tr.Start();auto gen=tr.generation;
  for(unsigned i=0;i<10;i++){s.ms=i;s.up=-123;s.raw[5]=-300;tr.Push(s);}
  tr.Start();assert(tr.generation==gen&&tr.count==4);assert(!tr.Read(gen,0,s));tr.Freeze();
  assert(tr.Read(gen,0,s)&&s.ms==6);assert(tr.Read(gen,3,s)&&s.ms==9);assert(!tr.Read(gen,4,s));
  uint8_t packet[20];assert(Encode(s,1,gen,3,0,packet)&&packet[0]==0xD3&&packet[8]==9&&packet[12]==0x85&&packet[13]==0xff);
  assert(Encode(s,2,gen,3,1,packet)&&packet[18]==0xd4&&packet[19]==0xfe);
  assert(!Encode(s,1,gen,3,3,packet));tr.Start();assert(!tr.Read(gen,0,s));
  std::puts("PASS: profile grammar, idempotency, transition timeout/rollover, supervised rearm, bounded correction, ring and packet encoding");
}
