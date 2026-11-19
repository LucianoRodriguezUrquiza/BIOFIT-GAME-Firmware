#include <cassert>
#include <iostream>
struct Event { int type; uint32_t ms; float up; };
std::vector<Event> events;
void pushEvent(uint8_t t,uint32_t ms,float,float up,uint8_t) { events.push_back({t,ms,up}); }
uint32_t nowMs=0;
void sample(float ax,float ay,float az,float wx=0,float wy=0,float wz=0) {
 nowMs+=5; ImuSample s={};s.a[0]=ax;s.a[1]=ay;s.a[2]=az;s.w[0]=wx;s.w[1]=wy;s.w[2]=wz;s.ms=nowMs;s.us=nowMs*1000;
 processSample(s,.005f);
}
void stationary(int samples) { for(int i=0;i<samples;i++)sample(0,0,1); }
int ups() { int n=0;for(auto e:events)if(e.type==3)n++;return n; }
void boot() {
 cfg=defaults();nowMs=0;events.clear();autoRestCount=0;autoRestEnabled=true;
 beginCalibration(0);stationary(200);assert(calibrated);
}
void gesture() { for(int i=0;i<20;i++)sample(0,0,1.35);for(int i=0;i<25;i++)sample(0,0,.75);stationary(155); }
int main(int argc,char**) {
 std::cout.setf(std::ios::unitbuf);
 bool baseline=argc>1;
 // Initial calibration must reject an observably changing pose.
 cfg=defaults();nowMs=0;beginCalibration(0);
 for(int i=1;i<=170;i++){float a=8.f*(i*.005f)*DEG_TO_RAD_F;sample(-sinf(a),0,cosf(a),0,8,0);}
 if(baseline)assert(calibrated);else assert(!calibrated);
 stationary(200);assert(calibrated);
 std::cout<<"PASS moving CAL window / stable completion\n";
 boot();for(int i=0;i<4;i++){int before=ups();gesture();assert(ups()==before+1);}
 std::cout<<"PASS first four gestures after CAL\n";
 boot();uint32_t before=autoRestCount;
 for(int i=1;i<=600;i++){float a=2.f*(i*.005f)*DEG_TO_RAD_F;sample(-sinf(a),0,cosf(a),0,2,0);}
 std::cout<<"AUTO REST during 2 deg/s tilt: "<<autoRestCount-before<<"\n";
 if(baseline)assert(autoRestCount>before);else assert(autoRestCount==before);
 boot();autoRestEnabled=false;det.upArmed=false;det.lastUpMs=nowMs-1000;det.moving=false;fUp=.033f;fActivity=.033f;
 for(int i=0;i<30;i++)sample(0,0,1.033f);
 std::cout<<"Rearmed in accepted QUIET at +0.033g: "<<det.upArmed<<"\n";
 if(baseline)assert(!det.upArmed);else assert(det.upArmed);
 boot();stationary(2000);assert(ups()==0);assert(autoRestCount>=4);
 // 120 s including quiet, sustained gestures, pauses and resumption.
 for(int block=0;block<3;block++){for(int i=0;i<20;i++)gesture();stationary(2000);for(int i=0;i<10;i++)gesture();}
 assert(ups()==90);std::cout<<"PASS 120s replay: 90 gestures, repeated rest, no additional UP\n";
 // 250ms input cannot imply four game jumps/s; record detector rate only.
 for(int interval:{50,100,200,400}){boot();autoRestEnabled=false;for(int g=0;g<10;g++){for(int i=0;i<interval;i++)sample(0,0,i<15?1.35f:i<30?.75f:1.f);}assert(ups()<=10);std::cout<<"period_ms="<<interval*5<<" UP="<<ups()<<"\n";}
 // Bias drift during motion: do not claim it is eliminated. Test recovery at rest.
 boot();autoRestEnabled=false;
 for(int i=0;i<12000;i++)sample(0,0,1,0,.5f,0);
 assert(fabsf(fUp)>.05f); // expected residual drift in the uncorrected gyro integrator
 autoRestEnabled=true;stationary(700);assert(fabsf(fUp)<.01f);assert(!det.moving);
 std::cout<<"PASS induced gyro drift observed and stable-rest recovery\n";
 // Independent NVS-like parameter snapshots, preserving constraints.
 boot();cfg.upOnG=.2f;cfg.upGapMs=500;assert(validConfig(cfg));gesture();assert(ups()==1);
 cfg.upOnG=NAN;assert(!validConfig(cfg));
 // Rollover with actual uint32 timestamps used by processSample.
 boot();nowMs=0xfffff000u;det.lastUpMs=nowMs-1000;stationary(1000);int n=ups();gesture();assert(ups()==n+1);
 std::cout<<"PASS rollover, no duplicate rest impulses\n";
 if(!baseline) {
  boot();stationary(50);autoRestEnabled=false;
  for(int n=0;n<30;n++) {
   for(int i=0;i<30;i++)sample(0,0,1.3f);
   for(int i=0;i<60;i++)sample(0,0,.7f);
   for(int i=0;i<30;i++)sample(0,0,1.3f);
   stationary(40);
  }
  int count=0;for(auto e:events)if(e.type==4)++count;
  std::cout<<"Actual .ino + filter: LIFT_ONSET="<<count<<" legacy UP="<<ups()<<" /30 synthetic cycles\n";
  assert(count==30);
 }

}
