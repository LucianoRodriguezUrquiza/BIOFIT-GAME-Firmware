#include <cassert>
#include <cstring>
#include <cstdio>
#include <cstdint>
static uint64_t fingerprint=14695981039346656037ull;
static uint32_t nEvents[5]={};
static uint32_t tick=0,rng=0x71ab2241u,nSamples=0;
void hash32(uint32_t v){for(unsigned i=0;i<4;i++){fingerprint^=uint8_t(v>>(i*8));fingerprint*=1099511628211ull;}}
void hashFloat(float f){uint32_t v;std::memcpy(&v,&f,sizeof(v));hash32(v);}
void pushEvent(uint8_t t,uint32_t ms,float gyro,float up,uint8_t flags){
 if(t<5)++nEvents[t];
 hash32(t);hash32(ms);hashFloat(gyro);hashFloat(up);hash32(flags);
}
float noise(){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return (int32_t(rng%2001u)-1000)*.00001f;}
void sample(float ax,float ay,float az,float wx,float wy,float wz,uint32_t step){
 tick+=step;ImuSample s={};s.ms=tick;s.us=tick*1000u;s.a[0]=ax;s.a[1]=ay;s.a[2]=az;s.w[0]=wx;s.w[1]=wy;s.w[2]=wz;
 processSample(s,step*.001f);++nSamples;
 hashFloat(fUp);hashFloat(fActivity);hashFloat(shortV);hashFloat(gravityNorm);
 for(unsigned j=0;j<3;j++){hashFloat(verticalUnit[j]);hashFloat(gyroBias[j]);}
 hash32(det.upArmed);hash32(det.downSeen);hash32(det.upEvidence);hash32(det.lastUpMs);hash32(det.ups);
 hash32(det.moving);hash32(det.moves);hash32(rearmCount);hash32(blockedUpSamples);hash32(upBlockReason);
 hash32(autoRestCount);hash32(restRejectedWindows);hash32(liftCycle.onsets);hash32(liftCycle.rearms);hash32(liftCycle.faults);
}
int main(){
 cfg=defaults();tick=0xffffe000u;beginCalibration(tick);
 for(int i=0;i<300;i++)sample(0,0,1,0,0,0,5);
 assert(calibrated);
 for(int n=0;n<160;n++){
   for(int i=0;i<160;i++){
     float z=1.f;
     if(i<22)z+=.27f*i/21.f;
     else if(i<36)z+=.27f;
     else if(i<86)z-=.24f;
     else if(i<109)z+=.17f;
     float w=(i<109)?9.f*sinf(i*.15f):0.f;
     uint32_t dt=(i==75&&n%13==0)?25u:(i%3==0?4u:5u);
     sample(noise(),noise(),z+noise(),noise(),w+noise(),noise(),dt);
   }
   if(n%11==0){for(int i=0;i<500;i++)sample(0,0,1,0,0,0,5);}
   if(n==70){cfg.upOnG=.14f;cfg.upGapMs=350;}
 }
 assert(nEvents[3]>10);
 std::printf("samples=%u MOV_ON=%u MOV_OFF=%u UP=%u LIFT=%u fingerprint=%016llx\n",
 nSamples,nEvents[1],nEvents[2],nEvents[3],nEvents[4],(unsigned long long)fingerprint);
}
