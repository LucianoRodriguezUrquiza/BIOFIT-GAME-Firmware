#pragma once
#include <stdint.h>
#include <math.h>
#include <stddef.h>

// Experimental RAM-only options. Zero preserves v229 control and event format.
namespace BiofitDiag {
enum : uint8_t { AlignThreshold=1, SupervisedRearm=2, AdaptiveGravity=4, IsolateLiftTx=8, All=15 };
constexpr uint16_t Version=230;
constexpr uint32_t SettleMs=250, RequestTimeoutMs=10000;
inline bool ValidMask(uint32_t mask) { return mask<=All; }
inline float EmissionFloor(float configured,uint8_t mask) {
  return (mask&AlignThreshold)?fmaxf(configured,.120f):configured;
}
// Strict decimal grammar: reject empty, sign, trailing separators, overflow and extra fields.
inline bool ParseFields(const char* s,uint16_t* out,size_t n) {
  for(size_t i=0;i<n;++i) {
    if(*s<'0'||*s>'9') return false;
    uint32_t v=0;
    do { v=v*10u+uint32_t(*s-'0'); if(v>65535u)return false; ++s; } while(*s>='0'&&*s<='9');
    out[i]=uint16_t(v);
    if(i+1<n){if(*s!=',')return false;++s;}
    else if(*s!='\0')return false;
  }
  return n!=0;
}
// All accesses serialized by caller. Request acknowledgement is not application.
struct Control {
  uint8_t active=0,target=0,state=0; // 0 applied, 1 pending, 2 expired
  uint16_t token=0;
  uint32_t requested=0,quietSince=0;
  bool quietCounting=false;
  bool Request(uint16_t mask,uint16_t id,uint32_t now) {
    if(!ValidMask(mask)||id==0)return false;
    // Duplicate delivery is idempotent; a reused token cannot change target.
    if(token==id)return target==mask;
    if(state==1)return false;
    target=uint8_t(mask);token=id;requested=now;quietCounting=false;
    state=active==target?0:1;return true;
  }
  bool Observe(uint32_t now,bool quiet) {
    if(state!=1)return false;
    if(uint32_t(now-requested)>=RequestTimeoutMs){state=2;quietCounting=false;return false;}
    if(!quiet){quietCounting=false;return false;}
    if(!quietCounting){quietCounting=true;quietSince=now;}
    if(uint32_t(now-quietSince)<SettleMs)return false;
    active=target;state=0;quietCounting=false;return true;
  }
};
struct Rearm {
  bool counting=false;uint32_t since=0;
  void Reset(){counting=false;since=0;}
  bool Observe(uint32_t now,bool armed,uint32_t sinceUp,uint32_t minGap,
               float up,float activity,float gyro,float velocity) {
    // A finite low-energy valley, never elapsed time alone or a strong brake.
    bool valley=!armed && sinceUp>=minGap && isfinite(up)&&isfinite(activity)&&
      isfinite(gyro)&&isfinite(velocity)&&fabsf(up)<=.025f&&
      activity>=0&&activity<=.055f&&gyro>=0&&gyro<=20.f&&
      velocity>=-.08f&&velocity<=.015f;
    if(!valley){Reset();return false;}
    if(!counting){counting=true;since=now;}
    return uint32_t(now-since)>=60u;
  }
};
// Corrections act only on a parallel vertical vector, not the saved anatomical map
// or original gyro integration. Unit-normalized small low-energy corrections.
inline bool CorrectVertical(float u[3],const float a[3],float norm,float gravity,
                            float gyro,float activity,float dt) {
  if(!isfinite(norm)||!isfinite(gravity)||!isfinite(gyro)||!isfinite(activity)||
     !isfinite(dt)||norm<.80f||norm>1.20f||gravity<.80f||gravity>1.20f||
     fabsf(norm-gravity)>.045f||gyro<0||gyro>12.f||activity<0||activity>.045f||
     dt<=0||dt>.030f)return false;
  float v[3]={a[0]/norm,a[1]/norm,a[2]/norm};
  float dot=u[0]*v[0]+u[1]*v[1]+u[2]*v[2];
  if(!isfinite(dot)||dot<.8660254f||fabsf(norm*dot-gravity)>.065f)return false;
  float k=dt/(.8f+dt);
  float next[3]={u[0]+k*(v[0]-u[0]),u[1]+k*(v[1]-u[1]),u[2]+k*(v[2]-u[2])};
  float n=sqrtf(next[0]*next[0]+next[1]*next[1]+next[2]*next[2]);
  if(!isfinite(n)||n<.5f)return false;
  for(unsigned i=0;i<3;++i)u[i]=next[i]/n;
  return true;
}
struct Sample {
  uint32_t ms=0;
  int16_t up=0,velocity=0,activity=0,gyro=0,raw[6]={},legacyUp=0,adaptiveUp=0;
  uint16_t dtUs=0;
  uint8_t flags=0,block=0,mask=0,events=0;
};
// Bounded RAM trace: no allocation or BLE inside Push; chronological indexes
// are stable only once frozen. All accesses serialized by caller.
template<uint16_t N> struct Trace {
  static_assert(N>0,"trace capacity");
  Sample samples[N]={};
  uint16_t head=0,count=0,generation=0;
  uint8_t state=0; // 0 empty, 1 recording, 2 frozen
  void Start(){if(state==1)return;head=count=0;if(++generation==0)++generation;state=1;}
  void Freeze(){if(state==1)state=2;}
  void Push(const Sample& s){if(state!=1)return;samples[head]=s;head=(head+1u)%N;if(count<N)++count;}
  bool Read(uint16_t gen,uint16_t index,Sample& out)const {
    if(state!=2||gen!=generation||index>=count)return false;
    out=samples[(uint32_t(head)+N-count+index)%N];return true;
  }
};
inline void Put16(uint8_t* p,uint16_t v){p[0]=uint8_t(v);p[1]=uint8_t(v>>8);}
inline void Put32(uint8_t* p,uint32_t v){for(unsigned i=0;i<4;++i)p[i]=uint8_t(v>>(8*i));}
inline bool Encode(const Sample& s,uint8_t id,uint16_t gen,uint16_t index,uint8_t page,uint8_t out[20]) {
  if(page>2||id<1||id>4)return false;
  for(unsigned i=0;i<20;++i)out[i]=0;
  out[0]=0xD3;out[1]=id;out[2]=page;out[3]=1;Put16(out+4,gen);Put16(out+6,index);
  if(page==0){Put32(out+8,s.ms);Put16(out+12,s.up);Put16(out+14,s.velocity);Put16(out+16,s.activity);Put16(out+18,s.gyro);}
  if(page==1)for(unsigned i=0;i<6;++i)Put16(out+8+2*i,s.raw[i]);
  if(page==2){out[8]=s.flags;out[9]=s.block;out[10]=s.mask;out[11]=s.events;Put16(out+12,s.legacyUp);Put16(out+14,s.adaptiveUp);Put16(out+16,s.dtUs);}
  return true;
}
} // namespace BiofitDiag
