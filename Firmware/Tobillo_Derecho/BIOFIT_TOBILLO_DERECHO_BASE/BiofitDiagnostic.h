#pragma once
#include <stdint.h>
#include <math.h>
#include <stddef.h>

// Experimental RAM-only options. Zero preserves v229 control and event format.
namespace BiofitDiag {
enum : uint8_t { AlignThreshold=1, SupervisedRearm=2, AdaptiveGravity=4, IsolateLiftTx=8, All=15 };
constexpr uint16_t Version=231;
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
  uint32_t requested=0,quietSince=0,lastObservation=0;
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
    // A sample acquired just before a command may be processed after it.
    const uint32_t elapsed=uint32_t(now-requested);
    if(elapsed>0x7fffffffu)return false;
    if(elapsed>=RequestTimeoutMs){state=2;quietCounting=false;return false;}
    if(!quiet){quietCounting=false;return false;}
    if(quietCounting&&uint32_t(now-lastObservation)>30u)quietCounting=false;
    lastObservation=now;
    if(!quietCounting){quietCounting=true;quietSince=now;}
    if(uint32_t(now-quietSince)<SettleMs)return false;
    active=target;state=0;quietCounting=false;return true;
  }
};
struct Rearm {
  bool counting=false;uint32_t since=0,lastObservation=0;
  void Reset(){counting=false;since=0;}
  bool Observe(uint32_t now,bool armed,uint32_t sinceUp,uint32_t minGap,
               float up,float activity,float gyro,float velocity) {
    // A finite low-energy valley, never elapsed time alone or a strong brake.
    bool valley=!armed && sinceUp>=minGap && isfinite(up)&&isfinite(activity)&&
      isfinite(gyro)&&isfinite(velocity)&&fabsf(up)<=.025f&&
      activity>=0&&activity<=.055f&&gyro>=0&&gyro<=20.f&&
      velocity>=-.08f&&velocity<=.015f;
    if(!valley){Reset();return false;}
    if(counting&&uint32_t(now-lastObservation)>30u)counting=false;
    lastObservation=now;
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
  uint16_t head=0,count=0,generation=0,capacity=N,externalCapacity=0;
  Sample* external=nullptr; // assigned once before acquisition; caller owns storage
  uint8_t state=0; // 0 empty, 1 recording, 2 frozen
  void Attach(Sample* p,uint16_t n){if(state==0&&p&&n>N){external=p;externalCapacity=n;}}
  Sample* Data(){return capacity>N?external:samples;}
  const Sample* Data()const{return capacity>N?external:samples;}
  bool Start(uint16_t requested=N){
    if(requested==0||(requested>N&&(!external||requested>externalCapacity)))return false;
    if(state==1)return requested==capacity; // retry is idempotent
    capacity=requested;head=count=0;if(++generation>=0x8000u)generation=1;state=1;return true;
  }
  void Freeze(){if(state==1)state=2;}
  void Push(const Sample& s){if(state!=1)return;Data()[head]=s;head=(head+1u)%capacity;if(count<capacity)++count;}
  bool Read(uint16_t gen,uint16_t index,Sample& out)const {
    if(state!=2||gen!=generation||index>=count)return false;
    out=Data()[(uint32_t(head)+capacity-count+index)%capacity];return true;
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
constexpr uint16_t MaxTrace=13452;
constexpr uint8_t MaxBlock=16;
inline uint16_t TraceCapacity(uint16_t seconds){
  return seconds==5?1121:seconds==15?3363:seconds==30?6726:seconds==60?MaxTrace:0;
}
inline uint32_t CrcUpdate(uint32_t crc,const uint8_t* p,size_t n){
  while(n--){crc^=*p++;for(unsigned b=0;b<8;++b)crc=(crc>>1)^(0xedb88320u&uint32_t(0u-(crc&1u)));}
  return crc;
}
inline void EncodeBlockEnd(uint8_t id,uint16_t gen,uint16_t start,uint8_t count,uint32_t crc,uint8_t out[20]){
  for(unsigned i=0;i<20;++i)out[i]=0;
  out[0]=0xD4;out[1]=id;out[2]=count;out[3]=1;Put16(out+4,gen);Put16(out+6,start);Put32(out+8,crc);
}
// Event-triggered bank. History stays live; each failure gets its own immutable
// snapshot. Step() copies at most ONE history sample per call under caller lock.
// Sensor timestamps use signed modular differences (windows << 2^31 ms).
constexpr uint16_t AutoHistory=2048, AutoWindow=1024;
constexpr uint8_t AutoSlots=16;
constexpr uint32_t AutoStorage=uint32_t(AutoHistory)+uint32_t(AutoWindow)*AutoSlots;
struct Snapshot {
  uint16_t id=0,gen=0,count=0;
  uint32_t target=0,received=0,cursor=0,end=0;
  uint8_t state=0,flags=0; // 1 wait post, 2 copy, 3 ready, 4 empty
};
struct AutoBank {
  Sample* memory=nullptr;
  Snapshot slots[AutoSlots]={};
  uint32_t total=0;
  uint16_t historyCount=0,session=0,nextGen=0x8000u;
  uint8_t used=0;
  bool armed=false;
  void Attach(Sample* p){if(!armed&&p)memory=p;}
  bool Pending()const{for(uint8_t i=0;i<used;++i)if(slots[i].state==1||slots[i].state==2)return true;return false;}
  bool Begin(uint16_t token){
    if(!memory||token==0)return false;
    if(session==token)return armed; // duplicate does not erase finished captures
    if(armed||Pending())return false;
    session=token;used=0;total=0;historyCount=0;armed=true;return true;
  }
  void Stop(){armed=false;}
  void Push(const Sample& s){
    if(!armed)return;
    memory[total%AutoHistory]=s;++total;if(historyCount<AutoHistory)++historyCount;
  }
  int Mark(uint16_t id,uint32_t target,uint32_t received){
    if(!id||!memory)return -1;
    for(uint8_t i=0;i<used;++i)if(slots[i].id==id)return slots[i].target==target?i:-1;
    int32_t delay=int32_t(received-target);
    if(!armed||used>=AutoSlots||delay< -250||delay>5000)return -1;
    auto& v=slots[used];v=Snapshot{};
    v.id=id;v.target=target;v.received=received;v.state=1;
    if(++nextGen<0x8000u)nextGen=0x8001u;
    v.gen=nextGen;
    return used++;
  }
  Sample* Window(uint8_t index){return memory+AutoHistory+uint32_t(index)*AutoWindow;}
  const Sample* Window(uint8_t index)const{return memory+AutoHistory+uint32_t(index)*AutoWindow;}
  void Finish(uint8_t index){
    auto& v=slots[index];
    if(!v.count){v.flags|=3;v.state=4;return;}
    const Sample* w=Window(index);
    if(int32_t(w[0].ms-(v.target-3000u))>20)v.flags|=1;
    if(int32_t((v.target+1000u)-w[v.count-1].ms)>20)v.flags|=2;
    v.state=3;
  }
  bool Step(){
    if(!memory)return false;
    for(uint8_t i=0;i<used;++i){
      auto& v=slots[i];
      if(v.state==1){
        if(!historyCount){if(armed)continue;Finish(i);return true;}
        uint32_t newest=memory[(total-1u)%AutoHistory].ms;
        if(armed&&int32_t(newest-(v.target+1000u))<0)continue;
        v.cursor=total-historyCount;v.end=total;v.state=2;
      }
      if(v.state!=2)continue;
      if(v.cursor==v.end){Finish(i);return true;}
      if(uint32_t(total-v.cursor)>historyCount){
        v.flags|=4;v.cursor=total-historyCount;
        if(int32_t(v.cursor-v.end)>=0){Finish(i);return true;}
      }
      const Sample s=memory[v.cursor%AutoHistory];++v.cursor;
      if(int32_t(s.ms-(v.target+1000u))>0){Finish(i);return true;}
      if(int32_t(s.ms-(v.target-3000u))>=0){
        if(v.count>=AutoWindow){v.flags|=8;Finish(i);return true;}
        auto* w=Window(i);
        if(v.count&&uint32_t(s.ms-w[v.count-1].ms)>20u)v.flags|=16;
        w[v.count++]=s;
      }
      if(v.cursor==v.end)Finish(i);
      return true;
    }
    return false;
  }
  bool Range(uint16_t gen,uint16_t start,uint16_t n)const{
    if(armed||!memory)return false; // no transfer during auto acquisition
    for(uint8_t i=0;i<used;++i)if(slots[i].state==3&&slots[i].gen==gen)
      return n>0&&uint32_t(start)+n<=slots[i].count;
    return false;
  }
  bool Read(uint16_t gen,uint16_t index,Sample& s)const{
    if(!Range(gen,index,1))return false;
    for(uint8_t i=0;i<used;++i)if(slots[i].gen==gen){s=Window(i)[index];return true;}
    return false;
  }
};
inline bool ParseMarker(const char* s,uint16_t& id,uint32_t& target){
  uint32_t first=0,second=0;
  if(*s<'0'||*s>'9')return false;
  do{first=first*10u+uint32_t(*s-'0');if(first>65535u)return false;++s;}while(*s>='0'&&*s<='9');
  if(*s++!=','||*s<'0'||*s>'9')return false;
  do{uint32_t digit=uint32_t(*s-'0');if(second>(0xffffffffu-digit)/10u)return false;second=second*10u+digit;++s;}while(*s>='0'&&*s<='9');
  if(*s||!first)return false;
  id=uint16_t(first);target=second;return true;
}
inline void EncodeSnapshot(uint8_t foot,uint8_t index,const Snapshot& v,uint8_t out[20]){
  for(unsigned i=0;i<20;++i)out[i]=0;
  out[0]=0xD5;out[1]=foot;out[2]=index;out[3]=v.state;
  Put16(out+4,v.gen);Put16(out+6,v.id);Put32(out+8,v.target);Put32(out+12,v.received);
  Put16(out+16,v.count);out[18]=v.flags;
}
} // namespace BiofitDiag
