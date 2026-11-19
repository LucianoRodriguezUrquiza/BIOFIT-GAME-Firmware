// Approximate reconstruction only: 20Hz RAW linearly interpolated to 5ms.
// Missing internal samples/NVS/state cannot be reconstructed by interpolation.
#include <fstream>
#include <iostream>
#include <array>
struct Event {int type;uint32_t ms;float up;};std::vector<Event> events;
void pushEvent(uint8_t type,uint32_t ms,float,float up,uint8_t){events.push_back({type,ms,up});}
int main(int argc,char**argv){
 if(argc!=2)return 2;
 std::ifstream input(argv[1]);uint32_t t;std::array<float,6> a;std::vector<std::pair<uint32_t,std::array<float,6>>> samples;
 while(input>>t>>a[0]>>a[1]>>a[2]>>a[3]>>a[4]>>a[5])samples.push_back({t,a});
 if(samples.empty())return 3;
 int reasons[12]={};
 cfg=defaults();beginCalibration(samples[0].first-1000);
 for(int k=1;k<=200;k++){ImuSample s={};s.ms=samples[0].first-1000+k*5;for(int j=0;j<3;j++){s.a[j]=samples[0].second[j];s.w[j]=samples[0].second[j+3];}processSample(s,.005f);}
 for(size_t i=1;i<samples.size();i++){
  auto x=samples[i-1],y=samples[i];uint32_t span=y.first-x.first;
  if(span>150){liftCycle.Reset();continue;}
  uint32_t prev=0;for(uint32_t k=std::min(5u,span);k<=span;){
   float f=float(k)/span;ImuSample s={};s.ms=x.first+k;s.us=s.ms*1000;
   for(int j=0;j<3;j++){s.a[j]=x.second[j]+f*(y.second[j]-x.second[j]);s.w[j]=x.second[j+3]+f*(y.second[j+3]-x.second[j+3]);}
   processSample(s,(k-prev)*.001f);++reasons[(int)liftCycle.reason];if(k==span)break;prev=k;k=std::min(k+5,span);
  }
 }
 for(int i=0;i<12;i++){std::cerr<<"reason"<<i<<"="<<reasons[i]<<" ";}std::cerr<<"faults="<<liftCycle.faults<<" rearms="<<liftCycle.rearms<<"\n";
 for(auto e:events)if(e.type>=3)std::cout<<e.type<<","<<e.ms<<","<<e.up<<"\n";
}
