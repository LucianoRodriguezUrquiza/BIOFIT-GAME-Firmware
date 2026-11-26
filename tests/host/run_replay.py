"""Compile actual estimator/calibration/detector code extracted from .ino. Hardware IO mocked."""
from pathlib import Path
import subprocess, tempfile, sys
root=Path(__file__).resolve().parents[2]
sketch=next(root.glob('Firmware/Tobillo_Izquierdo/**/*.ino'))
s=sketch.read_text()
if '--baseline' in sys.argv:
 s=subprocess.check_output(['git','show','f52b188:'+str(sketch.relative_to(root))],cwd=root,text=True)
def function(name):
 import re
 m=re.search(r'(?:void|bool|float|static int16_t|RuntimeConfig) '+name+r'\([^;]*?\)\s*\{',s);assert m,name
 level=0
 for i in range(s.index('{',m.start()),len(s)):
  if s[i]=='{':level+=1
  if s[i]=='}':
   level-=1
   if not level:return s[m.start():i+1]
def section(a,b):return s[s.index(a):s.index(b)]
pre='''#include <cmath>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <cstdio>
using std::min;using std::max;using std::isfinite;
int dataMux;void portENTER_CRITICAL(int*){}void portEXIT_CRITICAL(int*){}
template<class T>T constrain(T v,T lo,T hi){return std::max(lo,std::min(hi,v));}
struct SerialMock { template<class...T> void printf(const char*,T...){} void println(const char*){} } Serial;
bool serialPlotMode=true,requestCalibration=false;
constexpr float DEG_TO_RAD_F=0.01745329252f,G_MS2=9.80665f;
void pushEvent(uint8_t,uint32_t,float,float,uint8_t);
'''
pre+=section('struct RuntimeConfig {','// Prototipos explicitos')+function('defaults')+function('validConfig')+'\nRuntimeConfig cfg;RuntimeConfig configSnapshot(){return cfg;}\n'
pre+=section('struct ImuSample {','// Prototipos explicitos para evitar')
pre+=section('struct LiveData {','LiveData liveSnapshot();')+'\nLiveData live={};LiveData liveSnapshot(){return live;}void setLive(const LiveData &d){live=d;}\n'
for n in ['length3','unit3','toI16']:pre+=function(n)+'\n'
pre+=(sketch.parent/'BiofitLiftCycle.h').read_text().replace('#pragma once','')+'\n'
pre+=(sketch.parent/'BiofitDiagnostic.h').read_text().replace('#pragma once','')+'\n'
pre+=(sketch.parent/'BiofitPhaseEvidence.h').read_text().replace('#pragma once','')+'\n'
pre+=section('struct MotionDetector {','// Los vectores de montaje se guardan')
pre+=function('tryRestoreSavedAxisMap')+'\nvoid axisCaptureStep(const ImuSample&){}void finishTest(){activeTest=0;}void startTest(uint8_t,uint32_t){}\n'
pre+=section('float fGyro[3]', 'TaskHandle_t imuTaskHandle')
harness=Path(__file__).with_name('replay_diagnostics.cpp' if '--diagnostics' in sys.argv else 'replay.cpp')
if '--raw-input' in sys.argv:
 harness=Path(__file__).with_name('replay_raw.cpp')
 rawPath=sys.argv[sys.argv.index('--raw-input')+1]
 sys.argv=[sys.argv[0],rawPath]
pre+=harness.read_text()
with tempfile.TemporaryDirectory() as d:
 p=Path(d,'replay.cpp');p.write_text(pre)
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-unused-function',str(p),'-o',d+'/replay'],check=True)
 subprocess.run([d+'/replay']+sys.argv[1:],check=True)
