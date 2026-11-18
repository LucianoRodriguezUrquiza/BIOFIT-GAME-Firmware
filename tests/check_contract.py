"""Source/protocol regression checks, not an ESP32 build or physical validation."""
from pathlib import Path
import re, subprocess, struct
root = Path(__file__).resolve().parents[1]
paths = sorted(root.glob('Firmware/**/*.ino'))
a,b = (p.read_text() for p in paths)
assert re.sub(r'#define TRACKER_ID [12]', '#define TRACKER_ID X',a) == re.sub(r'#define TRACKER_ID [12]', '#define TRACKER_ID X',b)
assert '#define TRACKER_ID 2' in a and '#define TRACKER_ID 1' in b
headers=sorted(root.glob('Firmware/**/BiofitPhaseEvidence.h'))
assert len(headers)==2 and headers[0].read_bytes()==headers[1].read_bytes()
for p in paths:
    now=p.read_text()
    old=subprocess.check_output(['git','show','9bb4183:'+str(p.relative_to(root))],cwd=root,text=True)
    def function(src,name):
        start=src.index(name+'(')
        # Skip declarations; find the actual definition.
        pattern=r'(?:void|bool|RuntimeConfig) '+name+r'\([^;]*?\)\s*\{'
        m=re.search(pattern,src); assert m,name
        begin=m.start(); level=0
        for i in range(src.index('{',m.start()),len(src)):
            if src[i]=='{': level+=1
            if src[i]=='}':
                level-=1
                if level==0:return src[begin:i+1]
    for name in ['loadConfig','saveConfig','loadSavedAxisMap','saveAxisMapInMainLoop','tryRestoreSavedAxisMap','applyAutoRest']:
        new=function(now,name).replace('  phaseEvidence.Reset(); // only after 2-second verified raw stationary window\n','')
        assert function(old,name).replace('s.us / 1000UL','s.ms')==new,name
    assert now.count('phaseEvidence.Reset();') == 2  # CAL and auto-rest only
    assert 'pushEvent(3,ms,gyro,fUp,(uint8_t)(0x03 | (phaseReady ? 0x10 : 0x00)))' in now
    assert 'phaseEvidence.Observe(ms,fUp,gyro,fActivity,aNorm,gravityNorm)' in now
    assert 'put16(q+18,228)' in now
    assert now.count('d.restCount=(uint16_t)min(')==2
    assert 's.us/1000UL' not in now and 's.us / 1000UL' not in now
    tx=now[now.index('void transportTask(void*) {'):]
    tx=re.sub(r'//[^\n]*','',tx)
    assert 'Serial.' not in tx and 'lcd->' not in tx and 'prefs.' not in tx
    assert 'put16(evt.b+14,age)' in tx and 'evt.b[3] |= 0x80' in tx
    assert 'BLEUUID(UUID_SERVICE), 32' in now
    assert 'STATUS_EVERY_MS = 100' in now
    packet=struct.pack('<BBBBHIhhH',2,1,3,0x81,65535,0xffffffff,250,120,1500)
    assert len(packet)==16 and struct.unpack_from('<H',packet,14)[0]==1500
print('PASS: mirror identity, NVS/map/reanchor preservation, clock source, TX isolation and 16-byte protocol. Static checks only.')
