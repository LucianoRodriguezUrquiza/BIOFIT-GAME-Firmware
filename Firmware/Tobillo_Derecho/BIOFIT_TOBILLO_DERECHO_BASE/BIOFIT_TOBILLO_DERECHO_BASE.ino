/*
  BIOFIT GAME | UNIVERSAL TRACKER v2.2.7 + SERIAL PLOTTER EJE_X/EJE_Y/EJE_Z
  Hardware: Waveshare ESP32-S3-LCD-1.47B (NO Touch), IMU QMI8658A.
  Arduino-ESP32 core 3.3.x; instalar "GFX Library for Arduino".

  MISMO firmware en ambas placas. Cambiar SOLO TRACKER_ID:
    1 = tobillo izquierdo; 2 = tobillo derecho; 3/4 = futuras munecas.

  El firmware NO cuenta pasos biomecanicos ni determina contacto con el suelo.
  Detecta actividad MOV_ON / MOV_OFF y un candidato UP_IMPULSE.
  Unity decide STEP, caminar, correr y JUMP a partir de ambos trackers.
  Compatibilidad del protocolo BLE V2: EVENT/STATUS/CONFIG/RAW sin cambios.
  Extension opcional AXES para diagnostico XYZ corporal.
  Serial Plotter: arranca automaticamente con SOLO 3 curvas: Eje_X/Eje_Y/Eje_Z (g).
  Auto-reposo PERIODICO: cada ventana valida de 2 s estable por IMU CRUDA,
  reancla gravedad y sesgo gyro; incluso si permanece quieto mucho tiempo.
  No borra mapa anatomico NVS ni simula desplazamientos en centimetros.
  AUTO,STAT consulta correcciones. AUTO,0 desactiva; AUTO,1 activa.
  Para calibrar por primera vez: CAL, AXIS,TILT, AXIS,APPLY.
  Orientacion XYZ se almacena en NVS y se recupera si montaje neutral coincide.
  Usar PLOT,0 para mensajes; PLOT,1 para grafico.
  PLOT,RATE,5 (10 s), PLOT,RATE,10 (5 s), PLOT,RATE,25 (2 s), PLOT,RATE,3 (16 s).
  AXIS,YAW,+10 / AXIS,YAW,-10: ajuste incremental opcional X/Y, sin alterar Z.

  Orientacion: se estima la vertical inicial desde gravedad y luego con gyro;
  NO se equiparan los ejes internos del QMI a ejes anatomicos X/Y/Z.
  Sin magnetometro no es posible obtener un yaw absoluto sin deriva.
  AXIS,TILT + AXIS,APPLY permiten calibrar X/Y en pose neutral.
  NO mide posicion/altura, solo aceleracion lineal estimada (g).
  Ajustar thresholds despues de medir muestras REALES.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

// Modo plotter persistente. Evita mezclar texto de depuracion y graficas.
// Se carga desde NVS en loadConfig(); puede cambiarse con PLOT,1 / PLOT,0.
// La frecuencia USB de Plotter se cambia por PLOT,RATE y se guarda en NVS.
volatile bool serialPlotMode = false;

// ============= ROL (unica diferencia entre los dos archivos) =========
#define TRACKER_ID 2
#if TRACKER_ID == 1
  #define TRACKER_NAME "BIOFIT-LEFT-01"
  #define TRACKER_LABEL "TOBILLO IZQUIERDO"
#elif TRACKER_ID == 2
  #define TRACKER_NAME "BIOFIT-RIGHT-02"
  #define TRACKER_LABEL "TOBILLO DERECHO"
#elif TRACKER_ID == 3
  #define TRACKER_NAME "BIOFIT-LEFTWRIST-03"
  #define TRACKER_LABEL "MUNECA IZQUIERDA"
#elif TRACKER_ID == 4
  #define TRACKER_NAME "BIOFIT-RIGHTWRIST-04"
  #define TRACKER_LABEL "MUNECA DERECHA"
#else
  #error TRACKER_ID debe estar entre 1 y 4
#endif

// BLE UUIDs: mismos en TODOS los trackers, dispositivos distintos por nombre/ID.
static const char UUID_SERVICE[] = "f9710000-6d68-4b7d-a742-6fae7b5f0001";
static const char UUID_EVENTS[]  = "f9710000-6d68-4b7d-a742-6fae7b5f0002";
static const char UUID_STATUS[]  = "f9710000-6d68-4b7d-a742-6fae7b5f0003";
static const char UUID_CONFIG[]  = "f9710000-6d68-4b7d-a742-6fae7b5f0004";
static const char UUID_RAW[]     = "f9710000-6d68-4b7d-a742-6fae7b5f0005";
static const char UUID_AXES[]    = "f9710000-6d68-4b7d-a742-6fae7b5f0006"; // extension opcional V2.1

static constexpr int IMU_SDA = 48, IMU_SCL = 47, IMU_INT2 = 12;
static constexpr int LCD_DC = 41, LCD_CS = 42, LCD_SCLK = 40;
static constexpr int LCD_MOSI = 45, LCD_RST = 39, LCD_BL = 46;
static constexpr float ODR_HZ = 224.2f;
static constexpr float DEG_TO_RAD_F = 0.01745329252f;
static constexpr float G_MS2 = 9.80665f;
static constexpr uint32_t STATUS_EVERY_MS = 100; // estado a 10 Hz, eventos tienen prioridad
static constexpr uint32_t RAW_EVERY_MS = 50;    // seis ejes a 20 Hz si STREAM=1
static constexpr uint32_t SCREEN_EVERY_MS = 100;
static constexpr uint32_t AXES_EVERY_MS = 50; // opcional via AXSTREAM,1
// El Serial Plotter conserva una ventana corta de muestras.
// 5 Hz -> ~10 s visibles; 10 Hz -> ~5 s; 25 Hz -> ~2 s.
// SOLO afecta al grafico USB: IMU, BLE y deteccion permanecen igual.
volatile uint16_t plotEveryMs = 200; // 5 Hz por defecto; modificable PLOT,RATE,<Hz>

// LCD: ST7789, panel 172x320, modo apaisado 320x172.
Arduino_DataBus *lcdBus = new Arduino_ESP32SPI(
  LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI, -1);
Arduino_GFX *lcd = new Arduino_ST7789(
  lcdBus, LCD_RST, 1, true, 172, 320, 34, 0, 34, 0);
bool lcdReady = false;
static constexpr uint16_t C_BG = 0x0000, C_BLUE = 0x241F;
static constexpr uint16_t C_NAVY = 0x0213, C_GREEN = 0x07E0;
static constexpr uint16_t C_ORANGE = 0xFD20, C_WHITE = 0xFFFF;
static constexpr uint16_t C_GRAY = 0xBDF7;

struct RuntimeConfig {
  float movOnDps;      // gyro movimiento, dps
  float movOffDps;     // gyro para quietud, dps
  float movOnG;        // magnitud acc dinamica, g
  float movOffG;       // magnitud acc para quietud, g
  float upOnG;         // aceleracion ascendente compensada, g
  float upActG;        // actividad acelerometrica complementaria, g
  float upVms;         // velocidad vertical transitoria, m/s (no altura)
  float filterHz;      // frecuencia de corte filtro IIR, Hz
  uint16_t confirmMs; // para activar MOV
  uint16_t releaseMs; // baja actividad para MOV_OFF
  uint16_t upGapMs;   // intervalo minimo entre pulsos UP
};
// Prototipos explicitos por compatibilidad con el preprocesador de Arduino IDE.
void loadSavedAxisMap();
RuntimeConfig defaults();
bool validConfig(const RuntimeConfig &c);
RuntimeConfig configSnapshot();
void setConfig(const RuntimeConfig &next);
String configValue(const RuntimeConfig &c, const String &key); // tipo definido antes del prototipo

RuntimeConfig defaults() {
  RuntimeConfig c;
  c.movOnDps = 24.0f; c.movOffDps = 11.0f;
  c.movOnG = 0.075f; c.movOffG = 0.035f;
  c.upOnG = 0.090f; c.upActG = 0.035f;
  c.upVms = 0.022f; c.filterHz = 22.0f;
  c.confirmMs = 12; c.releaseMs = 130; c.upGapMs = 260;
  return c;
}

bool validConfig(const RuntimeConfig &c) {
  return isfinite(c.movOnDps) && c.movOnDps >= 8 && c.movOnDps <= 130 &&
    isfinite(c.movOffDps) && c.movOffDps >= 2 && c.movOffDps <= 70 &&
    c.movOffDps < c.movOnDps &&
    isfinite(c.movOnG) && c.movOnG >= 0.020f && c.movOnG <= 0.50f &&
    isfinite(c.movOffG) && c.movOffG >= 0.008f && c.movOffG <= 0.25f &&
    c.movOffG < c.movOnG &&
    isfinite(c.upOnG) && c.upOnG >= 0.035f && c.upOnG <= 0.60f &&
    isfinite(c.upActG) && c.upActG >= 0.008f && c.upActG <= 0.25f &&
    isfinite(c.upVms) && c.upVms >= 0.005f && c.upVms <= 0.20f &&
    isfinite(c.filterHz) && c.filterHz >= 4 && c.filterHz <= 50 &&
    c.confirmMs >= 4 && c.confirmMs <= 100 &&
    c.releaseMs >= 40 && c.releaseMs <= 550 &&
    c.upGapMs >= 120 && c.upGapMs <= 900;
}

RuntimeConfig cfg;
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
RuntimeConfig configSnapshot() {
  RuntimeConfig copy;
  portENTER_CRITICAL(&dataMux);
  copy = cfg;
  portEXIT_CRITICAL(&dataMux);
  return copy;
}
void setConfig(const RuntimeConfig &next) {
  portENTER_CRITICAL(&dataMux);
  cfg = next;
  portEXIT_CRITICAL(&dataMux);
}

Preferences prefs;
void loadConfig() {
  RuntimeConfig c = defaults();
  prefs.begin("biofit_v2", false);
  // v2.2.4: modo grafico DESDE EL ARRANQUE. Aunque una version anterior
  // haya guardado PLOT,0 en NVS, la salida ya no mezcla registros numericos
  // de diagnostico (que aparecen como value1..value7 y desescalan el grafico).
  // PLOT,0 activa Monitor Serie para calibrar; PLOT,1 vuelve al Plotter.
  serialPlotMode = true;
  const uint8_t hz=prefs.getUChar("plotHz",5);
  plotEveryMs=(hz==3 || hz==5 || hz==10 || hz==25) ? (uint16_t)(1000/hz) : 200;
  if (prefs.getUInt("version", 0) == 2) {
    c.movOnDps = prefs.getFloat("movOnDps", c.movOnDps);
    c.movOffDps = prefs.getFloat("movOffDps", c.movOffDps);
    c.movOnG = prefs.getFloat("movOnG", c.movOnG);
    c.movOffG = prefs.getFloat("movOffG", c.movOffG);
    c.upOnG = prefs.getFloat("upOnG", c.upOnG);
    c.upActG = prefs.getFloat("upActG", c.upActG);
    c.upVms = prefs.getFloat("upVms", c.upVms);
    c.filterHz = prefs.getFloat("filterHz", c.filterHz);
    c.confirmMs = prefs.getUShort("confMs", c.confirmMs);
    c.releaseMs = prefs.getUShort("relMs", c.releaseMs);
    c.upGapMs = prefs.getUShort("upGapMs", c.upGapMs);
  }
  if (!validConfig(c)) c = defaults();
  setConfig(c);
}
void saveConfig() {
  const RuntimeConfig c = configSnapshot();
  prefs.putUInt("version", 0); // guardado no valido durante escritura
  prefs.putFloat("movOnDps", c.movOnDps);
  prefs.putFloat("movOffDps", c.movOffDps);
  prefs.putFloat("movOnG", c.movOnG);
  prefs.putFloat("movOffG", c.movOffG);
  prefs.putFloat("upOnG", c.upOnG);
  prefs.putFloat("upActG", c.upActG);
  prefs.putFloat("upVms", c.upVms);
  prefs.putFloat("filterHz", c.filterHz);
  prefs.putUShort("confMs", c.confirmMs);
  prefs.putUShort("relMs", c.releaseMs);
  prefs.putUShort("upGapMs", c.upGapMs);
  prefs.putUInt("version", 2);
}

// ============= SENSOR QMI8658, registros verificados ================
static constexpr uint8_t QMI_WHO = 0x00, QMI_CTRL1 = 0x02;
static constexpr uint8_t QMI_CTRL2 = 0x03, QMI_CTRL3 = 0x04;
static constexpr uint8_t QMI_CTRL5 = 0x06, QMI_CTRL7 = 0x08;
static constexpr uint8_t QMI_STATUS0 = 0x2E, QMI_AX_L = 0x35;
uint8_t imuAddr = 0;
volatile uint32_t i2cErrors = 0;
volatile bool imuOk = false;

bool regRead(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imuAddr, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}
bool regWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool regBlock(uint8_t reg, uint8_t *dest, uint8_t n) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imuAddr, n) != n) return false;
  for (uint8_t i = 0; i < n; ++i) dest[i] = Wire.read();
  return true;
}
bool beginIMU() {
  for (uint8_t address : { (uint8_t)0x6A, (uint8_t)0x6B }) {
    imuAddr = address;
    uint8_t who = 0;
    if (regRead(QMI_WHO, who) && who == 0x05) break;
    imuAddr = 0;
  }
  if (!imuAddr) return false;
  if (!regWrite(QMI_CTRL7, 0x00)) return false;
  if (!regWrite(QMI_CTRL1, 0x50)) return false; // autoincremento, little endian, INT2
  if (!regWrite(QMI_CTRL2, 0x25)) return false; // +/-8 g, 6DOF 224.2 Hz
  if (!regWrite(QMI_CTRL3, 0x55)) return false; // +/-512 dps, 6DOF 224.2 Hz
  if (!regWrite(QMI_CTRL5, 0x00)) return false; // filtro causal en ESP32
  if (!regWrite(QMI_CTRL7, 0x03)) return false; // gyro + accel
  delay(150);
  return true;
}

struct ImuSample {
  float a[3]; // g, ejes internos QMI
  float w[3]; // dps, ejes internos QMI
  uint32_t us;
  uint32_t ms; // millis real: no dividir micros (wrap a los 71 minutos)
};

// Prototipos explicitos para evitar preprocesador Arduino con structs.
bool readSample(ImuSample &s);
void processSample(const ImuSample &s, float dt);
void axisCaptureStep(const ImuSample &s);
void calibrationSample(const ImuSample &s, uint32_t nowMs); // Correccion Arduino IDE: prototipo explicito
void observeAutoRest(const ImuSample &s, uint32_t ms); // Evita error del autoproto Arduino con ImuSample

static int16_t readLE16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
bool readSample(ImuSample &s) {
  uint8_t st = 0;
  if (!regRead(QMI_STATUS0, st)) { ++i2cErrors; return false; }
  if ((st & 0x03) != 0x03) return false;
  uint8_t b[12];
  if (!regBlock(QMI_AX_L, b, sizeof(b))) { ++i2cErrors; return false; }
  s.us = micros();
  s.ms = millis();
  for (int i = 0; i < 3; ++i) {
    s.a[i] = readLE16(b + 2*i) / 4096.0f;
    s.w[i] = readLE16(b + 6 + 2*i) / 64.0f;
  }
  return true;
}
float length3(const float v[3]) {
  return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
void unit3(float v[3]) {
  float n = length3(v);
  if (n > 0.1f) { v[0]/=n; v[1]/=n; v[2]/=n; }
}
float max3(float a, float b, float c) { return fmaxf(a, fmaxf(b,c)); }

// ================== ESTADOS COMPARTIDOS / PAQUETES ====================
struct LiveData {
  uint32_t ms;
  int16_t rawAxMg, rawAyMg, rawAzMg;
  int16_t rawGxX10, rawGyX10, rawGzX10;
  int16_t upMg, gyroX10, activityMg;
  // Aceleraciones lineales y giros en ejes ANATOMICOS, SOLO con AXIS OK.
  int16_t bodyAxMg, bodyAyMg, bodyAzMg;
  int16_t bodyGxX10, bodyGyX10, bodyGzX10;
  uint16_t imuHz;
  uint32_t motionCount, upCount;
  uint8_t intensity, flags;
  uint16_t restCount, disarmedMs, biasNormX100, gravityMg, calSamples;
  int16_t velocityMmS;
  uint16_t rearms, blockedSamples, restRejected;
  uint8_t upBlockReason, autoEnabled;
};
LiveData liveSnapshot();
void setLive(const LiveData &next);
// flags: bit0 MOV, bit1 UP reciente, bit2 calibrado, bit3 IMU OK, bit4 CAL activo
LiveData live = {};
LiveData liveSnapshot() {
  LiveData copy;
  portENTER_CRITICAL(&dataMux);
  copy = live;
  portEXIT_CRITICAL(&dataMux);
  return copy;
}
void setLive(const LiveData &next) {
  portENTER_CRITICAL(&dataMux);
  live = next;
  portEXIT_CRITICAL(&dataMux);
}

static int16_t toI16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}
static void put16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static void put32(uint8_t *p, uint32_t v) {
  for (int i=0; i<4; ++i) p[i] = (uint8_t)(v >> (i*8));
}

struct EventPacket { uint8_t b[16]; uint32_t queuedMs; };
struct CommandPacket { char text[64]; };
QueueHandle_t eventQueue = nullptr, commandQueue = nullptr;
volatile uint32_t droppedEvents = 0;
volatile bool bleConnected = false;
volatile bool requestCalibration = false;
volatile bool serialCsv = false;
volatile bool streamRaw = false;
volatile bool streamAxes = false;
BLECharacteristic *chrEvents = nullptr, *chrStatus = nullptr;
BLECharacteristic *chrConfig = nullptr, *chrRaw = nullptr, *chrAxes = nullptr;
BLEServer *bleServer = nullptr;
BLECharacteristic *chrTransport = nullptr;
BLE2902 *eventsCccd = nullptr;
static const char UUID_TRANSPORT[] = "f9710000-6d68-4b7d-a742-6fae7b5f0007";
uint16_t queueHighWater=0, maxQueueAgeMs=0, maxNotifyMs=0, maxTransportGapMs=0;
volatile uint32_t notifyErrors=0;
uint16_t transportSeq=0;
static uint16_t capped16(uint32_t v) { return v > 65535 ? 65535 : (uint16_t)v; }
class TransportCallbacks : public BLECharacteristicCallbacks {
  void onStatus(BLECharacteristic*, Status status, uint32_t) override {
    // SUCCESS_NOTIFY means accepted by stack, NOT received by Android.
    if (status == Status::ERROR_GATT) ++notifyErrors;
  }
};
void timedNotify(BLECharacteristic *characteristic) {
  const uint32_t beginMs=millis();
  characteristic->notify();
  const uint16_t elapsed=capped16((uint32_t)(millis()-beginMs));
  if (elapsed>maxNotifyMs) maxNotifyMs=elapsed;
}
void transportTask(void*);

uint16_t eventSeq = 0, statusSeq = 0, rawSeq = 0, axisSeq = 0;

void pushEvent(uint8_t type, uint32_t timestamp, float gyroDps, float upG, uint8_t flags) {
  EventPacket p = {};
  p.queuedMs = millis();
  p.b[0] = 2; p.b[1] = TRACKER_ID; p.b[2] = type; p.b[3] = flags;
  put16(p.b+4, ++eventSeq);
  put32(p.b+6, timestamp);
  put16(p.b+10, (uint16_t)toI16(gyroDps * 10.0f));
  put16(p.b+12, (uint16_t)toI16(upG * 1000.0f));
  put16(p.b+14, 0); // reservado
  if (xQueueSend(eventQueue, &p, 0) != pdTRUE) ++droppedEvents;
  if (!serialPlotMode) Serial.printf("EV,%lu,%u,%u,%u,%.1f,%+.3f\n",
    (unsigned long)timestamp, TRACKER_ID, type, eventSeq, gyroDps, upG);
}

class BiofitServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bleConnected = true; }
  void onDisconnect(BLEServer*) override { bleConnected = false; }
};
class ConfigWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *p) override {
    String s = p->getValue();
    s.trim();
    if (!s.length()) return;
    CommandPacket c = {};
    s.toCharArray(c.text, sizeof(c.text));
    if (commandQueue != nullptr) xQueueSend(commandQueue, &c, 0);
  }
};
void startBLE() {
  BLEDevice::init(TRACKER_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BiofitServerCallbacks());
  BLEService *svc = bleServer->createService(BLEUUID(UUID_SERVICE), 32); // seis caracteristicas + CCCD
  chrEvents = svc->createCharacteristic(UUID_EVENTS,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  chrStatus = svc->createCharacteristic(UUID_STATUS,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  chrConfig = svc->createCharacteristic(UUID_CONFIG,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY);
  chrRaw = svc->createCharacteristic(UUID_RAW,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  chrAxes = svc->createCharacteristic(UUID_AXES,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  eventsCccd = new BLE2902();
  chrEvents->addDescriptor(eventsCccd);
  chrEvents->setCallbacks(new TransportCallbacks());
  chrStatus->addDescriptor(new BLE2902());
  chrConfig->addDescriptor(new BLE2902());
  chrRaw->addDescriptor(new BLE2902());
  chrAxes->addDescriptor(new BLE2902());
  chrConfig->setCallbacks(new ConfigWriteCallbacks());
  chrTransport = svc->createCharacteristic(UUID_TRANSPORT,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  chrTransport->addDescriptor(new BLE2902());
  chrStatus->setCallbacks(new TransportCallbacks());
  chrTransport->setCallbacks(new TransportCallbacks());
  uint8_t emptyE[16] = {}; emptyE[0]=2; emptyE[1]=TRACKER_ID;
  uint8_t emptyS[20] = {}; emptyS[0]=2; emptyS[1]=TRACKER_ID;
  chrEvents->setValue(emptyE, sizeof(emptyE));
  chrStatus->setValue(emptyS, sizeof(emptyS));
  chrRaw->setValue(emptyS, sizeof(emptyS));
  chrAxes->setValue(emptyS, sizeof(emptyS));
  chrConfig->setValue("BIOFIT,V2");
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_SERVICE);
  adv->setScanResponse(false);
  BLEDevice::startAdvertising();
  if (!serialPlotMode) Serial.printf("BLE: %s / %s\n", TRACKER_NAME, UUID_SERVICE);
}

// ============= CLASIFICADOR UNIVERSAL / ESTIMADOR ================
struct MotionDetector {
  bool moving = false;
  bool upArmed = true;
  bool downSeen = false;
  uint32_t onCandidateSince = 0;
  uint32_t releaseSince = 0;
  uint32_t lastUpMs = 0;
  uint32_t lastUpEventMs = 0;
  uint8_t upEvidence = 0;
  uint32_t moves = 0, ups = 0;
};
MotionDetector det;
float gyroBias[3] = {0,0,0};
float verticalUnit[3] = {0,0,1};
float gravityNorm = 1.0f;
// Ejes corporales expresados en coordenadas internas de la IMU.
// Para x neutro se calibra inclinando la PARTE SUPERIOR de la placa
// hacia adelante (+X) y volviendo a la pose neutral.
float baseZneutral[3] = {0,0,1};
float candidateXneutral[3] = {1,0,0};
float bodyXsensor[3] = {1,0,0};
float bodyYsensor[3] = {0,1,0};
bool axisTiltReady = false, axisReady = false;
// Geometria fija de la IMU relativa al montaje: guardar (Zneutral, Xneutral).
// A diferencia de los umbrales de MOV/UP, esta informacion define los ejes.
struct SavedAxisMap {
  uint32_t magic;
  float neutralZ[3];
  float neutralX[3];
};
static constexpr uint32_t SAVED_AXES_MAGIC = 0x42465832UL;
SavedAxisMap savedMap = {};
bool savedMapValid = false;
volatile bool axisSavePending = false;
volatile uint8_t axisRequest = 0; // 1=TILT, 2=APPLY, 3=RESET, 4=YAW TRIM
volatile int16_t pendingYawTenths = 0; // grados * 10 (aplicar en tarea IMU)
uint8_t axisCaptureMode = 0;
uint16_t axisSampleCount = 0;
float axisSum[3] = {0,0,0};
// Prueba con CSV a 25 Hz y resumen de magnitudes por eje.
volatile uint8_t pendingTest = 0; // 1..5, 6 STOP
uint8_t activeTest = 0;
uint32_t testStartMs = 0, testLastCsvMs = 0;
uint32_t testN = 0;
float testSumAbs[3] = {0,0,0}, testPeak[3] = {0,0,0};
float testSumAbsGyro[3] = {0,0,0};
const char* testName(uint8_t mode) {
  switch (mode) {
    case 1: return "REST";
    case 2: return "UP";
    case 3: return "X";
    case 4: return "Y";
    case 5: return "ROT";
    default: return "NONE";
  }
}
float dot3(const float a[3], const float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
void cross3(const float a[3], const float b[3], float out[3]) {
  out[0]=a[1]*b[2]-a[2]*b[1];
  out[1]=a[2]*b[0]-a[0]*b[2];
  out[2]=a[0]*b[1]-a[1]*b[0];
}
void rotateWorldVectorInSensor(float u[3], const float wDps[3], float dt) {
  const float w[3] = {wDps[0]*DEG_TO_RAD_F,
                      wDps[1]*DEG_TO_RAD_F,
                      wDps[2]*DEG_TO_RAD_F};
  float cross[3]; cross3(w,u,cross);
  for (int i=0; i<3; ++i) u[i] -= cross[i]*dt;
  unit3(u);
}
void orthonormalizeBodyAxes() {
  // Y = Z x X, X = Y x Z: define base derecha X x Y = Z.
  const float k = dot3(bodyXsensor, verticalUnit);
  for (int i=0; i<3; ++i) bodyXsensor[i] -= k*verticalUnit[i];
  unit3(bodyXsensor);
  cross3(verticalUnit,bodyXsensor,bodyYsensor); unit3(bodyYsensor);
  cross3(bodyYsensor,verticalUnit,bodyXsensor); unit3(bodyXsensor);
}
// Los vectores de montaje se guardan una sola vez tras AXIS,APPLY; el
// movimiento de las piernas NO se integra como desplazamiento XYZ.
void loadSavedAxisMap() {
  savedMapValid = false;
  if (prefs.getBytesLength("axisMap") != sizeof(SavedAxisMap)) return;
  SavedAxisMap stored = {};
  if (prefs.getBytes("axisMap", &stored, sizeof(stored)) != sizeof(stored)) return;
  if (stored.magic != SAVED_AXES_MAGIC) return;
  for (int i=0;i<3;++i) {
    if (!isfinite(stored.neutralZ[i]) || !isfinite(stored.neutralX[i])) return;
  }
  const float zNorm=length3(stored.neutralZ), xNorm=length3(stored.neutralX);
  if (zNorm<0.9f || zNorm>1.1f || xNorm<0.9f || xNorm>1.1f ||
      fabsf(dot3(stored.neutralX,stored.neutralZ))>0.12f) return;
  savedMap=stored;
  savedMapValid=true;
}
void tryRestoreSavedAxisMap() {
  if (!savedMapValid) return;
  const float agreement=dot3(savedMap.neutralZ, verticalUnit);
  if (agreement < 0.94f) {
    if (!serialPlotMode) Serial.println("AXIS: montaje distinto al guardado, recalibrar X/Y");
    return;
  }
  // Restaurar tambien el X neutral: necesario para conservar los futuros
  // ajustes AXIS,YAW al guardar otra vez la matriz en NVS.
  for (int i=0;i<3;++i) {
    bodyXsensor[i]=savedMap.neutralX[i];
    candidateXneutral[i]=savedMap.neutralX[i];
  }
  orthonormalizeBodyAxes();
  axisTiltReady = true;
  axisReady = true;
  if (!serialPlotMode) Serial.println("AXIS RESTAURADOS: XYZ disponible");
}
void saveAxisMapInMainLoop() {
  if (!axisSavePending) return;
  axisSavePending = false;
  SavedAxisMap updated = {};
  updated.magic = SAVED_AXES_MAGIC;
  // Estos vectores son los de la postura de calibracion inicial, no los
  // vectores dinamicos alterados por giroscopio durante el movimiento.
  for (int i=0;i<3;++i) {
    updated.neutralZ[i] = baseZneutral[i];
    updated.neutralX[i] = candidateXneutral[i];
  }
  if (prefs.putBytes("axisMap", &updated, sizeof(updated)) == sizeof(updated)) {
    savedMap=updated; savedMapValid=true;
    if (!serialPlotMode) Serial.println("AXIS GUARDADOS: sobreviven reinicios USB");
  } else {
    if (!serialPlotMode) Serial.println("WARN: no se pudo guardar calibracion XYZ");
  }
}

void finishTest() {
  if (!activeTest) return;
  float n=testN ? (float)testN : 1.0f;
  if (!serialPlotMode) Serial.printf("R,%s,N=%lu,MEAN_ABS_X=%.3f,MEAN_ABS_Y=%.3f,MEAN_ABS_Z=%.3f,PEAK_X=%.3f,PEAK_Y=%.3f,PEAK_Z=%.3f,GX=%.1f,GY=%.1f,GZ=%.1f,AXIS_OK=%u\n",
    testName(activeTest),(unsigned long)testN,
    testSumAbs[0]/n,testSumAbs[1]/n,testSumAbs[2]/n,
    testPeak[0],testPeak[1],testPeak[2],
    testSumAbsGyro[0]/n,testSumAbsGyro[1]/n,testSumAbsGyro[2]/n,
    axisReady ? 1U : 0U);
  if (!serialPlotMode) Serial.printf("T,END,%s\n",testName(activeTest));
  activeTest=0;
}
void startTest(uint8_t mode,uint32_t now) {
  finishTest();
  if (mode==6) return;
  activeTest=mode; testStartMs=now; testLastCsvMs=0; testN=0;
  for (int i=0; i<3; ++i) {
    testSumAbs[i]=0; testPeak[i]=0; testSumAbsGyro[i]=0;
  }
  if (!serialPlotMode) Serial.printf("T,START,%s,12s\n",testName(mode));
  if (!serialPlotMode) Serial.println("# B,ms,TEST,axS,ayS,azS,gxS,gyS,gzS,aX,aY,aZ,wX,wY,wZ,up_g,act_g,mov,axis_ok,Hz");
}
void axisCaptureStep(const ImuSample &s) {
  if (axisRequest) {
    uint8_t r=axisRequest; axisRequest=0;
    axisCaptureMode=0; axisSampleCount=0;
    for (float &v : axisSum) v=0;
    if (r==3) { axisReady=false; axisTiltReady=false;
      if (!serialPlotMode) Serial.println("AXIS RESET: solo Z calibrado; ejecuta AXIS,TILT y AXIS,APPLY");
      return;
    }
    // Ajuste fino SOLO en plano XY. NO toca Z ni la deteccion MOV/UP.
    // Un delta positivo gira +X hacia +Y; negativa gira +X hacia -Y.
    // Se ejecuta en la tarea IMU para no modificar ejes a la vez desde loop().
    if (r==4) {
      if (!axisReady) {
        if (!serialPlotMode) Serial.println("ERR AXIS YAW: primero AXIS,APPLY");
        return;
      }
      const float radians=(pendingYawTenths*0.1f)*DEG_TO_RAD_F;
      const float co=cosf(radians), si=sinf(radians);
      float liveY[3], neutralY[3];
      cross3(verticalUnit,bodyXsensor,liveY); unit3(liveY);
      cross3(baseZneutral,candidateXneutral,neutralY); unit3(neutralY);
      for (int i=0;i<3;++i) {
        bodyXsensor[i]=co*bodyXsensor[i]+si*liveY[i];
        candidateXneutral[i]=co*candidateXneutral[i]+si*neutralY[i];
      }
      orthonormalizeBodyAxes();
      const float norm=length3(candidateXneutral);
      if (norm>0.1f) for (int i=0;i<3;++i) candidateXneutral[i]/=norm;
      axisSavePending=true;
      if (!serialPlotMode) Serial.printf("AXIS YAW OK: delta=%+.1f grados; guardando en NVS\n", pendingYawTenths*0.1f);
      return;
    }
    if (r==2 && !axisTiltReady) {
      if (!serialPlotMode) Serial.println("ERR AXIS: primero AXIS,TILT"); return;
    }
    axisCaptureMode=r;
    if (!serialPlotMode) Serial.printf("AXIS CAPTURE: %s. MANTENE LA POSTURA QUIETA...\n",
      r==1?"INCLINADO HACIA ADELANTE":"NEUTRAL");
  }
  if (!axisCaptureMode) return;
  float w[3]; for (int i=0;i<3;++i) w[i]=s.w[i]-gyroBias[i];
  float an=length3(s.a), wn=length3(w);
  if (fabsf(an-gravityNorm)>0.12f || wn>8.0f) {
    axisSampleCount=0; for(float &v:axisSum) v=0;
    return;
  }
  for (int i=0;i<3;++i) axisSum[i]+=s.a[i];
  ++axisSampleCount;
  if (axisSampleCount<75) return;
  float captured[3];
  for (int i=0;i<3;++i) captured[i]=axisSum[i]/axisSampleCount;
  unit3(captured);
  const uint8_t mode=axisCaptureMode;
  axisCaptureMode=0; axisSampleCount=0;
  if (mode==1) {
    const float tiltCos=dot3(captured,baseZneutral);
    float delta[3];
    for(int i=0;i<3;++i) delta[i]=baseZneutral[i]-captured[i];
    const float proj=dot3(delta,baseZneutral);
    for(int i=0;i<3;++i) delta[i]-=proj*baseZneutral[i];
    const float magnitude=length3(delta);
    if (magnitude < 0.13f || magnitude > 0.86f || tiltCos < 0.55f) {
      axisTiltReady=false; axisReady=false;
      if (!serialPlotMode) Serial.printf("ERR AXIS TILT: inclinacion no valida (proy=%.3f). Proba inclinacion de ~15 a 35 grados.\n",magnitude);
      return;
    }
    for(int i=0;i<3;++i) candidateXneutral[i]=delta[i]/magnitude;
    axisTiltReady=true; axisReady=false;
    if (!serialPlotMode) Serial.printf("AXIS TILT OK: forwardNeutral=(%.3f,%.3f,%.3f) ; ahora volve a vertical y manda AXIS,APPLY\n",
      candidateXneutral[0],candidateXneutral[1],candidateXneutral[2]);
    return;
  }
  const float agreement=dot3(captured,baseZneutral);
  if (agreement<0.96f) {
    axisReady=false;
    if (!serialPlotMode) Serial.printf("ERR AXIS APPLY: no volviste a pose neutral (dot=%.3f). Endereza y repeti AXIS,APPLY\n",agreement);
    return;
  }
  for(int i=0;i<3;++i) {
    verticalUnit[i]=captured[i]; bodyXsensor[i]=candidateXneutral[i];
  }
  orthonormalizeBodyAxes(); axisReady=true;
  axisSavePending=true;
  if (!serialPlotMode) Serial.printf("AXIS OK: Xs=(%.3f,%.3f,%.3f) Ys=(%.3f,%.3f,%.3f) Zs=(%.3f,%.3f,%.3f)\n",
    bodyXsensor[0],bodyXsensor[1],bodyXsensor[2],
    bodyYsensor[0],bodyYsensor[1],bodyYsensor[2],
    verticalUnit[0],verticalUnit[1],verticalUnit[2]);
  if (!serialPlotMode) Serial.println("AXIS: ejes del CUERPO estimados. X adelante, Y izquierda, Z arriba. No son centimetros.");
}

float fGyro[3] = {0,0,0};
float fActivity = 0, fUp = 0, shortV = 0;
uint32_t imuSamples = 0, lastRateTime = 0, lastRateCount = 0;
uint16_t imuRateHz = 0;

// ===== AUTO-REPOSO: independiente del detector MOV y de fUp =====
// La placa NO necesita estar exactamente vertical. La aceleracion cruda
// debe ser estable (baja varianza), la norma proxima a 1g y el giro pequeno.
// Cada ventana COMPLETA de 2 s en reposo actualiza bias y gravedad.
// No hay bloqueo de una sola correccion por intervalo de quietud prolongado.
// NO modifica la calibracion anatomica neutral almacenada en NVS.
static constexpr uint32_t AUTO_REST_MS = 2000;
static constexpr float AUTO_REST_GYRO_DPS = 7.0f;
static constexpr float AUTO_REST_MIN_G = 0.80f;
static constexpr float AUTO_REST_MAX_G = 1.20f;
static constexpr float AUTO_REST_STD_G = 0.025f; // desviacion tipica por eje
static constexpr float AUTO_REST_STD_W = 2.5f;   // dps por eje
volatile bool calibrated = false;
volatile bool autoRestEnabled = true;
// Main/CONFIG only requests; the IMU task owns all rest accumulators.
volatile int8_t pendingAutoRestMode = -1;
uint32_t rearmCount=0, blockedUpSamples=0, restRejectedWindows=0;
uint8_t upBlockReason=0;
static constexpr float STABLE_ACCEL_SPAN_G = 0.040f; // guards slow tilts hidden by variance
float restMinA[3]={},restMaxA[3]={};
volatile uint32_t autoRestCount = 0;
volatile uint32_t autoRestLastMs = 0;
uint32_t autoRestStartMs = 0;
uint16_t autoRestN = 0;
float autoRestSumA[3] = {0,0,0};
float autoRestSumSqA[3] = {0,0,0};
float autoRestSumW[3] = {0,0,0};
float autoRestSumSqW[3] = {0,0,0};

void resetAutoRestWindow() {
  autoRestStartMs = 0;
  autoRestN = 0;
  for (int i=0;i<3;++i) {
    autoRestSumA[i]=autoRestSumSqA[i]=0;
    autoRestSumW[i]=autoRestSumSqW[i]=0;
  }
}

void applyAutoRest(const float meanA[3], const float meanW[3], uint32_t nowMs) {
  // Reanclar el vector vertical usando la gravedad MEDIDA durante reposo.
  // No usar un offset XYZ que borre tambien aceleraciones de juego.
  const float n = length3(meanA);
  if (n < AUTO_REST_MIN_G || n > AUTO_REST_MAX_G) return;
  for (int i=0;i<3;++i) {
    verticalUnit[i] = meanA[i] / n;
    gyroBias[i] = meanW[i];
  }
  gravityNorm = n;
  // Conservamos el plano X/Y y corregimos su ortogonalidad a la nueva Z.
  if (axisReady) {
    // Si volvimos a la postura de montaje calibrada, restaura la referencia
    // anatomica X desde NVS. Esto NO corrige yaw absoluto en cualquier pose.
    if (savedMapValid && dot3(savedMap.neutralZ,verticalUnit)>0.975f) {
      for (int i=0;i<3;++i) bodyXsensor[i]=savedMap.neutralX[i];
    }
    orthonormalizeBodyAxes();
  }
  // Reinicio SOLO de filtros dinamicos, no de contadores ni configuracion.
  for (int i=0;i<3;++i) fGyro[i]=0;
  fActivity=0; fUp=0; shortV=0;
  if (det.moving) pushEvent(2,nowMs,0,0,0); // MOV_OFF hacia Unity
  det.moving=false;
  det.onCandidateSince=0; det.releaseSince=0;
  det.upArmed=true; det.downSeen=false; det.upEvidence=0;
  ++autoRestCount;
  autoRestLastMs=nowMs;
  if (!serialPlotMode) Serial.printf("AUTO_REST OK: %lu, gyroBias=%.2f/%.2f/%.2f, grav=%.4fg, XYZ conservados=%u\n",
    (unsigned long)autoRestCount,gyroBias[0],gyroBias[1],gyroBias[2],gravityNorm,axisReady?1U:0U);
}

void observeAutoRest(const ImuSample &s, uint32_t ms) {
  if (!autoRestEnabled || !calibrated) {resetAutoRestWindow(); return;}
  float w[3];
  for (int i=0;i<3;++i) w[i]=s.w[i]-gyroBias[i];
  const float normA=length3(s.a);
  const bool grossStationary =
    length3(w)<AUTO_REST_GYRO_DPS &&
    normA>=AUTO_REST_MIN_G && normA<=AUTO_REST_MAX_G;
  if (!grossStationary) {
    resetAutoRestWindow();
    return;
  }
  // La correccion se repite tras cada nueva ventana de reposo estable.
  // Asi no se acumula deriva mientras la placa permanece inmovil.
  if (autoRestN==0) autoRestStartMs=ms;
  if (autoRestN==0) for(int i=0;i<3;++i) restMinA[i]=restMaxA[i]=s.a[i];
  ++autoRestN;
  for (int i=0;i<3;++i) {
    restMinA[i]=fminf(restMinA[i],s.a[i]); restMaxA[i]=fmaxf(restMaxA[i],s.a[i]);
    autoRestSumA[i]+=s.a[i];
    autoRestSumSqA[i]+=s.a[i]*s.a[i];
    autoRestSumW[i]+=s.w[i];
    autoRestSumSqW[i]+=s.w[i]*s.w[i];
  }
  if ((uint32_t)(ms-autoRestStartMs)<AUTO_REST_MS || autoRestN<100) return;
  float meanA[3],meanW[3];
  bool steady=true;
  for (int i=0;i<3;++i) {
    meanA[i]=autoRestSumA[i]/autoRestN;
    meanW[i]=autoRestSumW[i]/autoRestN;
    float va=fmaxf(0.0f,autoRestSumSqA[i]/autoRestN-meanA[i]*meanA[i]);
    float vw=fmaxf(0.0f,autoRestSumSqW[i]/autoRestN-meanW[i]*meanW[i]);
    if (restMaxA[i]-restMinA[i]>STABLE_ACCEL_SPAN_G ||
        va>AUTO_REST_STD_G*AUTO_REST_STD_G ||
        vw>AUTO_REST_STD_W*AUTO_REST_STD_W) steady=false;
  }
  // Evitar confundir un giro lento y constante con reposo absoluto.
  float meanWResidual[3];
  for (int i=0;i<3;++i) meanWResidual[i]=meanW[i]-gyroBias[i];
  if (length3(meanWResidual)>2.5f) steady=false;
  resetAutoRestWindow();
  if (steady) {
    applyAutoRest(meanA,meanW,ms);
  } else ++restRejectedWindows;
}

volatile bool calibrating = true;
uint32_t calStartMs = 0;
uint16_t calCount = 0;
float calA[3] = {0,0,0}, calW[3] = {0,0,0};
float calMinA[3]={},calMaxA[3]={},calSqA[3]={},calSqW[3]={};

void beginCalibration(uint32_t nowMs) {
  calibrating = true;
  calibrated = false;
  calStartMs = nowMs;
  calCount = 0;
  for (int i=0; i<3; ++i) { calA[i]=0; calW[i]=0; }
  det = MotionDetector();
  resetAutoRestWindow();
  axisReady=false; axisTiltReady=false; axisCaptureMode=0; axisRequest=0;
  finishTest();
  shortV = 0; fActivity = 0; fUp = 0;
  for (float &v : fGyro) v = 0;
  if (!serialPlotMode) Serial.println("CAL: deja la placa inmovil y el pie apoyado (~1 s).");
}
void calibrationSample(const ImuSample &s, uint32_t nowMs) {
  const float aNorm = length3(s.a), wNorm = length3(s.w);
  if (fabsf(aNorm - 1.0f) <= 0.14f && wNorm < 12.0f) {
    if (calCount==0) for(int i=0;i<3;++i) {
      calMinA[i]=calMaxA[i]=s.a[i]; calSqA[i]=calSqW[i]=0;
    }
    for (int i=0; i<3; ++i) {
      calA[i]+=s.a[i]; calW[i]+=s.w[i];
      calSqA[i]+=s.a[i]*s.a[i]; calSqW[i]+=s.w[i]*s.w[i];
      calMinA[i]=fminf(calMinA[i],s.a[i]); calMaxA[i]=fmaxf(calMaxA[i],s.a[i]);
    }
    ++calCount;
  } else {
    calCount=0;
    for (int i=0; i<3; ++i) { calA[i]=0; calW[i]=0; }
  }
  if (calCount >= 170) {
    // Norm and gyro magnitude alone also accept a rotating sensor.
    bool stable=true;
    for(int i=0;i<3;++i) {
      float ma=calA[i]/calCount,mw=calW[i]/calCount;
      if(calMaxA[i]-calMinA[i]>STABLE_ACCEL_SPAN_G ||
         calSqA[i]/calCount-ma*ma>AUTO_REST_STD_G*AUTO_REST_STD_G ||
         calSqW[i]/calCount-mw*mw>AUTO_REST_STD_W*AUTO_REST_STD_W) stable=false;
    }
    if(!stable) {
      calCount=0; for(int i=0;i<3;++i) calA[i]=calW[i]=0;
      return;
    }
    for (int i=0; i<3; ++i) {
      gyroBias[i] = calW[i] / calCount;
      verticalUnit[i] = calA[i] / calCount;
    }
    gravityNorm = length3(verticalUnit);
    unit3(verticalUnit);
    for (int i=0;i<3;++i) baseZneutral[i]=verticalUnit[i];
    calibrated = true; calibrating = false;
    shortV=0; fActivity=0; fUp=0;
    if (!serialPlotMode) Serial.printf("CAL OK: g=%.3f, verticalSensor=(%.3f, %.3f, %.3f)\n",
      gravityNorm, verticalUnit[0],verticalUnit[1],verticalUnit[2]);
    if (!serialPlotMode) Serial.printf("GYRO BIAS: %.2f, %.2f, %.2f dps\n",
      gyroBias[0],gyroBias[1],gyroBias[2]);
    tryRestoreSavedAxisMap();
  } else if ((uint32_t)(nowMs - calStartMs) > 12000) {
    calStartMs = nowMs;
    if (!serialPlotMode) Serial.println("CAL: esperando quietud estable; reintentando...");
  }
}

void snapshotDetectorDiagnostics(LiveData &d,uint32_t ms) {
  d.restCount=(uint16_t)min((uint32_t)autoRestCount,(uint32_t)65535);
  d.disarmedMs=det.upArmed ? 0 : (uint16_t)min((uint32_t)(ms-det.lastUpMs),(uint32_t)65535);
  d.biasNormX100=(uint16_t)min((uint32_t)lroundf(length3(gyroBias)*100),(uint32_t)65535);
  d.gravityMg=(uint16_t)lroundf(gravityNorm*1000);
  d.velocityMmS=toI16(shortV*1000); d.calSamples=calCount;
  d.rearms=(uint16_t)min(rearmCount,(uint32_t)65535);
  d.blockedSamples=(uint16_t)min(blockedUpSamples,(uint32_t)65535);
  d.restRejected=(uint16_t)min(restRejectedWindows,(uint32_t)65535);
  d.upBlockReason=upBlockReason; d.autoEnabled=autoRestEnabled?1:0;
}
void processSample(const ImuSample &s, float dt) {
  const uint32_t ms = s.ms;
  portENTER_CRITICAL(&dataMux);
  int8_t autoMode=pendingAutoRestMode; pendingAutoRestMode=-1;
  portEXIT_CRITICAL(&dataMux);
  if (autoMode>=0) {
    autoRestEnabled=autoMode==1;
    resetAutoRestWindow();
  }
  if (requestCalibration) {
    requestCalibration = false;
    beginCalibration(ms);
  }
  if (calibrating) {
    calibrationSample(s, ms);
    LiveData d = liveSnapshot();
    d.ms = ms; d.flags = 0x80 | 0x08 | (calibrating ? 0x10 : 0x04) |
      (!calibrating && det.upArmed ? 0x40:0) | (axisReady?0x20:0);
    d.imuHz = imuRateHz;
    snapshotDetectorDiagnostics(d,ms);
    setLive(d);
    return;
  }
  axisCaptureStep(s);
  // Auto calibracion 2 s de QUIETUD FISICA, usando IMU cruda.
  // Independiente de MOV, fUp y orientacion previamente calculada.
  observeAutoRest(s, ms);
  if (pendingTest) {
    uint8_t mode=pendingTest; pendingTest=0;
    startTest(mode,ms);
  }
  const RuntimeConfig c = configSnapshot();
  const float tau = 1.0f/(6.283185307f*c.filterHz);
  const float alpha = dt/(tau + dt);
  const float aNorm = length3(s.a);
  float w[3];
  for (int i=0; i<3; ++i) {
    w[i] = s.w[i] - gyroBias[i];
    fGyro[i] += alpha * (w[i] - fGyro[i]);
  }
  const float gyro = length3(fGyro);
  const float activity = fabsf(aNorm - gravityNorm);
  fActivity += alpha * (activity - fActivity);

  // Vectores fijos en el cuerpo referidos al marco local del chip:
  // u_dot = -(omega_S x u). No supone que ax/ay/az sean X/Y/Z corporales.
  rotateWorldVectorInSensor(verticalUnit,w,dt);
  if (axisReady) rotateWorldVectorInSensor(bodyXsensor,w,dt);

  const float up = s.a[0]*verticalUnit[0] + s.a[1]*verticalUnit[1] +
                   s.a[2]*verticalUnit[2] - gravityNorm;
  fUp += alpha*(up-fUp);
  shortV = constrain(shortV*expf(-dt/0.40f) + fUp*G_MS2*dt,
                     -0.65f,0.65f);
  const bool quiet = gyro < c.movOffDps && fActivity < c.movOffG &&
                     fabsf(fUp) < fmaxf(0.030f,c.movOffG*1.8f);

  // No corregimos la vertical usando fUp o det.moving: esos valores
  // pueden estar erroneos precisamente cuando hay deriva de gravedad.
  // observeAutoRest() la corrige tras reposo crudo estable por 2 s.
  if (axisReady) orthonormalizeBodyAxes();
  if (quiet && !det.moving) shortV=0.0f;
  // Aceleracion lineal ANATOMICA en g, solo valida con AXIS OK.
  float bodyAcc[3]={NAN,NAN,fUp}, bodyGyro[3]={NAN,NAN,NAN};
  if (axisReady) {
    float aLinearS[3];
    for(int i=0;i<3;++i)
      aLinearS[i]=s.a[i]-gravityNorm*verticalUnit[i];
    bodyAcc[0]=dot3(aLinearS,bodyXsensor);
    bodyAcc[1]=dot3(aLinearS,bodyYsensor);
    bodyAcc[2]=dot3(aLinearS,verticalUnit);
    bodyGyro[0]=dot3(w,bodyXsensor);
    bodyGyro[1]=dot3(w,bodyYsensor);
    bodyGyro[2]=dot3(w,verticalUnit);
  }
  if (activeTest) {
    if (axisReady) {
      ++testN;
      for (int i=0;i<3;++i) {
        const float a=fabsf(bodyAcc[i]);
        testSumAbs[i]+=a;
        testPeak[i]=fmaxf(testPeak[i],a);
        testSumAbsGyro[i]+=fabsf(bodyGyro[i]);
      }
    }
    if ((uint32_t)(ms-testLastCsvMs)>=40) {
      testLastCsvMs=ms;
      if (!serialPlotMode) Serial.printf("B,%lu,%s,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%+.3f,%.3f,%d,%u,%u\n",
        (unsigned long)ms,testName(activeTest),s.a[0],s.a[1],s.a[2],
        s.w[0],s.w[1],s.w[2],bodyAcc[0],bodyAcc[1],bodyAcc[2],
        bodyGyro[0],bodyGyro[1],bodyGyro[2],fUp,fActivity,
        det.moving ? 1 : 0, axisReady ? 1U : 0U,(unsigned)imuRateHz);
    }
    if ((uint32_t)(ms-testStartMs)>=12000) finishTest();
  }

  // MOV = actividad general (no equivalencia fisica con contacto del pie).
  const bool motionCandidate = gyro >= c.movOnDps ||
    fActivity >= c.movOnG || fUp >= c.upOnG;
  if (!det.moving) {
    if (motionCandidate) {
      if (det.onCandidateSince == 0) det.onCandidateSince = ms;
      if ((uint32_t)(ms - det.onCandidateSince) >= c.confirmMs) {
        det.moving=true; det.releaseSince=0;
        det.onCandidateSince=0; ++det.moves;
        pushEvent(1,ms,gyro,fUp,0x01);
      }
    } else det.onCandidateSince=0;
  } else {
    if (quiet) {
      if (det.releaseSince==0) det.releaseSince=ms;
      if ((uint32_t)(ms-det.releaseSince) >= c.releaseMs) {
        det.moving=false; det.releaseSince=0;
        pushEvent(2,ms,gyro,fUp,0x00);
      }
    } else det.releaseSince=0;
  }

  // UP = candidato a impulso ascendente (no altura ni salto confirmado).
  if (!det.upArmed) {
    if (fUp < -c.upOnG*0.55f) det.downSeen=true;
    if ((uint32_t)(ms-det.lastUpMs) >= c.upGapMs &&
        (quiet || (det.downSeen && fUp < c.upOnG*0.35f))) {
      det.upArmed = true; det.downSeen = false; det.upEvidence = 0; ++rearmCount;
    }
  }
  const bool upCandidate = fUp >= c.upOnG &&
    fActivity >= c.upActG && shortV >= c.upVms;
  // Bitmask of current blockers; blockedSamples counts samples, NOT gestures.
  upBlockReason=(fUp<c.upOnG?1:0)|(fActivity<c.upActG?2:0)|(shortV<c.upVms?4:0)|
    (!det.upArmed?8:0)|(!det.upArmed && (uint32_t)(ms-det.lastUpMs)<c.upGapMs?16:0);
  if(upCandidate && !det.upArmed) ++blockedUpSamples;
  if (det.upArmed) {
    det.upEvidence = upCandidate ? (uint8_t)min(3,(int)det.upEvidence+1) : 0;
    if (det.upEvidence >= 2) {
      det.upArmed=false; det.upEvidence=0; det.downSeen=false;
      det.lastUpMs=ms; det.lastUpEventMs=ms; ++det.ups;
      pushEvent(3,ms,gyro,fUp,0x03);
    }
  }

  const float scaledMotion = fmaxf(gyro/c.movOnDps, fActivity/c.movOnG);
  LiveData d = {};
  d.ms = ms;
  d.rawAxMg=toI16(s.a[0]*1000.0f); d.rawAyMg=toI16(s.a[1]*1000.0f);
  d.rawAzMg=toI16(s.a[2]*1000.0f);
  d.rawGxX10=toI16(s.w[0]*10); d.rawGyX10=toI16(s.w[1]*10);
  d.rawGzX10=toI16(s.w[2]*10);
  d.upMg=toI16(fUp*1000); d.gyroX10=toI16(gyro*10);
  d.activityMg=toI16(fActivity*1000);
  d.bodyAxMg=axisReady ? toI16(bodyAcc[0]*1000):0;
  d.bodyAyMg=axisReady ? toI16(bodyAcc[1]*1000):0;
  d.bodyAzMg=axisReady ? toI16(bodyAcc[2]*1000):0;
  d.bodyGxX10=axisReady ? toI16(bodyGyro[0]*10):0;
  d.bodyGyX10=axisReady ? toI16(bodyGyro[1]*10):0;
  d.bodyGzX10=axisReady ? toI16(bodyGyro[2]*10):0;
  d.imuHz=imuRateHz;
  d.motionCount=det.moves; d.upCount=det.ups;
  d.intensity=(uint8_t)constrain((int)(scaledMotion*50),0,100);
  d.flags=(det.moving ? 0x01 : 0) |
    ((uint32_t)(ms-det.lastUpEventMs)<130 && det.ups>0 ? 0x02:0) |
    0x04 | 0x08 | 0x80 | (det.upArmed ? 0x40 : 0) | (axisReady ? 0x20 : 0);
  snapshotDetectorDiagnostics(d,ms);
  setLive(d);
}

TaskHandle_t imuTaskHandle = nullptr;
volatile uint32_t drdyCount = 0;
void IRAM_ATTR imuIRQ() {
  ++drdyCount;
  if (imuTaskHandle) {
    BaseType_t wake=pdFALSE;
    vTaskNotifyGiveFromISR(imuTaskHandle,&wake);
    if (wake) portYIELD_FROM_ISR();
  }
}
void imuTask(void*) {
  beginCalibration(millis());
  ImuSample s;
  uint32_t lastUs=0;
  uint32_t lastCsvMs=0;
  for (;;) {
    // DRDY del QMI8658, con polling de respaldo si no hay interrupcion.
    ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(3));
    if (!readSample(s)) continue;
    ++imuSamples;
    const uint32_t ms=s.ms;
    float dt=1.0f/ODR_HZ;
    if (lastUs!=0) {
      dt=(uint32_t)(s.us-lastUs)*1e-6f;
      if (dt < 0.001f || dt > 0.060f) dt=1.0f/ODR_HZ;
    }
    lastUs=s.us;
    if (lastRateTime==0) lastRateTime=ms;
    if ((uint32_t)(ms-lastRateTime)>=1000) {
      imuRateHz=(uint16_t)((imuSamples-lastRateCount)*1000UL/(ms-lastRateTime));
      lastRateTime=ms; lastRateCount=imuSamples;
    }
    processSample(s,dt);
    if (serialCsv && (uint32_t)(ms-lastCsvMs)>=100) {
      lastCsvMs=ms;
      const LiveData d=liveSnapshot();
      if (!serialPlotMode) Serial.printf("S,%lu,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%+.3f,%.3f,%d,%u\n",
        (unsigned long)ms,s.a[0],s.a[1],s.a[2],s.w[0],s.w[1],s.w[2],
        d.upMg/1000.0f,d.activityMg/1000.0f,(d.flags&1)?1:0,d.imuHz);
    }
  }
}

// ===================== CONFIGURACION DESDE UNITY ======================
// ASCII UTF8 por characteristic CONFIG (READ/WRITE/NOTIFY):
// SET,MOV_ON_DPS,24   GET,MOV_ON_DPS   GET   SAVE   DEFAULT
// CAL  STREAM,1   LOG,1   PING,15
// Las confirmaciones no exceden 20 bytes (MTU BLE predeterminado).
void replyConfig(const String &msg) {
  if (!serialPlotMode) Serial.print("CFG: "); if (!serialPlotMode) Serial.println(msg);
  if (chrConfig) {
    chrConfig->setValue(msg);
    if (bleConnected) chrConfig->notify();
  }
}
String configValue(const RuntimeConfig &c, const String &key) {
  if (key=="MOV_ON_DPS") return String(c.movOnDps,1);
  if (key=="MOV_OFF_DPS") return String(c.movOffDps,1);
  if (key=="MOV_ON_G") return String(c.movOnG,3);
  if (key=="MOV_OFF_G") return String(c.movOffG,3);
  if (key=="UP_ON_G") return String(c.upOnG,3);
  if (key=="UP_ACT_G") return String(c.upActG,3);
  if (key=="UP_V_MS") return String(c.upVms,3);
  if (key=="FILTER_HZ") return String(c.filterHz,1);
  if (key=="CONFIRM_MS") return String(c.confirmMs);
  if (key=="RELEASE_MS") return String(c.releaseMs);
  if (key=="UP_GAP_MS") return String(c.upGapMs);
  return "";
}
void runCommand(String command) {
  command.trim(); command.toUpperCase();
  if (command.length()==0) return;
  if (command.startsWith("PLOT,RATE,")) {
    // 3 Hz=~16.7s, 5 Hz=~10s, 10 Hz=~5s, 25 Hz=~2s.
    const String arg=command.substring(10);
    const int hz=arg.toInt();
    if ((hz==3 || hz==5 || hz==10 || hz==25) && arg==String(hz)) {
      plotEveryMs=(uint16_t)(1000 / hz);
      prefs.putUChar("plotHz", (uint8_t)hz);
      if (!serialPlotMode) Serial.printf("OK,PLOT,RATE,%dHz\n",hz);
    } else if (!serialPlotMode) Serial.println("ERR,PLOT,RATE (usar 3,5,10 o 25)");
    return;
  }
  if (command=="PLOT,1") {
    serialCsv=false;
    // Antes de habilitar, dejamos el resto del puerto para las curvas solamente.
    serialPlotMode=true;
    // El modo Plotter vuelve a activarse en cada arranque (v2.2.4).
    return;
  }
  if (command=="PLOT,0") {
    serialPlotMode=false;
    // PLOT,0 solo cambia el modo de esta sesion; al reiniciar vuelve a Plotter.
    replyConfig("OK,PLOT,0");
    return;
  }
  if (command=="AXIS,TILT") {
    if (!calibrated) { replyConfig("ERR,CAL"); return; }
    axisRequest=1; replyConfig("OK,AXIS,TILT"); return;
  }
  if (command=="AXIS,APPLY") {
    if (!axisTiltReady) { replyConfig("ERR,AXIS,TILT"); return; }
    axisRequest=2; replyConfig("OK,AXIS,APPLY"); return;
  }
  if (command=="AXIS,RESET") {
    prefs.remove("axisMap"); savedMapValid=false;
    axisRequest=3; replyConfig("OK,AXIS,RESET"); return;
  }
  if (command.startsWith("AXIS,YAW,")) {
    // Ajuste angular incremental limitado a +/-30 grados por comando.
    if (!axisReady) { replyConfig("ERR,AXIS,NO_CAL"); return; }
    const String arg=command.substring(9);
    char *end=nullptr;
    const float degrees=strtof(arg.c_str(),&end);
    if (end==arg.c_str() || *end!='\0' || !isfinite(degrees) ||
        degrees < -30.0f || degrees > 30.0f || fabsf(degrees)<0.01f) {
      replyConfig("ERR,AXIS,YAW,RANGE(-30..30)"); return;
    }
    pendingYawTenths=(int16_t)roundf(degrees*10.0f);
    axisRequest=4;
    replyConfig("OK,AXIS,YAW,PENDIENTE");
    return;
  }
  if (command=="AUTO,1" || command=="AUTO,0") {
    portENTER_CRITICAL(&dataMux);
    pendingAutoRestMode=command.endsWith("1")?1:0;
    portEXIT_CRITICAL(&dataMux);
    replyConfig(command.endsWith("1")?"OK,AUTO,1":"OK,AUTO,0"); return;
  }
  if (command=="AUTO,STAT") {
    if (!serialPlotMode) Serial.printf("AUTO REST: %s, reposo=%lums, correcciones=%lu, ultima=%lums\n",
      autoRestEnabled?"ON":"OFF",(unsigned long)AUTO_REST_MS,
      (unsigned long)autoRestCount,(unsigned long)autoRestLastMs);
    replyConfig("OK,AUTO,STAT"); return;
  }
  if (command=="AXIS,SHOW" || command=="STAT") {
    const LiveData d=liveSnapshot();
    if (!serialPlotMode) Serial.printf("AXIS STATE: cal=%u tilt=%u xyz=%u; Xs=(%.3f,%.3f,%.3f); Ys=(%.3f,%.3f,%.3f); Zs=(%.3f,%.3f,%.3f)\n",
      calibrated?1U:0U,axisTiltReady?1U:0U,axisReady?1U:0U,
      bodyXsensor[0],bodyXsensor[1],bodyXsensor[2],
      bodyYsensor[0],bodyYsensor[1],bodyYsensor[2],
      verticalUnit[0],verticalUnit[1],verticalUnit[2]);
    if (!serialPlotMode) Serial.printf("STAT: imu=%uHz, mov=%u, bx=%+.3fg, by=%+.3fg, bz=%+.3fg, i2cerr=%lu\n",
      (unsigned)d.imuHz,(d.flags&1)?1U:0U,
      d.bodyAxMg/1000.0f,d.bodyAyMg/1000.0f,d.bodyAzMg/1000.0f,
      (unsigned long)i2cErrors);
    if (command=="AXIS,SHOW") replyConfig("OK,AXIS,SHOW");
    return;
  }
  if (command.startsWith("TEST,")) {
    String mode=command.substring(5);
    uint8_t t=mode=="REST"?1:mode=="UP"?2:mode=="X"?3:mode=="Y"?4:mode=="ROT"?5:mode=="STOP"?6:0;
    if (!t) { replyConfig("ERR,TEST"); return; }
    if (!calibrated) { replyConfig("ERR,CAL"); return; }
    if (!axisReady && t!=6) { replyConfig("ERR,AXIS"); return; }
    pendingTest=t; replyConfig("OK,TEST,"+mode); return;
  }
  if (command=="AXSTREAM,1" || command=="AXSTREAM,0") {
    streamAxes=command.endsWith("1");
    replyConfig(streamAxes?"OK,AXSTREAM,1":"OK,AXSTREAM,0"); return;
  }
  if (command=="SAVE") { saveConfig(); replyConfig("OK,SAVE"); return; }
  if (command=="DEFAULT") {
    setConfig(defaults()); replyConfig("OK,DEFAULT"); return;
  }
  if (command=="CAL") {
    requestCalibration=true; replyConfig("OK,CAL"); return;
  }
  if (command=="GET") { replyConfig("OK,GET_USE_KEY"); return; }
  if (command.startsWith("GET,")) {
    const String key=command.substring(4);
    const String value=configValue(configSnapshot(),key);
    replyConfig(value.length()?"VAL,"+key+","+value:"ERR,KEY");
    return;
  }
  if (command.startsWith("PING,")) {
    // Unity puede estimar desfase de reloj con RTT y esta respuesta.
    String token=command.substring(5);
    if (token.length()>4) { replyConfig("ERR,PING"); return; }
    replyConfig("PONG,"+token+","+String(millis()));
    return;
  }
  if (command=="STREAM,1" || command=="STREAM,0") {
    streamRaw=command.endsWith("1");
    replyConfig(streamRaw?"OK,STREAM,1":"OK,STREAM,0"); return;
  }
  if (command=="LOG,1" || command=="LOG,0") {
    serialCsv=command.endsWith("1");
    replyConfig(serialCsv?"OK,LOG,1":"OK,LOG,0"); return;
  }
  if (!command.startsWith("SET,")) { replyConfig("ERR,CMD"); return; }
  const int comma=command.indexOf(',',4);
  if (comma<0) { replyConfig("ERR,FORMAT"); return; }
  const String key=command.substring(4,comma);
  const String value=command.substring(comma+1);
  if (value.length()==0 || value.length()>14) { replyConfig("ERR,VALUE"); return; }
  char *endPtr=nullptr;
  const float f=strtof(value.c_str(),&endPtr);
  if (endPtr==value.c_str() || *endPtr!='\0' || !isfinite(f)) {
    replyConfig("ERR,NUMBER"); return;
  }
  RuntimeConfig c=configSnapshot();
  if      (key=="MOV_ON_DPS")  c.movOnDps=f;
  else if (key=="MOV_OFF_DPS") c.movOffDps=f;
  else if (key=="MOV_ON_G")    c.movOnG=f;
  else if (key=="MOV_OFF_G")   c.movOffG=f;
  else if (key=="UP_ON_G")     c.upOnG=f;
  else if (key=="UP_ACT_G")    c.upActG=f;
  else if (key=="UP_V_MS")     c.upVms=f;
  else if (key=="FILTER_HZ")   c.filterHz=f;
  else if (key=="CONFIRM_MS")  {
    if (f<4 || f>100 || floorf(f)!=f) { replyConfig("ERR,RANGE"); return; }
    c.confirmMs=(uint16_t)f;
  }
  else if (key=="RELEASE_MS")  {
    if (f<40 || f>550 || floorf(f)!=f) { replyConfig("ERR,RANGE"); return; }
    c.releaseMs=(uint16_t)f;
  }
  else if (key=="UP_GAP_MS") {
    if (f<120 || f>900 || floorf(f)!=f) { replyConfig("ERR,RANGE"); return; }
    c.upGapMs=(uint16_t)f;
  }
  else { replyConfig("ERR,KEY"); return; }
  if (!validConfig(c)) { replyConfig("ERR,RANGE"); return; }
  setConfig(c);
  replyConfig("OK,"+key+","+configValue(c,key));
}

// ========================== PANTALLA ================================
void drawStaticScreen() {
  if (!lcdReady) return;
  lcd->fillScreen(C_BG);
  lcd->fillRect(0,0,320,38,C_NAVY);
  lcd->setTextWrap(false);
  lcd->setTextColor(C_WHITE); lcd->setTextSize(2);
  lcd->setCursor(10,5); lcd->print("BIOFIT GAME");
  lcd->setTextSize(1); lcd->setCursor(10,27);
  lcd->print(TRACKER_LABEL); lcd->print("  v2.2.7");
  lcd->drawFastHLine(0,130,320,C_BLUE);
}
void drawDynamicScreen() {
  if (!lcdReady) return;
  static uint32_t oldMoves=0xFFFFFFFF, oldUps=0xFFFFFFFF;
  static uint8_t oldFlags=255;
  static uint16_t oldHz=0xFFFF;
  static bool oldBle=true;
  static uint32_t oldAutoCount=0xFFFFFFFF;
  const LiveData d=liveSnapshot();
  const bool moving=(d.flags&0x01)!=0;
  const bool cal=(d.flags&0x10)!=0;
  if (d.motionCount!=oldMoves) {
    lcd->fillRect(92,45,80,22,C_BG);
    lcd->setTextSize(2); lcd->setTextColor(C_GREEN);
    lcd->setCursor(92,45); lcd->print(d.motionCount);
    oldMoves=d.motionCount;
  }
  if (d.upCount!=oldUps) {
    lcd->fillRect(243,45,75,22,C_BG);
    lcd->setTextSize(2); lcd->setTextColor(C_BLUE);
    lcd->setCursor(243,45); lcd->print(d.upCount);
    oldUps=d.upCount;
  }
  if (oldFlags != (d.flags & 0x11)) {
    lcd->fillRect(3,79,312,17,C_BG);
    lcd->setTextSize(2);
    lcd->setTextColor(cal?C_ORANGE:(moving?C_GREEN:C_GRAY));
    lcd->setCursor(12,79);
    lcd->print(cal?"CALIBRANDO":(moving?"MOVIMIENTO":"QUIETO"));
    oldFlags=(d.flags & 0x11);
  }
  // Diagnostico XYZ corporal. La LCD no mide desplazamiento ni altura.
  lcd->fillRect(8,103,303,18,C_BG);
  lcd->setTextSize(1); lcd->setTextColor(C_WHITE);
  lcd->setCursor(8,104);
  if (d.flags&0x20) lcd->printf("aX:%+.2f aY:%+.2f aZ:%+.2f g",
      d.bodyAxMg/1000.0f,d.bodyAyMg/1000.0f,d.bodyAzMg/1000.0f);
  else lcd->print("XYZ: SIN CALIBRAR | Z estimada");
  if (oldBle!=bleConnected || oldHz!=d.imuHz || oldAutoCount!=autoRestCount) {
    lcd->fillRect(8,141,303,17,C_BG);
    lcd->setTextSize(1); lcd->setTextColor(C_GRAY);
    lcd->setCursor(8,142); lcd->printf("BLE:%s IMU:%uHz AR:%lu",
      bleConnected?"OK":"ESP", (unsigned)d.imuHz,(unsigned long)autoRestCount);
    oldBle=bleConnected; oldHz=d.imuHz; oldAutoCount=autoRestCount;
  }
  static bool labelsDrawn=false;
  if (!labelsDrawn) {
    lcd->setTextSize(2);
    lcd->setTextColor(C_WHITE); lcd->setCursor(10,45); lcd->print("MOV");
    lcd->setCursor(182,45); lcd->print("UP");
    labelsDrawn=true;
  }
}

// =========================== MAIN ===================================
void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0); // USB ausente/lento no debe frenar adquisicion
#endif
  delay(350);
  // Leer NVS antes de imprimir: si se abre el Plotter y ocurre un reset,
  // ninguna linea de texto no numerico contamina las curvas.
  loadConfig();
  loadSavedAxisMap();
  if (!serialPlotMode) Serial.printf("\nBIOFIT GAME v2.2.7 XYZ+PLOT | ID=%d | %s\n", TRACKER_ID, TRACKER_NAME);
  if (!serialPlotMode) Serial.println("MOV es movimiento estimado. UP es candidato, NO altura/salto confirmado.");

  eventQueue=xQueueCreate(32,sizeof(EventPacket));
  commandQueue=xQueueCreate(12,sizeof(CommandPacket));
  if (!eventQueue || !commandQueue) {
    if (!serialPlotMode) Serial.println("FATAL: no se pudo crear la cola FreeRTOS");
    while(true) delay(1000);
  }
  pinMode(LCD_BL,OUTPUT); digitalWrite(LCD_BL,HIGH);
  lcdReady=lcd->begin(40000000);
  if (lcdReady) drawStaticScreen();
  else if (!serialPlotMode) Serial.println("LCD: inicio fallido (IMU y BLE continuan)");

  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  if (!beginIMU()) {
    if (!serialPlotMode) Serial.println("FATAL IMU: WHO_AM_I QMI8658 no encontrado (0x6A/0x6B)");
    if (lcdReady) {
      lcd->setTextSize(2); lcd->setTextColor(C_ORANGE);
      lcd->setCursor(8,76); lcd->print("ERROR IMU");
    }
    while(true) delay(1000);
  }
  imuOk=true;
  if (!serialPlotMode) Serial.printf("QMI8658: 0x%02X, ODR nominal 224.2Hz, +/-8g, +/-512dps\n",imuAddr);
  startBLE();
  xTaskCreatePinnedToCore(imuTask,"BIOFIT_IMU",8192,nullptr,5,&imuTaskHandle,1);
  if (xTaskCreatePinnedToCore(transportTask,"BIOFIT_BLE_TX",6144,nullptr,2,nullptr,1)!=pdPASS) {
    if (!serialPlotMode) Serial.println("FATAL: tarea BLE TX");
    while(true) delay(1000);
  }
  pinMode(IMU_INT2,INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_INT2),imuIRQ,RISING);
  if (!serialPlotMode) Serial.println("Serial: CAL, AXIS,TILT, AXIS,APPLY, AXIS,SHOW, TEST,REST, TEST,UP, TEST,X, TEST,Y, TEST,ROT");
  if (!serialPlotMode) Serial.println("Config: GET,MOV_ON_DPS / SET,MOV_ON_DPS,30 / STREAM,1 / AXSTREAM,1 / SAVE / LOG,1");
  if (!serialPlotMode) Serial.println("CSV LOG: S,ms,ax,ay,az,gx,gy,gz,up_g,act_g,mov,Hz");
}

uint32_t lastStatus=0, lastRaw=0, lastAxes=0, lastScreen=0, lastPlot=0;
uint8_t prevConnected=0;
char serialBuffer[64];
uint8_t serialPos=0;

void loop() {
  // BLE callback no bloquea: encolar las ordenes y ejecutarlas aqui.
  CommandPacket c;
  uint8_t commands=0;
  while (commands<4 && xQueueReceive(commandQueue,&c,0)==pdTRUE) {
    runCommand(String(c.text)); ++commands;
  }
  while (Serial.available()) {
    char ch=(char)Serial.read();
    if (ch=='\r' || ch=='\n') {
      if (serialPos) {
        serialBuffer[serialPos]='\0';
        runCommand(String(serialBuffer));
        serialPos=0;
      }
    } else if (serialPos<sizeof(serialBuffer)-1) serialBuffer[serialPos++]=ch;
    else serialPos=0;
  }
  const uint32_t now=millis();
  // No escritura Serial desde la IMU al usar PLOT; graficar desde loop.
  // Antes de AXIS OK, no rotulamos datos crudos como anatomicos:
  // las tres lineas permanecen en cero hasta calibrar la transformacion.
  if (serialPlotMode && Serial.availableForWrite()>=64 && (uint32_t)(now-lastPlot)>=plotEveryMs) {
    lastPlot=now;
    const LiveData p=liveSnapshot();
    if ((p.flags & 0x20) && (p.flags & 0x04)) {
      // IMPORTANTE IDE 2.3.x: exactamente 3 pares etiqueta:numero.
      // Sin signo '+' obligatorio y sin campos adicionales.
      Serial.printf("Eje_X:%.3f,Eje_Y:%.3f,Eje_Z:%.3f\n",
        p.bodyAxMg/1000.0f, p.bodyAyMg/1000.0f, p.bodyAzMg/1000.0f);
    } else {
      Serial.println("Eje_X:0.000,Eje_Y:0.000,Eje_Z:0.000");
    }
  }
  saveAxisMapInMainLoop();
  const bool connected=bleConnected;
  if (prevConnected && !connected) {
    BLEDevice::startAdvertising();
    if (!serialPlotMode) Serial.println("BLE: desconectado, anuncios reactivados");
  }
  prevConnected=connected;

  if ((uint32_t)(now-lastScreen)>=SCREEN_EVERY_MS) {
    lastScreen=now;
    drawDynamicScreen();
  }
  delay(2);
}

void transportTask(void*) {
  for (;;) {
  // Unico propietario de EVENT/STATUS/RAW/AXES/TRANSPORT. Sin LCD, NVS ni Serial.
  const uint32_t now=millis();
  const bool connected=bleConnected;
  static uint32_t lastPass=0, lastTransport=0;
  if (lastPass) {
    const uint16_t gap=capped16((uint32_t)(now-lastPass));
    if (gap>maxTransportGapMs) maxTransportGapMs=gap;
  }
  lastPass=now;
  const UBaseType_t depth=uxQueueMessagesWaiting(eventQueue);
  if (depth>queueHighWater) queueHighWater=(uint16_t)depth;
  EventPacket evt;
  uint8_t drained=0;
  while (drained<8 && xQueueReceive(eventQueue,&evt,0)==pdTRUE) {
    if (connected && chrEvents && eventsCccd->getNotifications()) {
      const uint16_t age=capped16((uint32_t)(millis()-evt.queuedMs));
      if (age>maxQueueAgeMs) maxQueueAgeMs=age;
      evt.b[3] |= 0x80; // V2 extension: bytes 14..15 = espera en cola firmware, ms
      put16(evt.b+14,age);
      chrEvents->setValue(evt.b,sizeof(evt.b));
      timedNotify(chrEvents);
    }
    ++drained;
  }
  const LiveData d=liveSnapshot();
  if ((uint32_t)(now-lastStatus)>=STATUS_EVERY_MS) {
    lastStatus=now;
    uint8_t p[20] = {};
    p[0]=2; p[1]=TRACKER_ID; p[2]=d.flags; p[3]=d.intensity;
    put16(p+4,++statusSeq); put32(p+6,d.ms);
    put16(p+10,(uint16_t)d.upMg); put16(p+12,(uint16_t)d.gyroX10);
    put16(p+14,(uint16_t)d.activityMg); put16(p+16,d.imuHz);
    put16(p+18,(uint16_t)(i2cErrors & 0xFFFF));
    chrStatus->setValue(p,sizeof(p));
    if (connected) timedNotify(chrStatus);
  }
  if (streamRaw && (uint32_t)(now-lastRaw)>=RAW_EVERY_MS) {
    lastRaw=now;
    uint8_t p[20] = {};
    p[0]=2; p[1]=TRACKER_ID; put16(p+2,++rawSeq); put32(p+4,d.ms);
    put16(p+8,(uint16_t)d.rawAxMg); put16(p+10,(uint16_t)d.rawAyMg);
    put16(p+12,(uint16_t)d.rawAzMg);
    put16(p+14,(uint16_t)d.rawGxX10); put16(p+16,(uint16_t)d.rawGyX10);
    put16(p+18,(uint16_t)d.rawGzX10);
    chrRaw->setValue(p,sizeof(p));
    if (connected) timedNotify(chrRaw);
  }
  if (streamAxes && (uint32_t)(now-lastAxes)>=AXES_EVERY_MS) {
    lastAxes=now;
    uint8_t p[20]={};
    p[0]=2; p[1]=TRACKER_ID; put16(p+2,++axisSeq);
    put32(p+4,d.ms);
    put16(p+8,(uint16_t)d.bodyAxMg); put16(p+10,(uint16_t)d.bodyAyMg);
    put16(p+12,(uint16_t)d.bodyAzMg); put16(p+14,(uint16_t)d.bodyGxX10);
    put16(p+16,(uint16_t)d.bodyGyX10); put16(p+18,(uint16_t)d.bodyGzX10);
    chrAxes->setValue(p,sizeof(p));
    if (connected && (d.flags&0x20)) timedNotify(chrAxes);
  }
  if (connected && (uint32_t)(now-lastTransport)>=1000) {
    lastTransport=now;
    uint8_t p[20]={1,TRACKER_ID};
    put16(p+2,++transportSeq); put32(p+4,millis());
    put16(p+8,queueHighWater); put16(p+10,capped16(droppedEvents));
    put16(p+12,maxQueueAgeMs); put16(p+14,maxNotifyMs);
    put16(p+16,maxTransportGapMs); put16(p+18,capped16(notifyErrors));
    chrTransport->setValue(p,sizeof(p)); timedNotify(chrTransport);
    // V2 diagnostic extension, <=20 bytes. Existing V1 transport stays intact.
    // State at 1Hz; one configuration/counter page per second (cycle 3s).
    uint8_t q[20]={2,TRACKER_ID,0,d.flags}; put32(q+4,d.ms);
    put16(q+8,d.restCount); put16(q+10,d.disarmedMs);
    put16(q+12,(uint16_t)d.upMg); put16(q+14,(uint16_t)d.velocityMmS);
    put16(q+16,d.biasNormX100); put16(q+18,d.gravityMg);
    chrTransport->setValue(q,sizeof(q)); timedNotify(chrTransport);
    static uint8_t page=1;
    const RuntimeConfig c=configSnapshot();
    q[2]=page; q[3]=d.autoEnabled;
    if(page==1) {
      put16(q+8,toI16(c.upOnG*1000)); put16(q+10,toI16(c.upActG*1000));
      put16(q+12,toI16(c.upVms*1000)); put16(q+14,c.upGapMs);
      put16(q+16,toI16(c.movOffG*1000)); put16(q+18,toI16(c.filterHz*100));
    } else if(page==2) {
      put16(q+8,toI16(c.movOnDps*10)); put16(q+10,toI16(c.movOffDps*10));
      put16(q+12,toI16(c.movOnG*1000)); put16(q+14,c.confirmMs);
      put16(q+16,c.releaseMs); put16(q+18,AUTO_REST_MS);
    } else {
      put16(q+8,d.rearms); put16(q+10,d.upBlockReason); put16(q+12,d.blockedSamples);
      put16(q+14,d.calSamples); put16(q+16,d.restRejected); put16(q+18,227);
    }
    chrTransport->setValue(q,sizeof(q)); timedNotify(chrTransport);
    page=page==3?1:page+1;
  }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
