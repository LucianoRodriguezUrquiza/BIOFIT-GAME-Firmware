# BIOFIT GAME — Universal Tracker V2.2.9

**Estado vigente (26/11/2026): V2.2.9, versión estable de trabajo aprobada por el usuario y autorizada para consolidación en main.** V2.2.5 queda obsoleta como referencia de trabajo; se conserva exclusivamente su historial y su uso como referencia de regresión. No se han modificado los sketches ni sus cabeceras respecto del código V2.2.9 publicado en `274625f605ea92189abfb16275012f34ee14371f`.

- Compilación ESP32 de ambos roles y pruebas host: [GitHub Actions 44444687178](https://github.com/LucianoRodriguezUrquiza/BIOFIT-GAME-Firmware/actions/runs/44444687178), 3/3 jobs correctos.
- Aceptación del usuario: versión de referencia actual para los trackers de BIOFIT DASH V6.5. No equivale a validación física exhaustiva, confirmación independiente de binarios/NVS instalados o certificación del canal experimental LIFT_ONSET.
- Control principal: BLE BILATERAL LEGACY, UP_IMPULSE. LIFT_ONSET sigue siendo un canal experimental separado, no sustituye LEGACY.
- Pendiente: diagnosticar la degradación de la detección tras varios saltos y recuperación después de 20–30 s de reposo. No se atribuye causa todavía ni se modifica el algoritmo en esta consolidación.
- [Contexto actual](docs/CONTEXTO_ACTUAL.md), [relevo vivo](docs/CONTEXTO_VIVO.md), [validación y consolidación V2.2.9](docs/CONSOLIDACION_V229.md), [revisión técnica Lift Cycle](docs/REVISION_V229_CICLO.md).
- Descargar la rama main completa y conservar ambos archivos .h junto al .ino de cada tobillo. No reprogramar para diagnosticar el fallo sin identificar primero versión efectiva, parámetros NVS y estado de los trackers.


## Hardware y utilización

Firmware para dos trackers corporales **Waveshare ESP32-S3-LCD-1.47B** con IMU **QMI8658**, uno en cada tobillo. Forma parte del proyecto **BIOFIT GAME**, una plataforma de exergaming que utilizará un smartphone Android como consola y recibirá datos de sensores mediante Bluetooth Low Energy (BLE).

## Estado del proyecto

- Versión indicada por ambos códigos: **Universal Tracker v2.2.9**.
- Revisión de estabilidad, pruebas y sesión consolidada: [REVISION_V08](docs/REVISION_V08.md).
- Los dos sketch son iguales salvo por `#define TRACKER_ID`.
- Los algoritmos actuales identifican actividad `MOV_ON` / `MOV_OFF` y un candidato `UP_IMPULSE`; **no confirman por sí solos pasos biomecánicos ni saltos**. Unity BIOFIT DASH combina la información de ambos tobillos para decidir acciones bilaterales.
- La compilación y regresión host se verificaron en el commit indicado arriba; los ensayos físicos independientes siguen pendientes.

## Estructura

```text
Firmware/
├── Tobillo_Izquierdo/
│   └── BIOFIT_TOBILLO_IZQUIERDO_BASE/
│       └── BIOFIT_TOBILLO_IZQUIERDO_BASE.ino
└── Tobillo_Derecho/
    └── BIOFIT_TOBILLO_DERECHO_BASE/
        └── BIOFIT_TOBILLO_DERECHO_BASE.ino
```

Históricamente, la revisión 2.2.7 conserva el transporte de 2.2.6 y corrige estabilidad de calibración/auto-reposo y rearme. Conserva NVS y mapa anatómico; los sketch mantienen su carpeta homónima requerida por Arduino IDE.

## Dispositivos

| Dispositivo | `TRACKER_ID` | Nombre BLE |
|---|---:|---|
| Tobillo izquierdo | `1` | `BIOFIT-LEFT-01` |
| Tobillo derecho | `2` | `BIOFIT-RIGHT-02` |

El bloque para seleccionar cada rol se encuentra cerca del comienzo del respectivo `.ino`.

## Entorno indicado en el firmware

- Placa: **Waveshare ESP32-S3-LCD-1.47B**, sin pantalla táctil.
- Núcleo: **Arduino-ESP32 3.3.x**.
- Biblioteca adicional indicada en los archivos: **GFX Library for Arduino** (`Arduino_GFX_Library.h`).
- Otras dependencias utilizadas: `Arduino.h`, `Wire.h`, `BLEDevice.h`, `BLEServer.h`, `BLEUtils.h`, `BLE2902.h`, `Preferences.h` y cabeceras estándar de C/C++.
- Monitor serie: **115200 baudios**.

## Cómo abrir cada firmware en Arduino IDE

1. Descargar o clonar este repositorio.
2. Abrir Arduino IDE y elegir **Archivo → Abrir**.
3. Para el pie izquierdo, seleccionar `Firmware/Tobillo_Izquierdo/BIOFIT_TOBILLO_IZQUIERDO_BASE/BIOFIT_TOBILLO_IZQUIERDO_BASE.ino`.
4. Para el pie derecho, seleccionar `Firmware/Tobillo_Derecho/BIOFIT_TOBILLO_DERECHO_BASE/BIOFIT_TOBILLO_DERECHO_BASE.ino`.
5. Seleccionar la placa compatible, el puerto y las opciones de compilación adecuadas para la Waveshare; comprobar y subir el sketch correspondiente a **cada** placa.
6. Verificar la identidad BLE y las lecturas del QMI8658 antes de las pruebas de movimiento.

## Funciones presentes en el código

- Adquisición de acelerómetro y giroscopio del QMI8658.
- Presentación de estado en LCD ST7789.
- Comunicación BLE de eventos, estado, configuración y datos opcionales de IMU (`RAW`, `AXES`).
- Calibración y mapeo de ejes mediante comandos como `CAL`, `AXIS,TILT`, `AXIS,APPLY` y `AXIS,SHOW`.
- Autoajuste en reposo y almacenamiento de parámetros de configuración en NVS.
- Salida de 3 curvas por el Serial Plotter (`Eje_X`, `Eje_Y` y `Eje_Z`), tras la calibración correspondiente.

**Nota técnica:** los ejes internos del QMI8658 no deben identificarse directamente con los ejes anatómicos. La relación se verifica experimentalmente. La orientación corporal de referencia de BIOFIT GAME es +X hacia adelante, +Y hacia la izquierda y +Z hacia arriba.

## Montaje físico de referencia

Ambas placas se instalan sobre el lado **izquierdo anatómico** de cada pierna, con la pantalla mirando hacia la izquierda corporal y el puerto **USB-C hacia atrás**, en dirección al talón. Los dispositivos no van montados en espejo.

## Próximos pasos

Diagnosticar el problema de detección sostenida sin modificar firmware ni NVS hasta obtener evidencia. El proyecto Unity/Android se mantiene en [BIOFIT-GAME-UNITY](https://github.com/LucianoRodriguezUrquiza/BIOFIT-GAME-UNITY), versión V6.5.

## Licencia

No se agregó ninguna licencia. Elegir una licencia explícita antes de autorizar la reutilización por terceros.

## Historial — revisión de transporte 2.2.6

La rama `fix/ble-latency-consolidated` separa TX BLE de LCD/USB/NVS,
reduce STATUS a 10 Hz, corrige los timestamps a `millis()` y añade diagnóstico
TRANSPORT opcional a 1 Hz. EVENT continúa V2/16 bytes: bit 7 de flags indica
que bytes 14–15 contienen espera en cola firmware en ms. La aplicación anterior
ignora esos bytes; la revisión Unity coordinada los usa para impedir acciones antiguas.
No se modificaron funciones de calibración, mapa anatómico, NVS o clasificación.

[Diagnóstico, protocolo y sesión única](https://github.com/LucianoRodriguezUrquiza/BIOFIT-GAME-UNITY/blob/fix/ble-latency-consolidated/docs/LATENCIA_REVISION_CONSOLIDADA.md).

Validación de fuentes: `python tests/check_contract.py`. El workflow
`ESP32 firmware validation` además compila ambos roles con core 3.3.5.
Su resultado debe verificarse: un control estático aprobado no es compilación,
y una compilación aprobada no es ensayo físico. No se declara resuelta la latencia.

Compilación confirmada de ambos roles: [Actions 43765893874](https://github.com/LucianoRodriguezUrquiza/BIOFIT-GAME-Firmware/actions/runs/43765893874), commit de código `9b3afb7ef69d96f0f4bdc8bb99ecaf883ac4c583`, core 3.3.5 y GFX 1.6.9. Esto no valida funcionamiento físico ni latencia movimiento-pantalla.
