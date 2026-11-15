# BIOFIT GAME — Firmware de tobillos

Firmware en desarrollo para dos trackers corporales **Waveshare ESP32-S3-LCD-1.47B** con IMU **QMI8658**, uno en cada tobillo. Forma parte del proyecto **BIOFIT GAME**, una plataforma de exergaming que utilizará un smartphone Android como consola y recibirá datos de sensores mediante Bluetooth Low Energy (BLE).

## Estado del proyecto

- Versión indicada por ambos códigos: **Universal Tracker v2.2.5**.
- Los dos sketch son iguales salvo por `#define TRACKER_ID`.
- Los algoritmos actuales identifican actividad `MOV_ON` / `MOV_OFF` y un candidato `UP_IMPULSE`; **no confirman por sí solos pasos biomecánicos ni saltos**. La futura aplicación Unity deberá combinar información de ambos tobillos para decidir `STEP`, `WALK`, `RUN` y `JUMP`.
- Este repositorio reúne los archivos fuente proporcionados; **no implica que hayan sido compilados o probados de nuevo**.

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

Se conservó **íntegro el contenido de ambos archivos originales**; solo se normalizaron sus nombres para que cada sketch `.ino` esté dentro de una carpeta con el mismo nombre, como exige Arduino IDE.

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

Validar señales y calibración en ambos tobillos, caracterizar los movimientos reales, comprobar comunicación BLE simultánea y luego construir el receptor Unity/Android. No se incluye todavía un proyecto Unity en este repositorio.

## Licencia

No se agregó ninguna licencia. Elegir una licencia explícita antes de autorizar la reutilización por terceros.

## Revisión de transporte 2.2.6

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
