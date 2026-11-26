# v2.2.9 EXP: canal de inicio de ciclo, separado de UP legacy

> Nota de vigencia (26/11/2026): V2.2.9 fue aprobada por el usuario como firmware estable de trabajo y autorizada para main. El canal LIFT_ONSET conserva su carácter experimental; no se afirma validación física exhaustiva. Las referencias a ramas y estados de esta revisión describen el desarrollo histórico. Ver [consolidación](CONSOLIDACION_V229.md).

Base: `feat/phase-evidence-v228` / `0947a509a32ae3b17390d20cfe21ba0f508d0d03`.
Rama: `feat/lift-cycle-v229`. Compañero: Unity `feat/reliable-consecutive-jumps-v0911`.
Estado: **algoritmo experimental, sin validación física**. No certifica contacto plantar ni fase biomecánica. No reemplaza el detector legacy.

## Motivo y decisión

Los CSV v0.9.10 confirman 59 comandos por sesión, pero cero pares rápidos y cero entradas al buffer. En la sesión FASE EXP, ningún par con ambos hints coincidió con el intervalo habilitante 600–900 ms. El hint v228 no impide que un UP normal nazca durante el frenado del descenso. Bajar solo el cooldown también admitiría ese segundo impulso.

Se agrega un canal causal independiente, `LIFT_ONSET`. Utiliza la proyección gravitatoria existente (acelerómetro + integración del giroscopio); **no supone que Z sea vertical**. El algoritmo procesa las mismas muestras a ~224 Hz. `shortV`, UP legacy, fase v228, MOV, filtros, autoreposo, ejes, NVS y parámetros guardados permanecen intactos.

## Máquina de estados, parámetros v1

`BiofitLiftCycle.h` es idéntico en ambos sketches:

1. `Unreferenced`: 160 ms con |up|≤0.05 g, gyro≤20 dps, actividad≤0.06 g y error de norma≤0.08 g; establece un sesgo local RAM. Esto es un anclaje inercial, no prueba de apoyo.
2. `Ready/Rising`: empieza a seguir al superar |a|=0.045 g. Emite **como máximo un evento** si a≥0.12 g, actividad≥0.035 g, integral corta≥0.020 m/s, persistencia≥8 ms y dentro de 160 ms desde el comienzo del seguimiento. Separación individual mínima 400 ms.
3. `Returning`: al acumular 0.004 g·s negativos con a<−0.06 g, veta emisiones positivas. Consume un lóbulo positivo de 0.004 g·s con a≥0.08 g antes de rearmar en su caída (a≤0.035 g durante 24 ms). No exige 80 ms sin giro ni que una integral con deriva vuelva a cero.
4. Una pausa estable de 60 ms y velocidad integrada pequeña también puede reanclar `Rising`. Una secuencia que empieza hacia abajo mantiene el veto durante el desplazamiento y el frenado.
5. Hueco >30 ms, muestra no finita/fuera de rango o ciclo >2500 ms invalidan la referencia: requiere otra ventana estable. CAL y autoreposo existentes invalidan solo el estado RAM nuevo. Contadores de diagnóstico sobreviven a esos resets, hasta reiniciar el MCU.

La integral corta ayuda al inicio y al diagnóstico; **no es velocidad física validada**. Los nombres Rising/Returning designan estados del controlador, no etiquetas humanas. La aceleración se limita a ±3 g para integrar, sin borrar historia por impactos válidos de 6–8 g observados en RAW. No cambia los valores originales enviados por RAW.

Estos parámetros son hipótesis de diseño, no calibraciones optimizadas con etiquetas humanas. Movimientos débiles/lentos, preparación negativa, impactos complejos, orientación con deriva y pérdida de historia pueden causar omisiones o falsos inicios. La pausa con poca IMU tampoco demuestra velocidad cero. No se afirma la imposibilidad universal de dobles disparos.

## Compatibilidad y presupuesto BLE

UUID y características existentes intactos. EVENT sigue V2/16 bytes, little endian:

| Campo | Valor nuevo |
|---|---|
| byte 2 | tipo 4 = `LIFT_ONSET` |
| byte 3 | bit 5 (`0x20`) identifica algoritmo v1; bit 7 sigue indicando edad TX |
| secuencia / millis | misma secuencia global EVENT y reloj original |
| vertical @12 | a corregida por sesgo RAM, mg; UP tipo 3 no cambia |

Unity nuevo pasa tipos 1–4 por la misma defensa de secuencia/edad. Solo CICLO EXP usa tipo 4 y requiere versión 229 de **ambos** sensores; LEGACY usa solo tipo 3. Firmware 227/228 con Unity nuevo mantiene LEGACY; CICLO no queda habilitado. Unity viejo ignora tipo 4 y puede contabilizar huecos de secuencia: para comparar usar **Unity nuevo + FW229 en ambos**, no una mezcla con APK vieja.

TRANSPORT V3, solo con `STREAM,1`, 20 bytes a 5 Hz por tracker:

| Offset | Campo |
|---|---|
| 0,1,2,3 | versión=3, ID, estado, razón |
| 4 u32 | millis sensor |
| 8,10 i16 | integral y mínimo, mm/s |
| 12,14,16 u16 | onsets, rearmes, fallos (contadores mod 65536) |
| 18 u16 | firmware=229 |

Razones 0–11: Anchoring, Armed, Tracking, Onset, ReturnVeto, RearmedQuiet, RearmedValley, DataGap, Horizon, Invalid, LateOrWeak, MinGap. El muestreo de diagnóstico a 5 Hz puede omitir transiciones breves; los contadores conservan los totales. No es un trace interno de 224 Hz.

Extra nominal de diagnóstico: 2×5×20=200 B/s de payload. EVENT4 agrega hasta 2×2.5×16=80 B/s bajo el guard de 400 ms entre onsets, mientras conserve historia. RAW permanece 2×20×20≈800 B/s. No incluye ATT, retransmisiones, resto de características ni negociación del central. Registrar TX qMax/drop/edad/notify/ciclo y demora/cola Unity para verificar el coste real; no se promete que sea cero.

## Pruebas reproducibles

```sh
python3 tests/check_contract.py
python3 tests/host/run_replay.py --baseline
python3 tests/host/run_replay.py
g++ -std=c++17 -Wall -Wextra -Werror tests/host/lift_cycle.cpp -o /tmp/lift && /tmp/lift
g++ -std=c++17 -Wall -Wextra -Werror tests/host/phase_evidence.cpp -o /tmp/phase && /tmp/phase
```

Resultados host: 30 ciclos balanceados sintéticos a 800 ms → 30 LIFT, sin eventos en el retorno; primer onset del helper a 20 ms con pasos de 5 ms. **No es latencia gesto-pantalla.** `processSample` real extraído del .ino, incluido su filtro: 30 LIFT frente a 31 UP legacy en esa secuencia. Pruebas adicionales: reposo, pausa elevada, descenso/frenado, impacto 6 g, espiga aislada, señal débil, gyro residual, hueco, rollover y valores inválidos; regresiones anteriores conservadas.

Replay aproximado opcional:

```sh
python3 tests/host/run_replay.py --raw-input /ruta/tracker.tsv
```

TSV: `sensor_ms ax ay az gx gy gz`, sin encabezado, un sensor. El script de Unity `tools/analyze_control_sessions.py --raw-tsv-dir /tmp/replay` lo genera sin editar los CSV. Interpola RAW 20 Hz a 5 ms, usa defaults y una calibración inicial reconstruida. **No recupera muestras faltantes ni NVS, sesgo/orientación/estado inicial reales.**

| Replay aproximado | UP original en CSV | UP recreado | LIFT nuevo |
|---|---:|---:|---:|
| A izquierdo | 124 | 120 | 43 |
| A derecho | 140 | 84 | 26 |
| B izquierdo | 127 | 105 | 28 |
| B derecho | 127 | 118 | 39 |

Esta pérdida de concordancia impide usar el replay como prueba de mejora; los recuentos bajos de LIFT advierten un **riesgo de sensibilidad** pendiente, no un resultado favorable. No se ajustaron parámetros para producir artificialmente 60 eventos. Un prototipo con rearme basado en velocidad integrada casi nula fue descartado por ser aún más restrictivo.

GitHub Actions compila **ambos sketches por separado** con core ESP32 3.3.5, GFX 1.6.9 y FQBN `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc`. Consultar el resultado real del PR; host no equivale a compilación ESP32. Ninguna prueba accede a hardware ni NVS.

## Preparación Arduino y reversión

1. En GitHub Desktop, repositorio Firmware → Fetch origin → Current branch → `feat/lift-cycle-v229`.
2. Conservar una copia de la carpeta/ajustes de compilación actuales. Abrir el .ino **IZQUIERDO** con sus dos .h contiguos. Core esp32 3.3.5 y biblioteca GFX Library for Arduino 1.6.9 reproducen CI.
3. Mantener placa ESP32S3 Dev Module y los valores de flash/particiones/PSRAM que ya funcionaban. USB Mode Hardware CDC and JTAG; USB CDC On Boot Enabled. **Erase All Flash Before Sketch Upload = Disabled**. No usar erase_flash, FACTORY ni comandos de calibración/ejes.
4. Conectar solo el tracker izquierdo, seleccionar su puerto y cargar. Repetir con el sketch DERECHO y el dispositivo derecho. Comprobar LCD `v2.2.9 EXP`, roles correctos. La calibración RAM normal del arranque sigue existiendo: dejar quietos al encender; no es borrado de NVS.
5. Para revertir firmware, elegir la rama `feat/phase-evidence-v228`, commit `0947a509a32ae3b17390d20cfe21ba0f508d0d03`, y recargar ambos lados con **mismo esquema de particiones y sin borrado total**. NVS conserva su formato. La recuperación inmediata del comportamiento de entrada se hace seleccionando BILATERAL LEGACY en la nueva APK.

Fuentes de procedimiento: [menú Arduino de Espressif](https://docs.espressif.com/projects/arduino-esp32/en/latest/guides/tools_menu.html), [conexión BLE ESP32-S3](https://docs.espressif.com/projects/esp-idf/en/release-v5.0/esp32s3/api-guides/ble/get-started/ble-connection.html). La comprobación física conjunta y el informe completo están en `docs/REVISION_V0911_CICLO.md` del repositorio Unity.
