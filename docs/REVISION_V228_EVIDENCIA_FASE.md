# Firmware v2.2.8 EXP — evidencia de nuevo ciclo, NO detector de contacto con suelo

**Base inalterada recuperable:** v2.2.7 commit `9cb5b7ac06315b0b657a33b8e608aca335eccb9e`. Rama nueva `feat/phase-evidence-v228`, PR #3 DRAFT. Ambos sketches izquierdo/derecho conservan mismo código salvo `TRACKER_ID`.

## Cambios mínimos
- Se agrega `BiofitPhaseEvidence.h` junto a cada sketch; fase de aceleración vertical negativa (`fUp<=-0.115g`), tiempo mínimo 340ms desde el último evento **calificado**, y reposo continuo>=80ms (`abs(fUp)<=.065g`, gyro<=23deg/s, actividad<=.075g y norma aceleración±.095g respecto gravedad).
- Tras esta secuencia, un `UP_IMPULSE` **original** de firmware v2.2.7 lleva `flags bit4=0x10` como indicador de ciclo aparentemente nuevo; en caso contrario sigue sin bit4. El evento que carece de evidencia **no** reinicia el rearme del helper.
- Los dos ESP32 siguen enviando el evento UP original al Unity receptor: ningún salto se elimina desde el firmware; el filtrado mejorado es opcional exclusivamente en Unity modo `BLE: FASE EXP`.
- Mantiene todos los IDs BLE, longitud del `EVENT` en 16 bytes, versión de protocolo V2, parámetros, NVS, filtros/compensación gravitatoria, sensor, tasa IMU, RAW y exportación. El display dice `v2.2.8 EXP` y la página3 `fwVersion=228`.
- `phaseEvidence.Reset` solo tras CAL o auto-rest de 2s; ***no*** se altera el detector `upArmed/downSeen` v2.2.7.
- El bit4 es evidencia probabilística, nunca indicador directo de pie apoyado ni prueba de Z anatómica. Especialmente, aceleración vertical positiva puede ocurrir al frenar un pie descendente.

## Validación
`tests/host/phase_evidence.cpp` compila el **mismo helper real** con C++17: arranque, continuidad de reposo, descartar falso rearme solo por aceleración positiva, giroscopio en movimiento, tiempo mínimo, rollover millis, interrupciones. `tests/check_contract.py` conserva contrato, NVS, páginas BLE y espejo sketches; `tests/host/run_replay.py` prueba funciones originales extraídas de las .ino. GitHub Actions compila los sketches con Arduino ESP32 core 3.3.5 + Arduino GFX 1.6.9.

**Estado de seguridad:** experimental. No flashear antes de CI verde y backup .ino y NVS; no afirmar que se han medido 0 falsos sin ensayo humano sincronizado. Los RAW previos submuestreados a 20Hz no pueden garantizar un algoritmo optimizado a 224Hz. Si falla, reflashear ambos sketches v2.2.7 sin formatear memoria NVS. Requisitos de app y prueba A/B en Unity PR #16, `docs/REVISION_V0910_FASES_BUFFER_180MS.md`.
