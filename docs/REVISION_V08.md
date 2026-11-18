# BIOFIT DASH v0.8 / firmware v2.2.7 — revisión consolidada

## Procedencia y límite de evidencia

Unity parte de `fix/ui-unilateral-v07`, `865a85cc578b264daf2517075e6c6d805aea4683`. Firmware parte de `fix/ble-latency-consolidated`, `f52b188de1b401befdb3b68a048a4659ed592566` (v2.2.6 publicada). Ambas revisiones están en ramas nuevas; no se fusiona ni modifica la versión funcional.

**Discrepancia de adjuntos:** los archivos recibidos `BIOFIT_TOBILLO_*_BASE(3).ino` contienen v2.2.5 y coinciden con `main` `9bb4183db5d2bfd76bc0d9aeefd203d0543b315d` tras normalizar finales de línea. No son v2.2.6. SHA256 del adjunto izquierdo: `6a2beba93204b3136dddb69e69ece555eb1c1d7c2cc7d6acd22e50be178f536a`; derecho: `9b0899c6b5b6e87bec7e84f34f2eb649958b581b75ade1c964b7e8936b27378b`.

La v2.2.6 publicada ya separa TX/IMU, usa reloj monotónico, STATUS de 100 ms y métricas de transporte; no se reemplaza con los adjuntos antiguos. El registro experimental contiene esas métricas, pero no permite certificar el binario cargado. `CSV(1).pdf` se corta durante el tramo izquierdo: hay 52 EV completos: 24 UP izquierdos, 13 MOV_ON/13 MOV_OFF izquierdos y 1 MOV_ON/1 MOV_OFF derecho. No hay UP derechos ni pares detallados suficientes; no reconstruirlos.

## Diagnóstico y correcciones

| Síntoma | Causa / evidencia | Función y acción |
|---|---|---|
| Primeros gestos perdidos | Causalidad física no demostrada. CAL original acepta una inclinación sintética de 8°/s; reloj alineado no significa calibración lista. | `calibrationSample`: exige estabilidad de pose/varianza; `BiofitTrackerReadiness`: CAL/ARM independientes del reloj. No bloquear eventos por ARM actual: el propio impulso lo desarma. |
| Recuperación tras quietud | Fallo reproducido: con residual +0.033 g se cumple QUIETO pero no el antiguo rearme (<0.0315 g). | `processSample`: QUIETO permite rearme después de upGap; descenso mantiene umbral anterior. Nunca rearme por tiempo solamente. |
| Degradación tras movimiento | `observeAutoRest` confundía una inclinación de 2°/s durante 2 s con reposo (replay: 1 corrección antes, 0 después). No prueba que sea la causa del ensayo humano. | Límite de recorrido por eje 0.040 g además de varianza; conserva ventana de 2 s y reanclaje. AUTO se aplica en tarea IMU, con petición protegida. |
| MOV persistente | Gyro integrado puede desviar la gravedad durante movimiento sostenido. Deriva inducida 0.5°/s por 60 s produce offset >0.05 g; reposo estable lo recupera. | Se instrumenta bias/gravedad/rearmado. **La deriva durante movimiento no queda eliminada.** LCD MOV procede de `det.moving`; gráfica fluctuante no equivale a MOV_ON. |
| Derecho menos eficaz | Captura: 20/30 UP derechos bajo .35 g frente a 9/24 izquierdos; son candidatos, no intentos humanos. Faltan muestras derechas completas. | Umbrales independientes, ambos .35 g por defecto; selector IZQ/DER. No bajar sensibilidad automáticamente. Rechazos unilaterales antes omitidos ahora quedan en CSV. |
| Latencia / cadencia | Resumen v0.7: qMax2, drop0, edadMax3 ms, notifyMax1 ms, ciclo4 ms, ~10Hz, 2 eventos comprimidos; no demuestra latencia física total. | Conserva TX y filtro de eventos antiguos. Añade callback→decisión y despacho→decisión. 260 ms upGap, 320 ms anti unilateral y vuelo ~0.74 s impiden exigir cuatro saltos visuales/s. |

## Compatibilidad y observabilidad

EVENT sigue en 16 bytes, STATUS en 20, UUID e IDs intactos. STATUS bit7 indica extensión ARM, bit6 armado; con v2.2.6 se muestra ARM:? (desconocido). Las banderas de CAL/IMU/ejes/MOV existentes no cambian. STATUS congelado/ausente >1 s no se presenta como estado actual. Se conserva v1 de TRANSPORT; Unity anterior ignora las páginas v2 nuevas. No se cambia el bilateral .12 g /180 ms, calibración anatómica, NVS, Serial Plotter ni comandos existentes.

TRANSPORT v2 utiliza el mismo UUID ...0007, 20 bytes little endian: versión/id/página/flags en 0..3, millis en 4..7 y seis campos de 16 bits en 8..19. Se añaden dos notificaciones/s por tracker: estado a 1Hz y una página rotativa de configuración/contadores (ciclo 3s).

| Página | Campos en orden |
|---|---|
| 0 | restCount, disarmedMs, upMg **signed**, velocityMmS **signed**, biasNorm×100, gravityMg; flags STATUS |
| 1 | upOnMg, upActMg, upVmmS, upGapMs, movOffMg, filterHz×100 |
| 2 | movOnDps×10, movOffDps×10, movOnMg, confirmMs, releaseMs, restWindowMs |
| 3 | rearms, blockMask, blockedSamples, calSamples, restRejected, fwVersion=227 |

Páginas 1..3: byte3=AUTO habilitado. Contadores saturan a 65535; blockedSamples cuenta muestras, no gestos. blockMask: 1 aceleración baja, 2 actividad baja, 4 velocidad baja, 8 desarmado,16 cooldown. Son instantáneas de diagnóstico, no trazas de cada muestra. Los cambios de banderas se registran a la tasa STATUS; eventos ARM breves pueden no observarse. Comparar configuración efectiva de ambos lados; valores NVS personalizados se conservan.

FINALIZAR guarda un CSV UTF8 completo en almacenamiento de la app; ARCHIVO abre el selector Android para guardar una copia, sin permisos generales de almacenamiento. COPIAR permanece como alternativa. Archivo con hasta 6000 filas, conteos total/EV, descartes y SHA256 al final. El hash detecta truncamiento/cambio de bytes, **no recupera filas descartadas por límite**. No convertir a PDF. Cancelar el selector conserva el original. Java compilar no equivale a validar el selector en Android.

## Verificación reproducible

- Firmware: `python3 tests/check_contract.py`; `python3 tests/host/run_replay.py --baseline`; `python3 tests/host/run_replay.py`. El runner extrae y compila el código real `.ino` con g++; solo IO se simula. Casos: CAL en movimiento/estable, primeros4gestos, auto-rest repetido/inclinación, rearme residual,90gestos en120s, cadencias .25/.5/1/2s (5/10/10/10 UP de10), deriva/recuperación, configuración distinta/NaN, rollover.
- Unity host: `dotnet run --project tests/LatencyRegression` (58+9+31 comprobaciones); `tests/ReceiverRegression` (25); `tests/DashUiRegression` (48). Total 171. Incluye receptor real con shims, límites de cola, eventos antiguos/duplicados, desconexión, protocolo nuevo y antiguo, CSV grande/alterado/truncado, umbrales independientes, botones reales OnGUI y geometría en4resoluciones.
- CI añade compilación Arduino de ambas placas (ESP32 core3.3.5, GFX1.6.9) y javac contra Android35. Ver estado real en PR antes de integrar.
- **No ejecutados aquí:** Unity6000.3.28f1/Play/render, Gradle/IL2CPP/APK, selector Android, BLE real, montaje, NVS físico, latencia física ni porcentaje de aciertos. Las pruebas host no sustituyen esas categorías.

## Próxima sesión única (después de CI y revisión Editor)

1. GitHub Desktop: preservar cambios locales; Fetch origin; seleccionar `fix/stability-warmup-v08` en Unity y `fix/imu-state-rest-v227` en firmware; Pull solo si se ofrece. No cambiar bases ni fusionar PRs para probar.
2. Abrir Unity6000.3.28f1. Menú BIOFIT: ejecutar verificaciones DASH, Input Engine y salto bilateral; crear/abrir diagnóstico BLE y BIOFIT DASH si faltan escenas. Play horizontal: salto manual, pausa/reinicio, panel oculto, SIN COLISION, selector IZQ/DER y captura FINALIZAR/ARCHIVO/COPIAR. Consola sin errores. Estos controles no requieren APK.
3. En Build Profiles activo: `DiagnosticoBLE` primera, `BiofitDash` habilitada, `SampleScene` deshabilitada. Si el perfil sobreescribe escenas globales, corregir su lista también. Android, Export Project y Build App Bundle desactivados. Una sola APK `BIOFIT_DASH_v08.apk` después de Play; anotar SHA de código y obtener hash con PowerShell `Get-FileHash .\BIOFIT_DASH_v08.apk -Algorithm SHA256`. No existe APK/hash de binario entregado en esta revisión.
4. Para ensayar los arreglos de detector se necesita v2.2.7 en ambas placas: compilar/cargar izquierdo ID1 primero y derecho ID2 después, identificándolos físicamente. Mantener placa/particionado y **Erase All Flash desactivado**; no ejecutar reset/defaults ni comandos de nueva calibración anatómica. Antes, Monitor Serie a 115200, `PLOT,0`, `AXIS,SHOW`, `AUTO,STAT` y `GET,UP_ON_G`, `GET,UP_ACT_G`, `GET,UP_V_MS`, `GET,UP_GAP_MS`, `GET,MOV_OFF_G`; guardar respuestas de cada placa. Después repetir y comparar. Estos comandos no borran NVS. Además, si se dispone del procedimiento habitual, respaldar flash/NVS usando la tabla de particiones real (no adivinar offsets). Después comprobar versión en LCD, IDs, ejes y configuración. El parche no escribe NVS automáticamente ni modifica sus claves; preservación física pendiente.
5. Montaje firme, USB-C atrás. Abrir diagnóstico/conectar2/2 y DASH; SIN COLISION:SI. Mantener .35g por pie y .12g/180ms bilateral. INICIAR; QUIETO20s. Registrar CAL por pie, ARM y reloj por separado. CAL debe terminar; si ARM:? se está usando firmware antiguo o falta STATUS. No provocar gestos durante CAL.
6. ELEVACION: modo SOLO IZQ,3 intentos contados en voz alta a separación2s; SOLO DER,3; BILATERAL,3. Anotar los primeros3 de cada modo por separado. Luego10izquierdos y10derechos a2s, cambiando modo solo entre bloques. No tocar teléfono por gesto; el cambio de modo queda registrado.
7. DASH/BILATERAL:60s cómodos,30pares a2s. Si aparece caída, anotar segundo/conteo sin cambiar umbral; QUIETO10s; ELEVACION y3pares de reanudación. Luego5pares bilaterales a2s y QUIETO20s final. No exigir4saltos/s. La marcha, si se evalúa, es un bloque CAMINAR separado y contado, no mezclar con elevaciones.
8. FINALIZAR → ARCHIVO → elegir Descargas → Guardar. Conservar CSV original y captura de contadores; comprobar footer END_BIOFIT y filas/descartes. Entregar archivo sin convertir junto con conteo físico por bloque y observación LCD MOV vs gráfica. Video lento opcional para movimiento→respuesta visual; timestamps solos no miden esa latencia total.

Siguiente decisión: alinear primeros fallos y caída de respuesta con CAL/ARM, restCount, bias/gravedad, configuración, EV y decisiones del juego. Si no se generan UP, investigar señal/montaje/estimador; si llegan y se rechazan, usar motivo; si se aceptan y no saltan, revisar ocupado/estado del avatar. No atribuir causalidad ni >95% de eficacia hasta esa sesión.
