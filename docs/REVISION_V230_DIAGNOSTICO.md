# V2.3.0 EXP — Biofit Motion Diagnostics (firmware)

Fecha 26/11/2026. Rama `feat/diagnostics-v230`. Base aprobada main V2.2.9 `7afb20df8ace2b4c61153474649d31a36040dee1`. La aceptación de V2.2.9 no prueba la causa de la pérdida sostenida de saltos. Autorización del usuario: desarrollar un firmware experimental configurable sin reflashear por cada hipótesis. Unity V6.5 estable permanece intacto; **la interfaz Android para estos nuevos comandos todavía no está implementada**. El contrato siguiente permite implementarla en una rama separada.

## Perfiles RAM-only
Arranque mask=0: conserva las condiciones de UP legacy V2.2.9 y eventos tipo 3/4, MOV, UUID, formato EVENT/STATUS/RAW, configuración NVS, AutoRest y ejes. La identificación de firmware pasa a 230 (LCD/Serial/TRANSPORT). Unity V6.5 LEGACY admite el protocolo; su modo experimental CICLO exige explícitamente 229, por tanto no se afirma compatibilidad con CICLO hasta actualizar Unity. No cambia cooldown, ventana bilateral ni físicas.

| Bit | Máscara | Cambio aislado |
|---|---:|---|
| 0 | 1 | Piso de emisión UP = max(UP_ON_G efectivo, 0.120g). No falsea valor ni timestamp, no baja umbral Unity; puede demorar o perder candidatos débiles. Rearme usa configuración base. |
| 1 | 2 | Rearme adicional solo tras intervalo individual y 60ms continuos de valle de baja energía: |fUp|≤0.025g, actividad≤0.055g, gyro≤20dps, -0.08≤shortV≤0.015m/s. Nunca rearma solo por tiempo. |
| 2 | 4 | Vertical paralela, integración gyro existente más corrección normalizada tau=0.8s condicionada a norma, giro, actividad, innovación y dt. Solo cambia proyección usada por detector; no escribe mapa anatómico ni sesgo. Aceleración dinámica puede contaminar incluso condiciones de baja energía; validar antes de aceptar. |
| 3 | 8 | Aísla transmisión LIFT_ONSET tipo 4 sin eliminar observación local Lift Cycle; prueba interferencia de tráfico/secuencia, no aísla coste CPU. |
| todos | 15 | Combinado: no utilizar como primer ensayo causal. |

Bits combinables. Cambio aceptado solo después de 250ms de baja energía, vence a 10s sin quietud, no reordena eventos ni fuerza salto; al aplicar reinicia fUp/shortV/evidencia y referencia paralela sin tocar contadores ni forzar armado. Solicitudes con token nuevo (1..65535), una pendiente por tracker, duplicado idempotente; si venció reenviar con token nuevo. Al reiniciar vuelve a mask=0; no escribe NVS. Perfil activo persiste en RAM al desconectar; verificar al reconectar. No cambiar perfiles durante juego: pausar, solicitar ambos, esperar ambos aplicados con token correcto, comprobar coincidencia, invalidar estados del detector Unity y reanudar tras comprobar relojes. El cambio no es transacción atómica entre dos conexiones; si uno falla no jugar hasta volver ambos a 0 o reiniciar.

## Protocolo BLE CONFIG
UUID f9710000-6d68-4b7d-a742-6fae7b5f0004, UTF-8, comandos ≤20 bytes; escribir con respuesta, luego leer/recibir notificación CONFIG; serializar solicitudes por tracker y verificar respuesta, timeout/reintento. No confundir D,Q (recibido en loop) con aplicado por IMU. No enviar SAVE/DEFAULT/CAL para experimentar.

- D,GET → P,active,target,token,state (0 aplicado, 1 pendiente, 2 vencido).
- D,P,mask,token → D,Q,token o ERR,D,PROFILE/FORMAT.
- D,T,1 → T,state,count,generation, inicia trace si no está grabando; repetición no reinicia.
- D,T,0 → T,state,count,generation, congela sin copiar.
- D,T → mismo estado (0 vacío, 1 grabando, 2 congelado).
- D,R,generation,index,page → respuesta binaria 20 bytes en CONFIG (read/notify); ERR,D,TRACE si no congelado, generación o índice incorrecto. Páginas 0..2. Solo descargar después del ensayo, secuencialmente, validar id/generación/índice/página. No enviar como cadena; decodificar bytes. Si respuesta se pierde, repetir lectura. Una nueva captura invalida generación anterior.

Trace de 1024 muestras en RAM, ~4.6s nominales a 224Hz, sobrescribe las más antiguas. Captura solo durante adquisición normal (no CAL); dtUs es dt efectivo usado por filtro, no tiempo bruto de hueco. Congelar rápidamente al fallo; no cubre toda la sesión: CSV normal de Unity sigue siendo necesario. No notifica automáticamente ni amplía RAW. Indexación cronológica desde 0; generación 16 bits, 0 reservado.

Cabecera binaria: [0]=0xD3, [1]=id, [2]=page, [3]=schema1, [4..5]=generation u16, [6..7]=index u16. Little endian:
- p0: ms u32 @8, fUp mg i16 @12, shortV mm/s i16 @14, actividad mg i16 @16, gyro 0.1dps i16 @18.
- p1: raw ax,ay,az mg y gx,gy,gz 0.1dps, seis i16 @8.
- p2: flags @8 (STATUS original), blockMask @9 (bit0 umbral efectivo, bit1 actividad, bit2 integral, bit3 desarmado, bit4 gap), perfil @10, events @11 (bit0 UP, bit1 rearme, bit2 LIFT local, bit3 corrección gravedad, bit4 transición, bit5 rearme extra sin quiet original), proyección original mg i16 @12, paralela mg i16 @14, dtUs u16 @16; @18..19 reservado.
- No añadir BLE continuo: coste del trace es RAM y breve sección crítica en IMU. Medir IMU Hz y TX qMax/drop/age/notify antes y después. No afirmar coste cero.

## Validación y protocolo experimental
CI añade tests del helper (gramática, reintentos, estados, rollover, filtros, trace y codificación) y replay de processSample real extraído (perfiles aislados y combinado, 100 ciclos sintéticos, reversión). Mantiene pruebas anteriores, compilación separada ambos roles core 3.3.5/GFX 1.6.9. Consultar ejecución real; no declarar validación hasta que pase. No reemplaza binarios/NVS o prueba física. 
1. Registrar CSV actual 229/Unity LEGACY sin RAW, UP/AR y síntomas; guardar referencia.
2. Cargar 230 en ambas placas solo tras comprobar CI y disponer de cliente CONFIG; no borrar flash ni modificar NVS.
3. Registrar mask0, misma cadencia habitual, incluir reposo y acciones negativas (caminar, aterrizaje) sin forzar fatiga. Comprobar fwVersion230 y config.
4. Probar 1,2,4,8 por separado, en orden condicionado por primera pérdida de la cadena. No elegir por cantidad de UP sin etiquetas de movimiento. Evaluar verdaderos positivos, falsos positivos, pares aceptados, latencia, primer fallo y tiempo de recuperación. Luego combinar solo los perfiles respaldados.
5. Usar trace para investigar bloqueo local, comparar valor instantáneo/pico y deriva. Congelar al primer fallo y descargar después.
6. Si falla, reiniciar (perfil0), o volver al código V2.2.9 main, sin borrar NVS. No promover hasta aceptación explícita.

## Límites
No se implementa Shadow Mode de Unity, protocolo bilateral de confirmación desde UI ni clasificador nuevo; los perfiles reales operan de a uno por tracker, pueden interactuar. El piso 0.12 no soluciona por sí mismo un detector que no emite; alternativa posterior: detector sombra Unity con umbrales distintos. No se garantiza ausencia de errores o mejora física. NVS y firmware instalado no se han leído de las placas. Mantener ICC/DIPL/RFD/BRO/FLM/RCA.

### RCA: revisión preventiva de temporización
La primera ejecución 45912598811 detectó una advertencia de indentación convertida en error en el nuevo arnés C++; se corrige sin desactivar -Werror. Se refuerzan los dwell de cambio de perfil/rearme contra huecos de muestreo >30 ms, timestamps de muestra anteriores a la orden BLE y se exige giro crudo bajo para el cambio. La prueba de rearme extra verifica el bit de telemetría, no solo el estado armado. Nueva ejecución pendiente.
