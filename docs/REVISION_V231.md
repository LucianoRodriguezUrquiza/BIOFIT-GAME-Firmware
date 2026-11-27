# V2.3.1 EXP — primera integración de diagnóstico universal
Base V230 0b651c1. No cambios en processSample, eventos EVENT, NVS, calibración, parámetros ni perfiles. Los ensayos P0–P8 fueron manuales en banco, P15 mixto banco/salto corporal con sensores en manos; no validación de tobillo. No ajustar salto hasta montaje real.
## Contrato TRACE
- D,T,1 conserva 1024 muestras, D,S,5/15/30/60 solicita 1121/3363/6726/13452. Memoria PSRAM reservada al arranque; si no disponible, D,S falla explícitamente y D,T,1 sigue operativo. D,T,0 congela. D,T consulta.
- D,B,generation,index,count (1..16 muestras congeladas): 3 páginas D3 de 20 B por muestra en orden índice/página; final D4 de 20 B: id,count,version1,gen u16 LE,start u16 LE,CRC32 IEEE de páginas D3 completas en orden (bytes 8..11 LE), resto 0. No depende de MTU extendido. D,C cancela. D,R conserva lectura anterior. Emisión paced desde loop (mínimo 6ms), jamás desde IMU ni durante adquisición por diseño del cliente. Bloques se pueden repetir; CRC protege contenido, SHA del CSV protege archivo, ninguno prueba verdad biomecánica. No prometer tiempos hasta prueba real.
- La RAM extendida es un solo buffer circular configurable, no múltiples snapshots GFA aún. Pendiente persistencia/resume entre desconexiones y captura pre/post disparada sin bloquear control; no declarar completos.
- STEP/WALK/RUN: conservar canal universal y roles 1–4; detector definitivo pospuesto hasta señales de tobillos. Mantener LEGACY intacto.
## Verificación
Pruebas host de ring extendido, CRC conocido, encabezado D4; CI incluye compilación ambas placas y replay exacto perfil0 contra 229. Validación Android/PSRAM/throughput real pendiente.
