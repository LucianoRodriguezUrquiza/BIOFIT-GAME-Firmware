# Consolidación y aceptación — Universal Tracker V2.2.9

Fecha: 26/11/2026. El usuario identifica V2.2.9 como la última versión de firmware y solicita expresamente incorporarla a main como validada y sustituir la referencia V2.2.5.

## Alcance y trazabilidad

- Main anterior: `9bb4183db5d2bfd76bc0d9aeefd203d0543b315d` (V2.2.5).
- Código V2.2.9: `274625f605ea92189abfb16275012f34ee14371f`, rama feat/lift-cycle-v229, 19 commits posteriores a main anterior, sin divergencia.
- Se conserva el historial, ramas antiguas y referencias de pruebas. V2.2.5 es obsoleta para desarrollo activo, no se borra del historial: tests/check_contract.py usa el commit 9bb4183 para verificar invariantes.
- La consolidación añade solo documentación sobre el árbol de V2.2.9; no modifica .ino, .h, workflow ni tests, calibración, NVS, filtros, protocolo, thresholds, rearme o timing.
- Proyecto Unity asociado: [BIOFIT-GAME-UNITY](https://github.com/LucianoRodriguezUrquiza/BIOFIT-GAME-UNITY), main V6.5, referencia 3ed95b2dc2ddfab081c587c68283221c206bf00b. No se cambia Unity en esta operación.
- Los encabezados históricos EXP y el canal LIFT_ONSET se conservan: no son una certificación física de ese algoritmo.

## Validación

Aceptada por el usuario como versión estable de trabajo. GitHub Actions [44444687178](https://github.com/LucianoRodriguezUrquiza/BIOFIT-GAME-Firmware/actions/runs/44444687178), commit de código V2.2.9, completed/success, 3/3:
- compile (Izquierdo): aprobado, core ESP32 3.3.5, GFX 1.6.9.
- compile (Derecho): aprobado, mismo entorno.
- source-contract: aprobado; contratos, replay baseline/actual, pruebas host lift_cycle y phase_evidence.

No se ha realizado carga en dispositivos ni validación independiente de binario/NVS, movimiento sostenido, sensibilidad, falsos positivos o latencia gesto-pantalla. Los resultados sintéticos y las limitaciones del replay RAW se mantienen en REVISION_V229_CICLO.md.

## Estado técnico y siguiente paso

El usuario reporta respuesta rápida en LEGACY y cooldown 400 ms, dificultad tras aproximadamente 6–7 saltos consecutivos, deterioro tras aproximadamente 20 acciones y recuperación después de 20–30 s quieto. Se observa MOV en LCD durante el fallo; aún no se sabe si UP y AR aumentan. Causa no determinada. Investigar primero contadores UP/AR, eventos CSV sin RAW y después RAW en sesión separada, reloj, secuencia, recepción, emparejamiento y acción física; verificar configuración efectiva de cada tracker. H1 rearme, H2 estimación gravitatoria, H3 transporte/sincronización son hipótesis, no conclusiones.

## Descarga e instalación

Descargar [ZIP de main](https://github.com/LucianoRodriguezUrquiza/BIOFIT-GAME-Firmware/archive/refs/heads/main.zip). Abrir cada .ino en su carpeta con las dos cabeceras auxiliares. Usar los ajustes que ya funcionaban y conservar NVS; no borrar toda la flash. No se requiere reprogramar dispositivos para esta consolidación documental. Consultar el procedimiento y reversión históricos en REVISION_V229_CICLO.md. Mantener ICC/DIPL/RFD/BRO/FLM/RCA.
