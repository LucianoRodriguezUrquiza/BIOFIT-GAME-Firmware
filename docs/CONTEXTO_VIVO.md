# V2.3.0 experimental — RCA

Rama `feat/diagnostics-v230`, 26/11/2026. V2.2.9 sigue estable en main. Nuevo firmware: [protocolo y revisión](REVISION_V230_DIAGNOSTICO.md), perfiles RAM 0/1/2/4/8 combinables, trace de 1024 muestras; Unity aún sin interfaz de control. CI y ensayo físico pendientes al publicar. No se cambia el detector de Unity ni las físicas. Mantener evidencia y distinguir pruebas host de hardware.

---

# BIOFIT GAME — Firmware V2.2.9 / relevo técnico

26/11/2026. V2.2.9 es la versión de referencia aprobada por el usuario para main. Código original: `274625f605ea92189abfb16275012f34ee14371f`; V2.2.5 obsoleta para uso activo, conservada en historial. Ver [consolidación y validación](CONSOLIDACION_V229.md), [revisión técnica](REVISION_V229_CICLO.md) y [README](../README.md).

Dos trackers Waveshare ESP32-S3-LCD-1.47B / QMI8658. BLE BILATERAL LEGACY, UP tipo 3, control corporal primario en BIOFIT DASH V6.5; LIFT_ONSET tipo 4 es experimental y separado. No cambiar detector, parámetros ni NVS sin diagnóstico y autorización. Verificar los valores efectivos de ambos trackers y no asumir que defaults equivalen a configuración instalada.

CI del código V2.2.9: 3/3 (run 44444687178). No prueba física independiente. Pendiente prioritario: pérdida progresiva de saltos y recuperación tras reposo; verificar contadores UP/AR y exportar CSV antes de modificar código. Hipótesis abiertas: rearme, gravedad, BLE/reloj/emparejamiento, condición de apoyo y otras causas. No atribuir causa a AutoRest solo por correlación.

RCA: registrar decisiones, pruebas, errores, versiones, cambios y límites de validación en GitHub durante el trabajo. Conservar los recursos y configuraciones existentes. La siguiente intervención la define el usuario.
