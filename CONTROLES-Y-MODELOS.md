# Engine Lab en navegador

Abrir Engine-Lab-Local.html en Chrome, Edge o Firefox. También se puede servir ese mismo HTML en cualquier hosting estático. No necesita servidor, librerías externas ni conexión a internet.

La página es un port de engine-sim: la misma pantalla (vista del motor en corte, relojes, osciloscopios, colores y tipografía Silkscreen), la misma simulación y la misma cadena de sonido. El navegador solo permite audio después de un clic o una tecla; el recuadro sobre la vista del motor lo indica.

## Controles (los del original)

| Tecla | Acción |
|---|---|
| A | Encendido |
| S (mantener) | Motor de arranque |
| Q / W / E / R | Acelerador al 1 %, 10 %, 20 % y 100 % |
| Espacio + rueda | Control fino del acelerador; con Espacio el embrague responde lento |
| D | Dinamómetro (barrido hasta el corte) |
| H | Mantener RPM con el dinamómetro; G + rueda cambia la velocidad |
| ↑ / ↓ | Subir / bajar marcha |
| Shift (mantener) | Pisar el embrague; T / U lo ajustan, Y lo mantiene |
| Z / X / C / V / B + rueda | Volumen, convolución, ganancia de agudos, ruido de graves, ruido de agudos |
| N + rueda | Frecuencia de simulación |
| 1 a 5 (mantener) | Cámara lenta (1/10 a 1/1000) |
| F1 / F2 / F3 | Rotar la vista / volver al frente |
| M / , | Capa de dibujo (cilindros más atrás) |
| I | Mostrar potencia y par máximos |
| Tab | Cambiar pantalla (completa, solo motor, motor + relojes) |
| F | Pantalla completa |
| Enter | Recargar el motor |
| Arrastrar / rueda sobre el motor | Mover / acercar la vista |
| Clic en los osciloscopios chicos | Mostrarlos en el grande |

Agregados del port: clic en el nombre del motor o RePág / AvPág para cambiar de motor, y `?engine=gm_ls` en la dirección para abrir uno en particular.

## Motores

- Subaru EJ25 (por defecto, como en el original): assets/engines/atg-video-2/01_subaru_ej25_eh.mr.
- Honda B18C5 con VTEC: assets/engines/atg-video-1/05_honda_vtec.mr.
- V6 60°: assets/engines/atg-video-2/04_60_degree_v6.mr.
- GM LS V8: assets/engines/atg-video-2/07_gm_ls.mr.

Cada motor usa su respuesta de impulso de escape original.

## Límites respecto al original

- El original resuelve cigüeñal, bielas y pistones como cuerpos rígidos con un solver de restricciones (scs). Acá el cigüeñal es un único grado de libertad, los pistones y bielas siguen la cinemática exacta de biela y manivela y la inercia depende del ángulo. Arranque, freno dinamométrico, rozamiento, embrague y resistencia del vehículo usan los mismos límites de par, resueltos por impulsos.
- Los motores están transcriptos a mano desde los .mr; no está portado el intérprete de scripts.
- La salida de audio va directo a Web Audio desde un AudioWorklet, sin el búfer intermedio del original; el reloj LATENCY mide la cola de entrada del sintetizador más la latencia del dispositivo.

## Desarrollo

- `node tests/physics.test.js` arranca cada motor, comprueba ralentí, acelerador a fondo, dinamómetro a 3000 RPM, apagado y nivel de audio, compara la convolución con la forma directa y verifica que el HTML tenga la última versión de los scripts.
- `node tools/inline-scripts.js` copia Engine-Sim-Assets.js, Engine-Sim-Physics.js y Engine-Sim-UI.js dentro de Engine-Lab-Local.html. Correrlo después de editar esos archivos.
- tools/assets tiene los scripts que extrajeron los modelos, la fuente y las respuestas de impulso (ver PROCEDENCIA.md).
