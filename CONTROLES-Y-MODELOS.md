# Engine Lab en navegador

Abrir Engine-Lab-Local.html en Chrome o Edge. También se puede servir ese mismo HTML en cualquier hosting estático. No necesita login, servidor de aplicaciones, librerías externas ni conexión a internet.

## Qué se simula

El núcleo de engine-sim está portado a JavaScript en Engine-Sim-Physics.js (y copiado dentro del HTML):

- Sistema de gases de cada cilindro, conducto de admisión, conducto de escape, plenum y colectores: presión, temperatura, mezcla, flujo estrangulado y subsónico, y velocidad de los gases.
- Combustión con frente de llama, turbulencia según la velocidad media del pistón, dilución, eficiencia aleatoria y pérdida de calor a las paredes.
- Encendido con curva de avance por RPM y limitador de revoluciones.
- Cigüeñal con la presión de cada cilindro, masas alternativas, rozamiento de pistón y de cigüeñal, arranque, freno dinamométrico, embrague y vehículo.
- Sonido: la presión de cada escape se retrasa según el largo del tubo, se pasa por el sintetizador original (jitter, ruido de aire, mezcla de derivada, nivelador) y luego por la respuesta de impulso original.

La simulación corre a la frecuencia del motor original (10.000 o 20.000 pasos por segundo, con 8 subpasos de gases). Si la computadora no llega, la simulación se ralentiza en lugar de trabar la página; el instrumento Renderizado muestra el porcentaje de tiempo real alcanzado.

## Motores

- Honda B18 · Inline 4 · 1.8 L: datos de assets/engines/atg-video-1/05_honda_vtec.mr con la leva de bajas, sin VTEC.
- V6 60° · 3.3 L: assets/engines/atg-video-2/04_60_degree_v6.mr.
- GM LS V8 · 5.3 L: assets/engines/atg-video-2/07_gm_ls.mr.

## Controles disponibles

- Encendido A (arranca solo con el motor de arranque hasta que toma), arranque manual S, acelerador Q/W/E/R y Espacio, carga en el cigüeñal, volumen.
- Barrido D: el freno dinamométrico mantiene 1000 RPM y sube hasta el corte, registrando el par promediado por ciclo. Exporta CSV.
- RPM hold H con objetivo ajustable (mismo freno dinamométrico).
- Marchas ↑/↓, embrague Shift y freno con punto, con los datos de vehículo y caja de cada motor original.
- Mezclador: ganancia previa, convolución con respuesta de impulso original, agudos, ruido de aire y jitter del sintetizador, compresión de picos. Medidor de salida en dBFS RMS.
- Velocidad de simulación, frecuencia de simulación, vista en tiempo real o lenta, pausa, avance angular y pantalla completa F.
- Orden de encendido, relación de compresión, avance adicional sobre la curva del motor, duración y alzada de leva (se aplican a admisión y escape) y AFR. Squish es solo visual.
- Modelo de vibración: masa alternativa, masa del bloque, frecuencia natural y amortiguamiento de soportes.

## Gráficos

1. Alzada de válvulas del cilindro 1.
2. Presión del cilindro 1 vs. ángulo.
3. Presión–volumen del cilindro 1.
4. Caudal de admisión y escape del cilindro 1 (negativo = reflujo).
5. Moles de gas del cilindro 1.
6. Avance vs. RPM: curva del motor más el ajuste.
7. RPM vs. tiempo.
8. Vibración vs. tiempo.
9. Forma de onda de la salida de Web Audio.
10. Espectro de la salida de Web Audio.
11. Potencia y par del barrido.

## Límites respecto al original

- El original resuelve cigüeñal, bielas y pistones como cuerpos rígidos con un solver de restricciones. Acá el cigüeñal es un único grado de libertad y los pistones siguen la cinemática exacta de biela y manivela; la masa de la biela se reparte entre el muñón y el pistón. Arranque, freno, embrague y resistencia del vehículo usan los mismos límites de par, aplicados como impulsos secuenciales.
- Cada cilindro tiene su propia fase según el orden de encendido (encendido parejo); no se modela la geometría de bancadas en V para el gas ni bielas maestras.
- No están portados el intérprete de archivos .mr, VTEC, el editor de curvas de avance ni la interfaz original.
- La convolución corre en un ConvolverNode de Web Audio después del nivelador, no por canal antes de mezclar.
- La vibración del bloque sigue siendo un modelo analítico aparte (fuerzas de inercia de primer y segundo orden y soportes elásticos).

## Desarrollo

- `node tests/physics.test.js` arranca cada motor, comprueba ralentí, aceleración, freno dinamométrico, apagado y sonido, y verifica que el HTML tenga la última versión de los scripts.
- `node tools/inline-scripts.js` copia Engine-Sim-Core.js y Engine-Sim-Physics.js dentro de Engine-Lab-Local.html. Correrlo después de editar esos archivos.
