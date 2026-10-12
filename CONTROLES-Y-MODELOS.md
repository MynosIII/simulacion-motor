# Engine Lab en navegador

Abrir Engine-Lab-Local.html en Chrome o Edge. También se puede servir ese mismo HTML en cualquier hosting estático. No necesita login, servidor de aplicaciones, librerías externas ni conexión a internet.

## Controles disponibles

- Encendido A, arranque S, acelerador Q/W/E/R y Espacio, carga, volumen.
- Barrido D con CSV y RPM hold H con objetivo ajustable.
- Marchas ↑/↓, embrague Shift y freno con punto. Modelo reducido del vehículo.
- Mezclador: ganancia previa, convolución con respuesta de impulso original, agudos, ruido LF/HF y compresión de picos. Medidor de salida en dBFS RMS.
- Velocidad de simulación, vista en tiempo real o lenta, pausa, avance angular y pantalla completa F.
- Orden de encendido, squish visual, relación de compresión, avance fijo, alzada y duración nominal de leva. Resolución de leva ajustable.
- Modelo de vibración: masa alternativa, masa del bloque, frecuencia natural y amortiguamiento de soportes. Ampliación visual independiente del valor RMS.

## Gráficos

1. Alzada de válvulas: perfil armónico adaptado del original.
2. Presión del cilindro vs. ángulo: modelo polytropico ilustrativo.
3. Presión–volumen: mismo modelo aproximado.
4. Caudal de admisión y escape: aproximación a partir de alzada y RPM.
5. Moles del cilindro: aproximación con gas ideal.
6. Avance vs. RPM: muestra el ajuste fijo actual, no una tabla de avance editable.
7. RPM vs. tiempo: historial real del modelo web.
8. Vibración vs. tiempo: desplazamiento estimado del bloque, muestreado por el renderizado.
9. Forma de onda: muestras reales de la salida de Web Audio.
10. Espectro: FFT real de la salida de Web Audio.
11. Potencia y par: curvas del barrido, con datos estimados y exportación CSV.

## Límites respecto al original

No están portados el solver C++ de restricciones mecánicas, gases y combustión, el intérprete de archivos .mr, el sintetizador C++ completo ni su frecuencia de simulación. Tampoco se implementó VTEC, el editor de curvas de avance o la interacción completa vehículo–transmisión del original. La interfaz muestra los controles disponibles y no usa controles simulados para afirmar que esas funciones existen.

El cálculo de vibración usa fuerzas inerciales de primer y segundo orden y respuesta armónica de soportes elásticos. No incluye pulsos de combustión, deformación del cigüeñal ni calibración experimental. Los valores son para comparar el modelo, no para predecir un motor real. Las fases de cilindros siguen siendo virtuales, según el orden de encendido elegido.

La presión, caudales, temperatura, AFR y rendimiento no son resultados del motor físico original. El código trasladado y su licencia figuran en PROCEDENCIA.md y dentro del HTML.
