# Código reutilizado

Repositorio: https://github.com/ange-yaghi/engine-sim
Commit: 85f7c3b959a908ed5232ede4f1a4ac7eafe6b630

- ESFunction.sampleTriangle: src/function.cpp
- esHarmonicLobe: scripting/include/actions.h
- ESCamshaft.valveLift: src/camshaft.cpp
- esPlaceCylinder: src/piston_engine_simulator.cpp
- ESIgnition.advance (adaptado a giro positivo): src/ignition_module.cpp
- Respuesta de impulso incrustada: assets/sound-library/smooth/smooth_39.wav
- Dimensiones y orden de GM LS: assets/engines/atg-video-2/07_gm_ls.mr

El HTML integra adaptaciones JavaScript de estos algoritmos, el WAV original y la licencia MIT. No ejecuta el motor completo de combustión, gases ni restricciones dinámicas. El audio usa Web Audio en lugar del sintetizador C++ completo.

Copyright 2022 AngeTheGreat (Ange Yaghi)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
