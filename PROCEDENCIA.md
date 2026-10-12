# Código reutilizado

Repositorio: https://github.com/ange-yaghi/engine-sim
Commit: 85f7c3b959a908ed5232ede4f1a4ac7eafe6b630

Engine-Sim-Physics.js es un port a JavaScript de:

- GasSystem: src/gas_system.cpp, include/gas_system.h
- CombustionChamber: src/combustion_chamber.cpp
- Intake: src/intake.cpp
- ExhaustSystem: src/exhaust_system.cpp
- Fuel: src/fuel.cpp
- IgnitionModule (adaptado a giro positivo): src/ignition_module.cpp
- DirectThrottleLinkage: src/direct_throttle_linkage.cpp
- PistonEngineSimulator::simulateStep_ y writeToSynthesizer: src/piston_engine_simulator.cpp
- Simulator (pasos por cuadro y latencia del sintetizador, promedio del par del dinamómetro): src/simulator.cpp
- Synthesizer, ButterworthLowPassFilter, JitterFilter, LevelingFilter, DelayFilter: src/synthesizer.cpp y filtros en include/ y src/
- StarterMotor, Dynamometer, VehicleDragConstraint (límites de par): src/starter_motor.cpp, src/dynamometer.cpp, src/vehicle_drag_constraint.cpp
- Levas armónicas: GenerateHarmonicCamLobeNode en scripting/include/actions.h
- Valores por defecto de objetos y unidades: es/objects/objects.mr, include/units.h, scripting/include/engine_node.h
- Motores: assets/engines/atg-video-1/05_honda_vtec.mr, assets/engines/atg-video-2/04_60_degree_v6.mr, assets/engines/atg-video-2/07_gm_ls.mr

Engine-Sim-Core.js conserva esPlaceCylinder (src/piston_engine_simulator.cpp), usado para dibujar el motor.

Respuesta de impulso incrustada en el HTML: assets/sound-library/smooth/smooth_39.wav

No se compila ni se ejecuta el programa C++ original. El solver de cuerpos rígidos (scs) no está portado: se reemplazó por un cigüeñal de un grado de libertad con cinemática exacta de biela. Detalle en CONTROLES-Y-MODELOS.md.

Copyright 2022 AngeTheGreat (Ange Yaghi)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
