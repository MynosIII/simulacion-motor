# Código y recursos reutilizados

## engine-sim

Repositorio: https://github.com/ange-yaghi/engine-sim
Commit: 85f7c3b959a908ed5232ede4f1a4ac7eafe6b630

Engine-Sim-Physics.js es un port a JavaScript de:

- GasSystem: src/gas_system.cpp, include/gas_system.h
- CombustionChamber: src/combustion_chamber.cpp
- Intake: src/intake.cpp
- ExhaustSystem: src/exhaust_system.cpp
- Fuel: src/fuel.cpp
- Camshaft, StandardValvetrain, VtecValvetrain: src/camshaft.cpp, src/standard_valvetrain.cpp, src/vtec_valvetrain.cpp
- IgnitionModule: src/ignition_module.cpp
- DirectThrottleLinkage: src/direct_throttle_linkage.cpp
- Simulator y PistonEngineSimulator (pasos por cuadro, latencia del sintetizador, par del dinamómetro, simulateStep_, writeToSynthesizer, endFrame): src/simulator.cpp, src/piston_engine_simulator.cpp
- Synthesizer, ButterworthLowPassFilter, JitterFilter, LevelingFilter, DelayFilter, ConvolutionFilter: src/synthesizer.cpp y filtros en include/ y src/
- StarterMotor, Dynamometer, Transmission, Vehicle, VehicleDragConstraint: src/starter_motor.cpp, src/dynamometer.cpp, src/transmission.cpp, src/vehicle.cpp, src/vehicle_drag_constraint.cpp
- Levas armónicas: GenerateHarmonicCamLobeNode en scripting/include/actions.h
- Valores por defecto y unidades: es/objects/objects.mr, include/units.h
- Motores: assets/engines/atg-video-2/01_subaru_ej25_eh.mr, assets/engines/atg-video-1/05_honda_vtec.mr, assets/engines/atg-video-2/04_60_degree_v6.mr, assets/engines/atg-video-2/07_gm_ls.mr

Engine-Sim-UI.js es un port a JavaScript de la interfaz:

- ui_math.h, UiElement, UiManager, Gauge, LabeledGauge, Oscilloscope y GeometryGenerator (formas usadas)
- OscilloscopeCluster, RightGaugeCluster, FiringOrderDisplay, ThrottleDisplay, AfrCluster, FuelCluster, PerformanceCluster, LoadSimulationCluster, MixerCluster, InfoCluster, EngineView
- Objetos de la vista del motor: ConnectingRodObject, PistonObject, CombustionChamberObject, CrankshaftObject, CylinderBankObject, CylinderHeadObject
- EngineSimApplication: bucle de cuadros, controles, mensajes y las tres pantallas; colores de ApplicationSettings

Engine-Sim-Assets.js contiene:

- Contornos de los modelos de assets/assets.ysce (Crankshaft, ConnectingRod, Piston, CylinderHead, Valve, CrankSnout, CrankSnoutThreads, Logo). tools/assets/ysce_models.py lee el archivo y tools/assets/model_outlines.py convierte las mallas en contornos.
- Respuestas de impulso de es/sound-library: new/minimal_muffling_02.wav, new/mild_exhaust.wav, new/mild_exhaust_reverb.wav y smooth/smooth_39.wav, recortadas como Synthesizer::initializeImpulseResponse (tools/assets/impulse_responses.py).
- La fuente Silkscreen (ver abajo).

tools/assets/build_assets.js arma Engine-Sim-Assets.js con esos tres archivos.

No se compila ni se ejecuta el programa C++ original. El solver de cuerpos rígidos (scs) no está portado; detalle en CONTROLES-Y-MODELOS.md.

Copyright 2022 AngeTheGreat (Ange Yaghi)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## delta-studio

Repositorio: https://github.com/ange-yaghi/delta-studio
Commit: e9956e7b06849cbbdc05df5848691112ec3c3258

Del motor gráfico se portaron el formato de mallas .ysce (lectura), TextRenderer (posición de texto) y la conversión a sRGB del shader. Licencia MIT, Copyright (c) 2020 Ange Yaghi, mismo texto que arriba.

## Silkscreen

engines/basic/fonts/Silkscreen/slkscr.ttf de delta-studio, la fuente que usa engine-sim. Autor: Jason Kottke. Se incluye sin modificar (en base64), como permite su licencia:

> This font is free for personal and corporate use and may be redistributed in this unmodified form on your Web site. I would ask that you not modify and then redistribute this font.

http://www.kottke.org/plus/type/silkscreen/index.html
