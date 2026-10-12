// Behavioural checks for the physics port. Run: node tests/physics.test.js
const assert = require('assert');
const { execFileSync } = require('child_process');
const path = require('path');
const { ES_ENGINES, ESEngineSimulation, ESSynthesizer } = require('../Engine-Sim-Physics.js');

function run(sim, seconds, synth) {
  const out = [0, 0];
  for (let i = 0, n = Math.round(seconds / sim.timestep); i < n; i++) {
    sim.step();
    if (synth) synth.writeInput(sim.exhaustSignal(out));
  }
}

for (const [key, spec] of Object.entries(ES_ENGINES)) {
  const sim = new ESEngineSimulation(spec);
  const c = sim.controls;

  c.ignition = true; c.starter = true;
  run(sim, 1.5);
  c.starter = false;
  run(sim, 2);
  const idle = sim.rpm();
  assert(idle > 500 && idle < 1200, `${key}: idles at ${idle.toFixed(0)} rpm`);
  assert(sim.manifoldPressure() < 75000, `${key}: manifold vacuum at idle (${(sim.manifoldPressure() / 1000).toFixed(0)} kPa)`);

  c.throttle = 1;
  run(sim, 3);
  assert(sim.rpm() > 0.85 * spec.redline, `${key}: revs to ${sim.rpm().toFixed(0)} rpm at full throttle`);

  c.dyno = { enabled: true, hold: true, rpm: 3000 };
  run(sim, 1.5);
  const torque = sim.averageDynoTorque();
  assert(Math.abs(sim.rpm() - 3000) < 50, `${key}: dyno holds 3000 rpm`);
  assert(torque > 100, `${key}: dyno torque ${torque.toFixed(0)} Nm`);

  c.dyno = { enabled: false, hold: false, rpm: 0 }; c.throttle = 0; c.ignition = false;
  run(sim, 6);
  assert(sim.rpm() < 1, `${key}: stops with the ignition off (${sim.rpm().toFixed(0)} rpm)`);

  // The synthesizer output is audible and its strongest component is a firing-frequency harmonic.
  const sim2 = new ESEngineSimulation(spec), synth = new ESSynthesizer(44100, spec.simulationFrequency);
  synth.setParameters(spec.audio);
  sim2.controls.ignition = true; sim2.controls.throttle = 0.5; sim2.controls.dyno = { enabled: true, hold: true, rpm: 3000 };
  run(sim2, 1.5, synth);
  synth.drain();
  run(sim2, 0.5, synth);
  const [left] = synth.drain();
  const rms = Math.sqrt(left.reduce((s, v) => s + v * v, 0) / left.length);
  assert(rms > 0.05 && rms < 1, `${key}: audio rms ${rms.toFixed(3)}`);
  let best = 0, bestF = 0;
  for (let f = 20; f <= 600; f++) {
    let re = 0, im = 0;
    for (let i = 0; i < left.length; i++) { const a = 2 * Math.PI * f * i / 44100; re += left[i] * Math.cos(a); im -= left[i] * Math.sin(a); }
    if (re * re + im * im > best) { best = re * re + im * im; bestF = f; }
  }
  const firing = 3000 * spec.cylinders / 120, harmonic = bestF / firing;
  assert(Math.abs(harmonic - Math.round(harmonic)) < 0.05 && Math.round(harmonic) >= 1, `${key}: audio peak ${bestF} Hz vs firing ${firing} Hz`);

  console.log(`ok ${key}: idle ${idle.toFixed(0)} rpm, ${torque.toFixed(0)} Nm at 3000 rpm WOT, audio peak ${bestF} Hz`);
}

execFileSync(process.execPath, [path.join(__dirname, '..', 'tools', 'inline-scripts.js'), '--check'], { stdio: 'inherit' });
