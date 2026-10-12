// Behavioural checks for the engine-sim port. Run: node tests/physics.test.js
const assert = require('assert');
const { execFileSync } = require('child_process');
const path = require('path');
const P = require('../Engine-Sim-Physics.js');
const { ES_ASSETS } = require('../Engine-Sim-Assets.js');

const U = P.ES_UNITS;

function impulseResponse(name) {
  const ir = ES_ASSETS.irs[name], b = Buffer.from(ir.samples, 'base64');
  return P.esImpulseResponse(new Int16Array(b.buffer, b.byteOffset, b.length / 2), ir.volume);
}

// The convolution is the same as the direct form in ConvolutionFilter.
{
  const ir = Float64Array.from({ length: 700 }, () => Math.random() - 0.5);
  const conv = new P.ESConvolution(ir, 128), x = Float64Array.from({ length: 128 * 10 }, () => Math.random() - 0.5);
  const block = new Float64Array(128);
  let err = 0;
  for (let b = 0; b < 10; b++) {
    conv.process(x.subarray(b * 128, b * 128 + 128), block);
    for (let i = 0; i < 128; i++) {
      const n = b * 128 + i;
      let s = 0;
      for (let k = 0; k < ir.length && k <= n; k++) s += ir[k] * x[n - k];
      err = Math.max(err, Math.abs(s - block[i]));
    }
  }
  assert(err < 1e-9, `convolution error ${err}`);
}

// Drives the simulator like EngineSimApplication: 60 frames a second, the
// speed control filtered toward its target, the audio rendered in blocks.
function bench(key) {
  const spec = P.ES_ENGINES[key], input = new P.ESSynthesizerInput(spec.exhausts.length);
  const sim = new P.ESSimulator(spec, input), e = sim.engine;
  const core = new P.ESSynthesizerCore(spec.exhausts.length, spec.exhausts.map(x => impulseResponse(x.impulseResponse)));
  core.setParameters({ dF_F_mix: e.hfGain, airNoise: e.noise, inputSampleNoise: e.jitter });
  const queue = spec.exhausts.map(() => []);
  let speed = 0, audio = [];
  const state = {
    sim, e, target: 0,
    run(seconds) {
      for (let i = 0, n = Math.round(seconds * 60); i < n; i++) {
        speed = state.target * 0.5 + 0.5 * speed;
        e.setSpeedControl(speed);
        sim.startFrame(1 / 60);
        while (sim.simulateStep());
        sim.endFrame();
        input.drain().forEach((a, c) => { for (const v of a) queue[c].push(v); });
        while (queue[0].length >= 128) {
          const out = new Float32Array(128);
          core.render(queue.map(q => Float32Array.from(q.splice(0, 128))), out);
          for (const v of out) audio.push(v);
        }
      }
    },
    takeAudio() { const a = audio; audio = []; return a; },
  };
  return state;
}

for (const key of P.ES_ENGINE_ORDER) {
  const b = bench(key), { sim, e } = b;
  sim.ignitionModule.enabled = true; sim.starterMotor.enabled = true;
  b.run(1);
  sim.starterMotor.enabled = false;
  b.run(3);
  const idle = e.rpm();
  assert(idle > 500 && idle < 1200, `${key}: idles at ${idle.toFixed(0)} rpm`);
  assert(e.manifoldPressure() < 75000, `${key}: manifold vacuum at idle (${(e.manifoldPressure() / 1000).toFixed(0)} kPa)`);
  const idleAudio = b.takeAudio().slice(-44100);
  const rms = Math.sqrt(idleAudio.reduce((s, v) => s + v * v, 0) / idleAudio.length);
  assert(rms > 0.02 && rms < 1, `${key}: idle audio rms ${rms.toFixed(3)}`);

  b.target = 1;
  b.run(3);
  assert(e.rpm() > 0.8 * e.redline / U.rpm, `${key}: revs to ${e.rpm().toFixed(0)} rpm at full throttle`);

  sim.dyno.enabled = true; sim.dyno.hold = true; sim.dyno.rotationSpeed = 3000 * U.rpm;
  b.run(3);
  const torque = sim.filteredDynoTorque() / U.lbft;
  assert(Math.abs(e.rpm() - 3000) < 50, `${key}: dyno holds 3000 rpm (${e.rpm().toFixed(0)})`);
  assert(torque > 80, `${key}: dyno torque ${torque.toFixed(0)} lb-ft`);

  sim.dyno.enabled = false; sim.dyno.hold = false; b.target = 0; sim.ignitionModule.enabled = false;
  b.run(6);
  assert(e.rpm() < 1, `${key}: stops with the ignition off (${e.rpm().toFixed(0)} rpm)`);
  console.log(`ok ${key}: idle ${idle.toFixed(0)} rpm, ${torque.toFixed(0)} lb-ft at 3000 rpm WOT, idle audio rms ${rms.toFixed(3)}`);
}

execFileSync(process.execPath, [path.join(__dirname, '..', 'tools', 'inline-scripts.js'), '--check'], { stdio: 'inherit' });
