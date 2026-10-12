/* Engine-Sim-Physics.js
 * JavaScript port of the engine-sim simulation core, Copyright 2022 AngeTheGreat
 * (Ange Yaghi), MIT. Source: https://github.com/ange-yaghi/engine-sim
 * commit 85f7c3b959a908ed5232ede4f1a4ac7eafe6b630. See PROCEDENCIA.md.
 *
 * Ported closely: GasSystem (src/gas_system.cpp), CombustionChamber
 * (src/combustion_chamber.cpp), Intake (src/intake.cpp), ExhaustSystem
 * (src/exhaust_system.cpp), Fuel (src/fuel.cpp), IgnitionModule
 * (src/ignition_module.cpp), PistonEngineSimulator::simulateStep_ and
 * writeToSynthesizer (src/piston_engine_simulator.cpp) and Synthesizer
 * (src/synthesizer.cpp) with its filters.
 *
 * Replaced: the original solves crank, rods and pistons as rigid bodies with
 * the scs constraint solver. Here the crank is a single degree of freedom and
 * pistons follow exact slider-crank kinematics, which is equivalent for rigid
 * links. Rod mass is split between the crank pin and the piston. Starter, dyno,
 * clutch and vehicle drag keep the original torque limits but are solved as
 * sequential impulses instead of constraints.
 */
'use strict';

const ES_R = 8.31446261815324;
const ES_AIR_MOLAR_MASS = 0.02897;
const ES_ATM = 101325;
const ES_T25 = 298.15;
const ES_TAU = 2 * Math.PI;
const ES_FOUR_PI = 4 * Math.PI;
const ES_UNITS = {
  inch: 0.0254, thou: 0.0000254, mm: 0.001, cm: 0.01, cc: 1e-6, L: 1e-3, cm2: 1e-4,
  g: 0.001, lb: 0.45359237, lbft: 0.3048 * 4.44822, rpm: 0.104719755,
  scfm: 0.002641 * 453.59237 / 60, inHg: 3386.3886666666713, deg: Math.PI / 180,
};
ES_UNITS.inH2O = ES_UNITS.inHg * 0.0734824;

function esClamp(x, a = 0, b = 1) { return x < a ? a : (x > b ? b : x); }
function esMod(x, n) { const r = x % n; return r < 0 ? r + n : r; }

// Piecewise linear lookup. For evenly spaced samples this is what
// Function::sampleTriangle returns when the filter radius equals the spacing.
class ESTable {
  constructor(xs, ys) { this.xs = xs; this.ys = ys; }
  sample(x) {
    const xs = this.xs, ys = this.ys, n = xs.length;
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (xs[mid] > x) hi = mid; else lo = mid; }
    const f = (x - xs[lo]) / (xs[hi] - xs[lo]);
    return ys[lo] + (ys[hi] - ys[lo]) * f;
  }
}

/* ---------------------------------------------------------------- Gas system */

const ES_HCR5 = 1.4;
const ES_CHOKED_LIMIT5 = Math.pow(2 / (ES_HCR5 + 1), ES_HCR5 / (ES_HCR5 - 1));
const ES_CHOKED_RATE5 = Math.sqrt(ES_HCR5) * Math.pow(2 / (ES_HCR5 + 1), (ES_HCR5 + 1) / (2 * (ES_HCR5 - 1)));
const ES_SUBSONIC_N = 4096, ES_SUBSONIC_SCALE = ES_SUBSONIC_N / Math.sqrt(1 - ES_CHOKED_LIMIT5);
const ES_SUBSONIC = new Float64Array(ES_SUBSONIC_N + 2);
for (let i = 0; i <= ES_SUBSONIC_N + 1; i++) {
  const u = Math.min(i, ES_SUBSONIC_N) / ES_SUBSONIC_SCALE, ratio = 1 - u * u, s = Math.pow(ratio, 1 / ES_HCR5);
  ES_SUBSONIC[i] = Math.sqrt(Math.max((2 * ES_HCR5) / (ES_HCR5 - 1) * s * (s - ratio), 0));
}
// P * (sqrt(x^7) - 1), x = 1 + (hcr - 1) / 2 * Mach^2: GasSystem::dynamicPressure for 5 degrees of freedom.
function esStagnation(P, halfMachTerm) {
  const x = 1 + halfMachTerm, x2 = x * x, x3 = x2 * x;
  return P * (Math.sqrt(x3 * x3 * x) - 1);
}

class ESGasSystem {
  constructor() {
    this.n = 0; this.E = 0; this.V = 0; this.px = 0; this.py = 0;
    this.fuel = 0; this.inert = 1; this.o2 = 0;
    this.dof = 5; this.width = 0; this.height = 0; this.dx = 0; this.dy = 0;
  }
  setGeometry(width, height, dx, dy) { this.width = width; this.height = height; this.dx = dx; this.dy = dy; }
  initialize(P, V, T, fuel = 0, inert = 1, o2 = 0) {
    this.n = P * V / (ES_R * T); this.V = V; this.E = T * (0.5 * this.dof * this.n * ES_R);
    this.fuel = fuel; this.inert = inert; this.o2 = o2; this.px = this.py = 0;
  }
  reset(P, T, fuel = 0, inert = 1, o2 = 0) {
    this.n = P * this.V / (ES_R * T); this.E = T * (0.5 * this.dof * this.n * ES_R);
    this.fuel = fuel; this.inert = inert; this.o2 = o2; this.px = this.py = 0;
  }
  pressure() { return this.V !== 0 ? this.E / (0.5 * this.dof * this.V) : 0; }
  temperature() { return this.n === 0 ? 0 : this.E / (0.5 * this.dof * this.n * ES_R); }
  mass() { return ES_AIR_MOLAR_MASS * this.n; }
  energyPerMol() { return this.E / this.n; }
  density() { return ES_AIR_MOLAR_MASS * this.n / this.V; }
  c() {
    if (this.n === 0 || this.E === 0) return 0;
    return Math.sqrt(this.pressure() * ES_HCR5 / this.density());
  }
  bulkKineticEnergy() {
    const m = ES_AIR_MOLAR_MASS * this.n; if (m === 0) return 0;
    return (this.px * this.px + this.py * this.py) / (2 * m);
  }
  totalEnergy() { return this.n === 0 ? 0 : this.E + this.bulkKineticEnergy(); }
  dynamicPressure(dx, dy) {
    if (this.n === 0 || this.E === 0) return 0;
    const m = ES_AIR_MOLAR_MASS * this.n;
    const v = (dx * this.px + dy * this.py) / m;
    if (v <= 0) return 0;
    const P = this.E / (0.5 * this.dof * this.V);
    return esStagnation(P, 0.2 * v * v * m / (P * ES_HCR5 * this.V));
  }
  setVolume(V) { this.changeVolume(V - this.V); }
  changeVolume(dV) {
    const L = Math.pow(this.V + dV, 1 / 3);
    const surfaceArea = L * L;
    const W = (-dV / surfaceArea) * this.pressure() * surfaceArea;
    this.V += dV; this.E += W;
  }
  changeEnergy(dE) { this.E += dE; }
  react(n, mixFuel, mixO2) {
    const lFuel = mixFuel * n, lO2 = mixO2 * n;
    const sysFuel = this.fuel * this.n, sysO2 = this.o2 * this.n, sysInert = this.inert * this.n, sysN = this.n;
    // 25[O2] + 2[C8H16] -> 16[CO2] + 18[H2O]
    const aFuel = Math.min(sysFuel, lFuel, (2 / 25) * lO2);
    const aO2 = Math.min(sysO2, lO2, (25 / 2) * lFuel);
    const reactants = aFuel + aO2;
    const products = (34 / 27) * reactants;
    const dn = products - reactants;
    this.n += dn;
    const newN = sysN + dn;
    if (newN !== 0) {
      this.fuel = (sysFuel - aFuel) / newN; this.inert = (sysInert + products) / newN; this.o2 = (sysO2 - aO2) / newN;
    } else this.fuel = this.inert = this.o2 = 0;
    return aFuel;
  }
  loseN(dn, ePerMol) {
    this.E -= ePerMol * dn; this.n -= dn;
    if (this.n < 0) this.n = 0;
  }
  gainN(dn, ePerMol, fuel, inert, o2) {
    const next = this.n + dn, current = this.n;
    this.E += dn * ePerMol; this.n = next;
    if (next !== 0) {
      this.fuel = (this.fuel * current + dn * fuel) / next;
      this.inert = (this.inert * current + dn * inert) / next;
      this.o2 = (this.o2 * current + dn * o2) / next;
    } else this.fuel = this.inert = this.o2 = 0;
  }
  dissipateExcessVelocity() {
    const m = this.mass(); if (m === 0) return;
    const vx = this.px / m, vy = this.py / m, v2 = vx * vx + vy * vy;
    const c = this.c(), c2 = c * c;
    if (c2 >= v2 || v2 === 0) return;
    const k = Math.sqrt(c2 / v2);
    this.px *= k; this.py *= k;
    this.E += 0.5 * m * (v2 - c2);
    if (this.E < 0) this.E = 0;
  }
  updateVelocity(dt, beta = 1) {
    if (this.n === 0) return;
    const depth = this.V / (this.width * this.height);
    const dx = this.dx, dy = this.dy;
    const m = ES_AIR_MOLAR_MASS * this.n, inv = 1 / m;
    let p0 = 0, p1 = 0, p2 = 0, p3 = 0;
    if (this.E !== 0) {
      // Four dynamicPressure() calls sharing the static pressure and sound speed.
      const P = this.E / (0.5 * this.dof * this.V), k = 0.2 * m / (P * ES_HCR5 * this.V);
      const va = (dx * this.px + dy * this.py) * inv, vb = (dy * this.px + dx * this.py) * inv;
      if (va > 0) p0 = esStagnation(P, k * va * va); else if (va < 0) p1 = esStagnation(P, k * va * va);
      if (vb > 0) p2 = esStagnation(P, k * vb * vb); else if (vb < 0) p3 = esStagnation(P, k * vb * vb);
      p0 *= this.height * depth; p1 *= this.height * depth; p2 *= this.width * depth; p3 *= this.width * depth;
    }
    const dpx = p0 * dx - p1 * dx + p2 * dy - p3 * dy;
    const dpy = p0 * dy - p1 * dy + p2 * dx - p3 * dx;
    const v0x = this.px * inv, v0y = this.py * inv;
    this.px -= dpx * dt * beta; this.py -= dpy * dt * beta;
    const v1x = this.px * inv, v1y = this.py * inv;
    this.E -= 0.5 * m * (v1x * v1x - v0x * v0x);
    this.E -= 0.5 * m * (v1y * v1y - v0y * v0y);
    if (this.E < 0) this.E = 0;
  }
  // GasSystem::flow(k_flow, dt, P_env, T_env, mix): exchange with an infinite reservoir.
  flowToEnvironment(k, dt, Penv, Tenv) {
    if (k === 0) return 0;
    let maxFlow;
    if (this.pressure() > Penv) maxFlow = -(Penv * (0.5 * this.dof * this.V) - this.E) / this.energyPerMol();
    else maxFlow = -(Penv * (0.5 * this.dof * this.V) - this.E) / (0.5 * Tenv * ES_R * this.dof);
    let flow = dt * esFlowRate(k, this.pressure(), Penv, this.temperature(), Tenv);
    if (Math.abs(flow) > Math.abs(maxFlow)) flow = maxFlow;
    if (flow < 0) {
      const b0 = this.bulkKineticEnergy();
      this.gainN(-flow, 0.5 * Tenv * ES_R * this.dof, 0, 1, 0);
      this.E += this.bulkKineticEnergy() - b0;
    } else {
      const start = this.n;
      this.loseN(flow, this.energyPerMol());
      this.px -= (flow / start) * this.px; this.py -= (flow / start) * this.py;
    }
    return flow;
  }
}

function esFlowRate(k, P0, P1, T0, T1) {
  if (k === 0) return 0;
  let direction, T, p0, pT;
  if (P0 > P1) { direction = 1; T = T0; p0 = P0; pT = P1; }
  else { direction = -1; T = T1; p0 = P1; pT = P0; }
  const ratio = pT / p0;
  let rate;
  if (ratio <= ES_CHOKED_LIMIT5) rate = ES_CHOKED_RATE5;
  else {
    // sqrt(2*hcr/(hcr-1) * s*(s-ratio)) with s = ratio^(1/hcr), tabulated against
    // u = sqrt(1 - ratio), where it is smooth, instead of calling Math.pow.
    const x = Math.sqrt(1 - ratio) * ES_SUBSONIC_SCALE, i = x | 0, f = x - i;
    rate = ES_SUBSONIC[i] + (ES_SUBSONIC[i + 1] - ES_SUBSONIC[i]) * f;
  }
  return rate / Math.sqrt(ES_R * T) * direction * p0 * k;
}

function esFlowConstant(targetFlowRate, P, pressureDrop, T) {
  const hcr = ES_HCR5, ratio = (P - pressureDrop) / P;
  let rate;
  if (ratio <= ES_CHOKED_LIMIT5) rate = ES_CHOKED_RATE5;
  else rate = Math.sqrt((2 * hcr) / (hcr - 1) * (1 - Math.pow(ratio, (hcr - 1) / hcr))) * Math.pow(ratio, 1 / hcr);
  rate *= P / Math.sqrt(ES_R * T);
  return targetFlowRate / rate;
}
const esKCarb = scfm => esFlowConstant(scfm * ES_UNITS.scfm, ES_ATM, 1.5 * ES_UNITS.inHg, ES_T25);
const esK28 = scfm => esFlowConstant(scfm * ES_UNITS.scfm, ES_ATM, 28 * ES_UNITS.inH2O, ES_T25);

// GasSystem::flow(FlowParameters): flow between two finite volumes, with momentum.
function esFlow(k, dt, dirX, dirY, area0, area1, s0, s1) {
  const P0 = s0.pressure() + s0.dynamicPressure(dirX, dirY);
  const P1 = s1.pressure() + s1.dynamicPressure(-dirX, -dirY);
  let source, sink, dx, dy, srcP, sinkP, srcArea, sinkArea, direction;
  if (P0 > P1) { source = s0; sink = s1; dx = dirX; dy = dirY; srcP = P0; sinkP = P1; srcArea = area0; sinkArea = area1; direction = 1; }
  else { source = s1; sink = s0; dx = -dirX; dy = -dirY; srcP = P1; sinkP = P0; srcArea = area1; sinkArea = area0; direction = -1; }

  let flow = dt * esFlowRate(k, srcP, sinkP, source.temperature(), sink.temperature());
  flow = esClamp(flow, 0, 0.9 * source.n);
  const fraction = source.n === 0 ? 0 : flow / source.n;
  const fractionVolume = fraction * source.V;
  const fractionMass = fraction * source.mass();

  if (flow !== 0) {
    const bulkSrc0 = source.bulkKineticEnergy(), bulkSink0 = sink.bulkKineticEnergy();
    const ePerMol = source.energyPerMol();
    sink.gainN(flow, ePerMol, source.fuel, source.inert, source.o2);
    source.loseN(flow, ePerMol);
    const dpx = source.px * fraction, dpy = source.py * fraction;
    source.px -= dpx; source.py -= dpy; sink.px += dpx; sink.py += dpy;
    sink.E -= (source.bulkKineticEnergy() + sink.bulkKineticEnergy()) - (bulkSrc0 + bulkSink0);
  }

  const sourceMass = source.mass(), sinkMass = sink.mass();
  const cSource = source.c(), cSink = sink.c();
  const src0x = source.px, src0y = source.py, sink0x = sink.px, sink0y = sink.py;

  if (sinkArea !== 0) {
    const v = esClamp((fractionVolume / sinkArea) / dt, 0, cSink);
    sink.px += v * dx * fractionMass; sink.py += v * dy * fractionMass;
  }
  if (srcArea !== 0 && sourceMass !== 0) {
    const v = esClamp((fractionVolume / srcArea) / dt, 0, cSource);
    source.px += v * dx * fractionMass; source.py += v * dy * fractionMass;
  }
  if (sourceMass !== 0) {
    const inv = 1 / sourceMass;
    const v0x = src0x * inv, v0y = src0y * inv, v1x = source.px * inv, v1y = source.py * inv;
    source.E -= 0.5 * sourceMass * (v1x * v1x - v0x * v0x);
    source.E -= 0.5 * sourceMass * (v1y * v1y - v0y * v0y);
  }
  if (sinkMass > 0) {
    const inv = 1 / sinkMass;
    const v0x = sink0x * inv, v0y = sink0y * inv, v1x = sink.px * inv, v1y = sink.py * inv;
    sink.E -= 0.5 * sinkMass * (v1x * v1x - v0x * v0x);
    sink.E -= 0.5 * sinkMass * (v1y * v1y - v0y * v0y);
  }
  if (sink.E < 0) sink.E = 0;
  if (source.E < 0) source.E = 0;
  return flow * direction;
}

/* ------------------------------------------------------- Intake and exhaust */

class ESIntake {
  constructor(p) {
    this.p = p;
    this.system = new ESGasSystem();
    this.system.initialize(ES_ATM, p.plenumVolume, ES_T25);
    this.system.setGeometry(Math.sqrt(p.plenumArea), p.plenumVolume / p.plenumArea, 1, 0);
    this.atmosphere = new ESGasSystem();
    this.atmosphere.initialize(ES_ATM, 1000, ES_T25);
    this.atmosphere.setGeometry(100, 100, 1, 0);
    this.inputK = esKCarb(p.flowRate); this.idleK = esKCarb(p.idleFlowRate || 0);
    this.runnerK = esKCarb(p.runnerFlowRate);
    this.throttle = 1; // throttle plate linkage position (1 = closed)
    this.molecularAfr = 12.5; this.afrScale = 1;
    this.flow = 0; this.flowAccumulator = 0; this.fuelInjected = 0;
  }
  platePosition() { return this.p.idlePlate * this.throttle; }
  process(dt) {
    const idealAfr = 0.8 * this.molecularAfr * 4 * this.afrScale;
    const pAir = idealAfr / (1 + idealAfr);
    const attenuation = Math.cos(this.platePosition() * Math.PI / 2);
    this.atmosphere.reset(ES_ATM, ES_T25, 1 - pAir, pAir * 0.75, pAir * 0.25);
    this.flow = esFlow(attenuation * this.inputK, dt, 0, -1, 10, this.p.plenumArea, this.atmosphere, this.system);
    if (this.idleK !== 0) {
      const pIdle = 2 / 3;
      this.atmosphere.reset(ES_ATM, ES_T25, 1 - pIdle, pIdle * 0.75, pIdle * 0.25);
      const idle = esFlow(this.idleK, dt, 0, -1, 10, this.p.plenumArea, this.atmosphere, this.system);
      if (idle > 0) this.fuelInjected += (1 - pIdle) * idle;
    }
    this.system.dissipateExcessVelocity();
    this.system.updateVelocity(dt, this.p.velocityDecay);
    if (this.flow > 0) this.fuelInjected += (1 - pAir) * this.flow;
    this.flowAccumulator += this.flow;
  }
}

class ESExhaustSystem {
  constructor(p, length, audioVolume) {
    this.p = p; this.length = length; this.audioVolume = audioVolume;
    const volume = p.collectorArea * length;
    this.system = new ESGasSystem();
    this.system.initialize(ES_ATM, volume, ES_T25);
    this.system.setGeometry(length, Math.sqrt(p.collectorArea), 1, 0);
    this.atmosphere = new ESGasSystem();
    this.atmosphere.initialize(ES_ATM, 1000, ES_T25);
    this.atmosphere.setGeometry(10, 10, 1, 0);
    this.outletK = esKCarb(p.outletFlowRate); this.primaryK = esKCarb(p.primaryFlowRate);
  }
  process(dt) {
    this.atmosphere.reset(ES_ATM, ES_T25, 0, 1, 0);
    esFlow(this.outletK, dt, 1, 0, this.p.collectorArea, 10, this.atmosphere, this.system);
    this.system.dissipateExcessVelocity();
    this.system.updateVelocity(dt, this.p.velocityDecay);
  }
}

/* ---------------------------------------------------------------------- Fuel */

class ESFuel {
  constructor(p = {}) {
    this.molecularMass = 0.1; this.energyDensity = 48.1e6; this.molecularAfr = 12.5;
    this.burningEfficiencyRandomness = 0.5; this.lowEfficiencyAttenuation = 0.6;
    this.maxBurningEfficiency = 0.8; this.maxTurbulenceEffect = 2.0; this.maxDilutionEffect = 10.0;
    Object.assign(this, p);
    const ratio = p.turbulenceToFlameSpeedRatio || [3, 7.5, 15, 22.5, 30, 37.5, 45, 52.5, 60, 67.5];
    this.turbulenceToFlameSpeed = new ESTable(ratio.map((_, i) => i * 5), ratio);
  }
  laminarBurningVelocity(molecularAfr, T, P) {
    const er = molecularAfr / this.molecularAfr;
    const alpha = 2.4 - 0.271 * Math.pow(er, 3.51);
    const beta = -0.357 + 0.14 * Math.pow(er, 2.77);
    const S0 = 0.305 - 0.549 * (er - 1.21) * (er - 1.21);
    return S0 * Math.pow(T / 298, alpha) * Math.pow(P / ES_ATM, beta);
  }
  flameSpeed(turbulence, molecularAfr, T, P) {
    const SL = this.laminarBurningVelocity(molecularAfr, T, P);
    return this.turbulenceToFlameSpeed.sample(turbulence / SL) * SL;
  }
}

/* --------------------------------------------------------------- Valvetrain */

// GenerateHarmonicCamLobeNode::_evaluate (scripting/include/actions.h), tabulated.
function esHarmonicLobeTable(durationDeg, lift, gamma = 1.1, steps = 256) {
  const thou50 = 50 * ES_UNITS.thou;
  const angle = durationDeg * ES_UNITS.deg / 4;
  const s = Math.pow(2 * thou50 / lift, 1 / gamma) - 1;
  const k = Math.acos(s) / angle, extents = Math.PI / k;
  const N = 2048, xs = new Float64Array(N), ys = new Float64Array(N);
  for (let i = 0; i < N; i++) {
    const x = -Math.PI + ES_TAU * i / (N - 1), ax = Math.abs(x);
    xs[i] = x;
    ys[i] = ax >= extents ? 0 : lift * Math.pow(0.5 + 0.5 * Math.cos(k * ax), gamma);
  }
  return new ESTable(xs, ys);
}

/* ------------------------------------------------------- Combustion chamber */

class ESChamber {
  constructor(engine, index) {
    const spec = engine.spec;
    this.engine = engine; this.index = index; this.number = index + 1;
    this.system = new ESGasSystem(); this.intakeRunner = new ESGasSystem(); this.exhaustRunner = new ESGasSystem();
    this.lit = false; this.flame = { travelX: 0, travelY: 0, lastVolume: 0, efficiency: 1, flameSpeed: 0, fuel: 0, o2: 0, totalN: 0 };
    this.intakeK = 0; this.exhaustK = 0; this.intakeLift = 0; this.exhaustLift = 0;
    this.exhaustFlow = 0; this.stepIntakeFlow = 0; this.stepExhaustFlow = 0;
    this.blowbyK = esK28((spec.blowby && spec.blowby[index]) || 0);
    this.soundAttenuation = (spec.soundAttenuation && spec.soundAttenuation[index]) || 1;
    this.headerLength = (spec.primaryLengths && spec.primaryLengths[index]) || 0;
    this.sparkTimer = 0;
  }
  initialize() {
    const e = this.engine, spec = e.spec, head = spec.head;
    this.exhaust = e.exhaustFor(this.index);
    const volume = e.chamberVolumeAt(this.index);
    const area = e.boreArea;
    this.system.initialize(ES_ATM, volume, ES_T25);
    this.system.setGeometry(Math.sqrt(area), volume / area, 1, 0);
    const iArea = head.intakeRunnerArea, iVol = head.intakeRunnerVolume + iArea * spec.intake.runnerLength;
    this.intakeRunner.initialize(ES_ATM, iVol, ES_T25);
    this.intakeRunner.setGeometry(iVol / iArea, Math.sqrt(iArea), 1, 0);
    const eArea = head.exhaustRunnerArea;
    const eVol = head.exhaustRunnerVolume + eArea * (spec.exhaust.primaryTubeLength + this.headerLength);
    this.exhaustRunner.initialize(ES_ATM, eVol, ES_T25);
    this.exhaustRunner.setGeometry(eVol / eArea, Math.sqrt(eArea), 1, 0);
  }
  ignite() {
    if (this.lit || this.system.fuel === 0) return;
    const e = this.engine, fuel = e.fuel, sys = this.system;
    const afr = sys.o2 / sys.fuel, equivalence = afr / fuel.molecularAfr;
    if (equivalence < 0.5 || equivalence > 1.9) return;
    const dilution = sys.inert / (sys.o2 / 0.7) - 1;
    const f = this.flame;
    f.lastVolume = sys.V; f.travelX = 0; f.travelY = 0; f.totalN = sys.n; f.fuel = sys.fuel; f.o2 = sys.o2;
    this.lit = true; this.sparkTimer = 0.03;
    const turbulence = e.turbulence.sample(e.meanPistonSpeed());
    const mixing = 1 - esClamp(turbulence / fuel.maxTurbulenceEffect) * esClamp(1 - dilution / fuel.maxDilutionEffect);
    const r = fuel.lowEfficiencyAttenuation * ((1 - fuel.burningEfficiencyRandomness) + fuel.burningEfficiencyRandomness * Math.random());
    f.efficiency = (mixing * r + (1 - mixing)) * fuel.maxBurningEfficiency;
    f.flameSpeed = fuel.flameSpeed(turbulence, afr, sys.temperature(), sys.pressure());
  }
  update() {
    const e = this.engine;
    this.system.setVolume(e.chamberVolumeAt(this.index));
    this.intakeLift = e.intakeLift(this.index); this.exhaustLift = e.exhaustLift(this.index);
    this.intakeK = e.intakePortFlow.sample(this.intakeLift);
    this.exhaustK = e.exhaustPortFlow.sample(this.exhaustLift);
  }
  flow(dt) {
    const e = this.engine, spec = e.spec, sys = this.system, intake = e.intake, exhaust = this.exhaust;
    const volume = sys.V, area = e.boreArea, height = volume / area;
    const surface = height * Math.PI * spec.bore + area * 2;
    sys.changeEnergy((363.15 - sys.temperature()) * surface * 100 * dt);
    if (this.blowbyK !== 0) sys.flowToEnvironment(this.blowbyK, dt, ES_ATM, ES_T25);

    const iArea = spec.head.intakeRunnerArea, eArea = spec.head.exhaustRunnerArea;
    esFlow(intake.runnerK, dt, 1, 0, spec.intake.plenumArea, iArea, intake.system, this.intakeRunner);
    this.intakeRunner.dissipateExcessVelocity();
    const intakeFlow = esFlow(this.intakeK, dt, 1, 0, iArea, volume / height, this.intakeRunner, sys);
    this.intakeRunner.dissipateExcessVelocity(); sys.dissipateExcessVelocity();
    const exhaustFlow = esFlow(this.exhaustK, dt, 1, 0, volume / height, eArea, sys, this.exhaustRunner);
    sys.dissipateExcessVelocity(); this.exhaustRunner.dissipateExcessVelocity();
    esFlow(exhaust.primaryK, dt, 1, 0, eArea, spec.exhaust.collectorArea, this.exhaustRunner, exhaust.system);

    this.intakeRunner.updateVelocity(dt, spec.intake.velocityDecay);
    sys.updateVelocity(dt, 0.5);
    this.exhaustRunner.updateVelocity(dt, spec.exhaust.velocityDecay);

    if (Math.abs(intakeFlow) > 1e-9 && this.lit) this.lit = false;
    this.exhaustFlow = exhaustFlow;
    this.stepExhaustFlow += exhaustFlow; this.stepIntakeFlow += intakeFlow;

    if (this.lit) {
      const f = this.flame;
      const totalX = spec.bore / 2, totalY = volume / area;
      const lastX = f.travelX, lastY = f.travelY * (volume / f.lastVolume);
      f.travelX = Math.min(lastX + dt * f.flameSpeed, totalX);
      f.travelY = Math.min(lastY + dt * f.flameSpeed, totalY);
      if (lastX < f.travelX || lastY < f.travelY) {
        const burned = f.travelX * f.travelX * Math.PI * f.travelY;
        const previous = lastX * lastX * Math.PI * lastY;
        const n = ((burned - previous) / volume) * sys.n;
        const fuelBurned = sys.react(n * f.efficiency, f.fuel, f.o2);
        sys.changeEnergy(fuelBurned * e.fuel.molecularMass * e.fuel.energyDensity);
      } else this.lit = false;
      f.lastVolume = volume;
    }
  }
}

/* -------------------------------------------------------------- Synthesizer */

class ESButterworth {
  constructor(cutoff, sampleRate) { this.x = [0, 0, 0, 0]; this.y = [0, 0, 0, 0]; this.a = [0, 0, 0, 0, 0]; this.f4 = 0; this.setCutoff(cutoff, sampleRate); }
  setCutoff(fc, rate) {
    const f = Math.tan(Math.PI * Math.min(fc, rate * 0.49) / rate), f2 = f * f, f3 = f2 * f, f4 = f2 * f2;
    const m = -2 * Math.cos(5 * Math.PI / 8), n = -2 * Math.cos(7 * Math.PI / 8), a = this.a;
    a[0] = 1 + (m + n) * f + (2 + n * m) * f2 + (m + n) * f3 + f4;
    a[1] = (-4 - 2 * (n + m) * f + 2 * (m + n) * f3 + 4 * f4) / a[0];
    a[2] = (6 - 2 * (2 + m * n) * f2 + 6 * f4) / a[0];
    a[3] = (-4 + 2 * (m + n) * f - 2 * (m + n) * f3 + 4 * f4) / a[0];
    a[4] = (1 - (n + m) * f + (2 + m * n) * f2 - (m + n) * f3 + f4) / a[0];
    this.f4 = f4;
  }
  f(sample) {
    const x = this.x, y = this.y, a = this.a;
    const out = this.f4 / a[0] * (sample + 4 * x[3] + 6 * x[2] + 4 * x[1] + x[0])
      - a[1] * y[3] - a[2] * y[2] - a[3] * y[1] - a[4] * y[0];
    x[0] = x[1]; x[1] = x[2]; x[2] = x[3]; x[3] = sample;
    y[0] = y[1]; y[1] = y[2]; y[2] = y[3]; y[3] = out;
    return out;
  }
}

class ESJitter {
  constructor(maxJitter, cutoff, rate) { this.history = new Float64Array(maxJitter); this.max = maxJitter; this.offset = 0; this.scale = 0; this.noise = new ESButterworth(cutoff, rate); }
  f(sample) {
    const h = this.history, max = this.max;
    h[this.offset] = sample; if (++this.offset >= max) this.offset = 0;
    const s = this.noise.f(Math.random() * (max - 1) * this.scale);
    const i0 = esClamp(Math.floor(s), 0, max - 1), i1 = esClamp(Math.ceil(s), 0, max - 1), frac = s - i0;
    const j0 = i0 + this.offset, j1 = i1 + this.offset;
    return h[j1 >= max ? j1 - max : j1] * frac + h[j0 >= max ? j0 - max : j0] * (1 - frac);
  }
}

// Synthesizer (src/synthesizer.cpp) without the convolution stage, which runs in
// a Web Audio ConvolverNode after this. Output is two channels in [-1, 1].
class ESSynthesizer {
  constructor(audioRate, inputRate, channels = 2) {
    this.audioRate = audioRate; this.channels = channels;
    this.params = { airNoise: 1, airNoiseCutoff: 2000, dFFMix: 0.01, jitter: 0.5, jitterCutoff: 10000, levelerTarget: 30000, levelerMax: 1.9, levelerMin: 0.00001 };
    this.setInputRate(inputRate);
    this.channelState = Array.from({ length: channels }, () => ({
      last: 0, antialiasing: new ESButterworth(1900, audioRate), jitter: new ESJitter(10, 10000, audioRate),
      dcY: 0, previous: 0,
    }));
    this.airNoise = new ESButterworth(2000, audioRate);
    this.outputAntialiasing = [new ESButterworth(audioRate * 0.45, audioRate), new ESButterworth(audioRate * 0.45, audioRate)];
    this.peak = 30000; this.attenuation = 1;
    this.position = 0; // fractional audio-sample position of the last input
    this.out = [[], []];
  }
  setInputRate(rate) { this.inputRate = rate; this.ratio = this.audioRate / rate; }
  setParameters(p) { Object.assign(this.params, p); this.airNoise.setCutoff(this.params.airNoiseCutoff, this.audioRate); }
  writeInput(values) {
    const next = this.position + this.ratio;
    const first = Math.floor(this.position) + 1, last = Math.floor(next);
    const resampled = this._scratch || (this._scratch = new Float64Array(this.channels));
    for (let k = first; k <= last; k++) {
      const f = (k - this.position) / this.ratio;
      for (let c = 0; c < this.channels; c++) {
        const st = this.channelState[c];
        resampled[c] = st.antialiasing.f(st.last * (1 - f) + values[c] * f);
      }
      this._render(resampled);
    }
    for (let c = 0; c < this.channels; c++) this.channelState[c].last = values[c];
    this.position = next - last;
  }
  _render(input) {
    const p = this.params, rate = this.audioRate, dt = 1 / rate;
    const rcDc = 1 / (10 * ES_TAU), alphaDc = dt / (rcDc + dt);
    const r = this.airNoise.f(2 * Math.random() - 1), rMixed = p.airNoise * r + (1 - p.airNoise);
    let left = 0, right = 0;
    for (let c = 0; c < this.channels; c++) {
      const st = this.channelState[c];
      st.jitter.scale = p.jitter;
      const fIn = st.jitter.f(input[c]);
      st.dcY = alphaDc * fIn + (1 - alphaDc) * st.dcY;
      const f = fIn - st.dcY;
      const fP = (fIn - st.previous) / dt; st.previous = fIn;
      const v = fP * p.dFFMix + f * rMixed * (1 - p.dFFMix);
      if (this.channels === 1 || this.mono) { left += v; right += v; }
      else if (c % 2 === 0) { left += v; right += 0.35 * v; } else { right += v; left += 0.35 * v; }
    }
    left = this.outputAntialiasing[0].f(left); right = this.outputAntialiasing[1].f(right);
    // LevelingFilter (src/leveling_filter.cpp), shared by both channels.
    this.peak *= 0.999;
    const peak = Math.max(Math.abs(left), Math.abs(right));
    if (peak > this.peak) this.peak = peak;
    if (this.peak > 0) {
      const raw = esClamp(p.levelerTarget / this.peak, p.levelerMin, p.levelerMax);
      this.attenuation = 0.9 * this.attenuation + 0.1 * raw;
    }
    this.out[0].push(esClamp(left * this.attenuation / 32768, -1, 1));
    this.out[1].push(esClamp(right * this.attenuation / 32768, -1, 1));
  }
  drain() { const out = this.out; this.out = [[], []]; return out; }
}

/* ------------------------------------------------------------------ Engines */

const esDisk = (mass, radius) => 0.5 * mass * radius * radius;
const U = ES_UNITS;
const ES_ENGINES = {
  // Honda B18C5 (assets/engines/atg-video-1/05_honda_vtec.mr) on its low-speed cam, without VTEC switching.
  i4: {
    name: 'Honda B18 · Inline 4', cylinders: 4, banks: 1, bankAngle: 0,
    bore: 81 * U.mm, stroke: 87.2 * U.mm, rod: 5.43 * U.inch,
    pistonMass: 303.5 * U.g, rodMass: 395.837 * U.g,
    crankInertia: 0.22986844776863666 * 0.5, crankFriction: 1 * U.lbft,
    starterTorque: 70 * U.lbft, starterSpeed: 500, redline: 8000, revLimit: 8400, limiterDuration: 0.05,
    firingOrder: [1, 3, 4, 2], compressionRatio: 11.8,
    cam: { duration: 210, lift: 7 * U.mm, gamma: 1.0, intakeCenter: 116, exhaustCenter: 116 },
    head: {
      intakeRunnerVolume: 149.6 * U.cc, intakeRunnerArea: (1.35 * U.inch) ** 2,
      exhaustRunnerVolume: 50 * U.cc, exhaustRunnerArea: (1.25 * U.inch) ** 2,
      intakeFlow: [0, 50, 80, 125, 160, 190, 210, 225, 230, 250], exhaustFlow: [0, 50, 80, 110, 130, 150, 160, 170, 170, 170],
    },
    intake: { plenumVolume: 1.325 * U.L, plenumArea: 20 * U.cm2, flowRate: 800, runnerFlowRate: 250, runnerLength: 7 * U.inch, idleFlowRate: 0, idlePlate: 0.9989, velocityDecay: 0.5, throttleGamma: 2 },
    exhaust: { outletFlowRate: 1000, primaryTubeLength: 10 * U.inch, primaryFlowRate: 200, velocityDecay: 1, collectorArea: Math.PI * (2 * U.inch) ** 2, lengths: [100 * U.L / (Math.PI * (2 * U.inch) ** 2), 100 * U.L / (Math.PI * (2 * U.inch) ** 2)], audioVolume: [6, 8] },
    blowby: [0.001, 0.002, 0.001, 0.002], soundAttenuation: [0.9, 1.1, 0.8, 0.9],
    fuel: { maxTurbulenceEffect: 2.5, maxBurningEfficiency: 0.75 },
    timing: [[0, 25], [1000, 25], [2000, 30], [3000, 40], [4000, 40]],
    vehicle: { mass: 2400 * U.lb, dragCoefficient: 0.2, area: 66 * 50 * U.inch * U.inch, diffRatio: 3.55, tireRadius: 10 * U.inch, rollingResistance: 300 },
    transmission: { gears: [3.23, 2.105, 1.458, 1.107, 0.848], maxClutchTorque: 300 * U.lbft },
    simulationFrequency: 20000, audio: { dFFMix: 0.002, airNoise: 0.253, jitter: 0.195 },
  },
  // Generic 60 degree V6 (assets/engines/atg-video-2/04_60_degree_v6.mr).
  v6: {
    name: 'V6 60°', cylinders: 6, banks: 2, bankAngle: 60,
    bore: 3.5 * U.inch, stroke: 3.48 * U.inch, rod: 5.142 * U.inch,
    pistonMass: (414 + 152) * U.g, rodMass: 535 * U.g,
    crankInertia: esDisk(50 * U.lb, 3.48 * U.inch) + esDisk(30 * U.lb, 7 * U.inch) + esDisk(10, 0.01), crankFriction: 20 * U.lbft,
    starterTorque: 70 * U.lbft, starterSpeed: 500, redline: 5500, revLimit: 5600, limiterDuration: 0.2,
    firingOrder: [1, 2, 3, 4, 5, 6], compressionRatio: 9.2,
    cam: { duration: 222, lift: 400 * U.thou, gamma: 1.0, intakeCenter: 117, exhaustCenter: 112 },
    head: {
      intakeRunnerVolume: 149.6 * U.cc, intakeRunnerArea: (1.35 * U.inch) ** 2,
      exhaustRunnerVolume: 50 * U.cc, exhaustRunnerArea: (2 * U.inch) ** 2,
      intakeFlow: [0, 58, 103, 156, 214, 249, 268, 280, 280, 281], exhaustFlow: [0, 37, 72, 113, 160, 196, 222, 235, 245, 246],
    },
    intake: { plenumVolume: 1.325 * U.L, plenumArea: 20 * U.cm2, flowRate: 400, runnerFlowRate: 250, runnerLength: 4 * U.inch, idleFlowRate: 0, idlePlate: 0.994, velocityDecay: 0.5, throttleGamma: 2 },
    exhaust: { outletFlowRate: 1000, primaryTubeLength: 20 * U.inch, primaryFlowRate: 500, velocityDecay: 1, collectorArea: Math.PI * (2 * U.inch) ** 2, lengths: [100 * U.inch, 100 * U.inch], audioVolume: [1, 1] },
    blowby: [0.1, 0.1, 0.2, 0.2, 0.2, 0.2], soundAttenuation: [0.9, 0.95, 0.95, 0.9, 1, 1],
    primaryLengths: [10 * U.inch, 10 * U.inch, 5 * U.inch, 5 * U.inch, 0, 0],
    fuel: {},
    timing: [[0, 12], [1000, 20], [2000, 25], [3000, 30], [4000, 30]],
    vehicle: { mass: 2700 * U.lb, dragCoefficient: 0.3, area: 72 * 56 * U.inch * U.inch, diffRatio: 3.9, tireRadius: 10 * U.inch, rollingResistance: 0.015 * 2700 * U.lb * 9.81 },
    transmission: { gears: [3.636, 2.375, 1.761, 1.346, 0.971, 0.756], maxClutchTorque: 300 * U.lbft },
    simulationFrequency: 10000, audio: { dFFMix: 0.01, airNoise: 1, jitter: 0.5 },
  },
  // GM LS (assets/engines/atg-video-2/07_gm_ls.mr).
  v8: {
    name: 'GM LS V8', cylinders: 8, banks: 2, bankAngle: 90,
    bore: 3.78 * U.inch, stroke: 3.622 * U.inch, rod: 160 * U.mm,
    pistonMass: 100 * U.g, rodMass: 50 * U.g,
    crankInertia: 1.5 * esDisk(60 * U.lb, 3.622 * U.inch) + esDisk(30 * U.lb, 8 * U.inch) + esDisk(1, 0.01), crankFriction: 20 * U.lbft,
    starterTorque: 200 * U.lbft, starterSpeed: 200, redline: 6500, revLimit: 6800, limiterDuration: 0.2,
    firingOrder: [1, 8, 7, 2, 6, 5, 4, 3], compressionRatio: 8.4,
    cam: { duration: 234, lift: 551 * U.thou, gamma: 1.1, intakeCenter: 116, exhaustCenter: 116 },
    head: {
      intakeRunnerVolume: 149.6 * U.cc, intakeRunnerArea: (2.2 * U.inch) ** 2,
      exhaustRunnerVolume: 50 * U.cc, exhaustRunnerArea: (1.75 * U.inch) ** 2,
      intakeFlow: [0, 1, 103, 156, 214, 249, 268, 280, 280, 281], exhaustFlow: [0, 1, 72, 113, 160, 196, 222, 235, 245, 246],
    },
    intake: { plenumVolume: 1.325 * U.L, plenumArea: 20 * U.cm2, flowRate: 700, runnerFlowRate: 100, runnerLength: 12 * U.inch, idleFlowRate: 0, idlePlate: 0.996, velocityDecay: 0.5, throttleGamma: 2 },
    exhaust: { outletFlowRate: 1000, primaryTubeLength: 29 * U.inch, primaryFlowRate: 500, velocityDecay: 1, collectorArea: Math.PI * (2 * U.inch) ** 2, lengths: [100 * U.inch, 172 * U.inch], audioVolume: [4, 4] },
    blowby: [], soundAttenuation: [],
    primaryLengths: [6 * U.inch + 2 * U.cm, 6 * U.inch + 1 * U.cm, 4 * U.inch + 1 * U.cm, 4 * U.inch + 5 * U.cm, 2 * U.inch + 3 * U.cm, 2 * U.inch + 7 * U.cm, 5 * U.cm, 0],
    fuel: { maxBurningEfficiency: 1.0, turbulenceToFlameSpeedRatio: [3, 7.5, 17.5, 30, 40, 50, 60, 70, 80, 90] },
    timing: [[0, 12], [1000, 12], [2000, 20], [3000, 30], [4000, 40], [8000, 40]],
    vehicle: { mass: 1614, dragCoefficient: 0.3, area: 72 * 50 * U.inch * U.inch, diffRatio: 3.42, tireRadius: 10 * U.inch, rollingResistance: 200 },
    transmission: { gears: [2.97, 2.07, 1.43, 1.0, 0.71, 0.57], maxClutchTorque: 500 * U.lbft },
    simulationFrequency: 10000, audio: { dFFMix: 0.01, airNoise: 1, jitter: 0.6 },
  },
};

/* ---------------------------------------------------------------- Simulator */

const ES_TORQUE_BINS = 512;

class ESEngineSimulation {
  constructor(spec, options = {}) {
    this.spec = spec;
    this.theta = 0; this.omega = 0;
    this.controls = {
      throttle: 0, ignition: false, starter: false, load: 0, brake: 0, gear: 0, clutch: 1,
      dyno: { enabled: false, hold: false, rpm: 0 },
    };
    this.vehicleSpeed = 0;
    this.revLimitTimer = 0;
    this.sparks = new Float64Array(spec.cylinders);
    this.ignitionEvents = 0;
    this.time = 0;
    this.torqueBins = new Float64Array(ES_TORQUE_BINS); this.dynoBins = new Float64Array(ES_TORQUE_BINS); this.lastBin = 0;
    this.trace = { pressure: new Float64Array(720), volume: new Float64Array(720), intake: new Float64Array(720), exhaust: new Float64Array(720), moles: new Float64Array(720), temperature: new Float64Array(720), intakeLift: new Float64Array(720), exhaustLift: new Float64Array(720) };
    this.configure(Object.assign({
      firingOrder: spec.firingOrder, compressionRatio: spec.compressionRatio,
      duration: spec.cam.duration, lift: spec.cam.lift, exhaustMode: 'split', afr: 14.7, timingOffset: 0,
      simulationFrequency: spec.simulationFrequency,
    }, options));
  }

  configure(options) {
    this.options = Object.assign({}, this.options, options);
    const spec = this.spec, o = this.options, n = spec.cylinders;
    this.timestep = 1 / o.simulationFrequency;
    this.crankRadius = spec.stroke / 2;
    this.boreArea = Math.PI * spec.bore * spec.bore / 4;
    this.sweptVolume = this.boreArea * spec.stroke;
    this.clearanceVolume = this.sweptVolume / (o.compressionRatio - 1);
    this.recipMass = spec.pistonMass + spec.rodMass / 2;
    this.inertia = spec.crankInertia + n * (spec.rodMass / 2) * this.crankRadius * this.crankRadius;
    // Cylinder i reaches firing TDC at cycle angle delta[i].
    this.delta = new Float64Array(n);
    o.firingOrder.forEach((cylinder, k) => { this.delta[cylinder - 1] = k * ES_FOUR_PI / n; });
    const lobe = esHarmonicLobeTable(o.duration, o.lift, spec.cam.gamma);
    this.intakeLobe = lobe; this.exhaustLobe = lobe;
    this.intakeCenter = ES_TAU + spec.cam.intakeCenter * U.deg;
    this.exhaustCenter = ES_TAU - spec.cam.exhaustCenter * U.deg;
    const lifts = spec.head.intakeFlow.map((_, i) => i * 50 * U.thou);
    this.intakePortFlow = new ESTable(lifts, spec.head.intakeFlow.map(esK28));
    this.exhaustPortFlow = new ESTable(lifts, spec.head.exhaustFlow.map(esK28));
    this.timingCurve = new ESTable(spec.timing.map(p => p[0]), spec.timing.map(p => p[1]));
    this.turbulence = new ESTable(Array.from({ length: 30 }, (_, i) => i), Array.from({ length: 30 }, (_, i) => i * 0.5));
    this.fuel = new ESFuel(spec.fuel);
    this.intake = new ESIntake(spec.intake);
    this.intake.afrScale = o.afr / 14.7;
    this.exhausts = spec.exhaust.lengths.map((length, i) => new ESExhaustSystem(spec.exhaust, length, spec.exhaust.audioVolume[i]));
    this.chambers = Array.from({ length: n }, (_, i) => new ESChamber(this, i));
    this.chambers.forEach(ch => ch.initialize());
    this.delays = this.chambers.map(ch => {
      const length = ch.headerLength + ch.exhaust.length;
      return { buffer: new Float64Array(Math.max(1, Math.round(length / 343 * o.simulationFrequency)) + 1), index: 0, length };
    });
    this.lastCycleAngle = this.cycleAngle();
  }

  exhaustFor(i) { return this.options.exhaustMode === 'single' ? this.exhausts[0] : this.exhausts[(i + 1) % 2 === 1 ? 0 : 1]; }
  cycleAngle() { return esMod(this.theta, ES_FOUR_PI); }
  localAngle(i) { return esMod(this.theta - this.delta[i], ES_FOUR_PI); }
  // Piston pin distance from crank centre along the bore axis (PistonEngineSimulator::placeCylinder).
  pistonPosition(a) {
    const r = this.crankRadius, L = this.spec.rod, s = Math.sin(a);
    return r * Math.cos(a) + Math.sqrt(L * L - r * r * s * s);
  }
  chamberVolumeAt(i) {
    const s = this.pistonPosition(this.localAngle(i));
    return this.boreArea * (this.crankRadius + this.spec.rod - s) + this.clearanceVolume;
  }
  intakeLift(i) { return this.intakeLobe.sample(esMod((this.localAngle(i) - this.intakeCenter) / 2 + Math.PI, ES_TAU) - Math.PI); }
  exhaustLift(i) { return this.exhaustLobe.sample(esMod((this.localAngle(i) - this.exhaustCenter) / 2 + Math.PI, ES_TAU) - Math.PI); }
  rpm() { return this.omega / U.rpm; }
  meanPistonSpeed() { return 2 * this.spec.stroke * Math.abs(this.omega) / ES_TAU; }
  timingAdvance() { return (this.timingCurve.sample(Math.abs(this.rpm())) + this.options.timingOffset) * U.deg; }

  // Cycle-averaged brake torque (Simulator::getFilteredDynoTorque uses the same binning).
  averageTorque() { let s = 0; for (const v of this.torqueBins) s += v; return s / ES_TORQUE_BINS; }
  averageDynoTorque() { let s = 0; for (const v of this.dynoBins) s += v; return s / ES_TORQUE_BINS; }

  step() {
    const dt = this.timestep, spec = this.spec, c = this.controls, n = spec.cylinders;
    const r = this.crankRadius, L = spec.rod;
    this.time += dt;

    // Throttle linkage (DirectThrottleLinkage): plate position = 1 - pedal^gamma.
    this.intake.throttle = 1 - Math.pow(esClamp(c.throttle), spec.intake.throttleGamma);

    // 1. Crank dynamics from cylinder pressures.
    let I = this.inertia, dI = 0, tau = 0, friction = 0;
    const omega = this.omega;
    for (let i = 0; i < n; i++) {
      const a = this.localAngle(i), sa = Math.sin(a), ca = Math.cos(a);
      const q = Math.sqrt(L * L - r * r * sa * sa);
      const s1 = -r * sa - r * r * sa * ca / q;
      const s2 = -r * ca - r * r * (ca * ca - sa * sa) / q - Math.pow(r, 4) * sa * sa * ca * ca / (q * q * q);
      I += this.recipMass * s1 * s1;
      dI += 2 * this.recipMass * s1 * s2;
      const F = this.boreArea * (this.chambers[i].system.pressure() - ES_ATM);
      tau -= F * s1;
      // CombustionChamber::calculateFrictionForce with the cylinder wall force from rod angle.
      const vs = s1 * omega, v = Math.abs(vs);
      const wall = Math.abs(F) * Math.abs(r * sa / q);
      const Fcoul = 0.06 * wall, vst = 0.1 * Math.SQRT2, vcoul = 0.01;
      const F1 = v / vst;
      const Ff = (Math.SQRT2 * Math.E * (50 - Fcoul) * Math.exp(-F1 * F1) * F1 + Fcoul * Math.tanh(v / vcoul) + 20 * v) * Math.min(v, 1e-3) / 1e-3;
      friction += Ff * Math.abs(s1);
    }
    const engineTorque = tau - Math.sign(omega) * friction;
    let w = omega + dt * (engineTorque - 0.5 * dI * omega * omega) / I;

    // 2. Clutch and vehicle (Transmission + Vehicle + VehicleDragConstraint).
    const gears = spec.transmission.gears, gear = Math.min(c.gear, gears.length);
    const veh = spec.vehicle;
    if (gear > 0) {
      const ratio = gears[gear - 1] * veh.diffRatio;
      const Iv = veh.mass * (veh.tireRadius / ratio) ** 2;
      const wv = this.vehicleSpeed / veh.tireRadius * ratio;
      const maxTc = esClamp(c.clutch) * spec.transmission.maxClutchTorque;
      const Tc = esClamp((w - wv) / (dt * (1 / I + 1 / Iv)), -maxTc, maxTc);
      w -= Tc * dt / I;
      this.vehicleSpeed += Tc * ratio / veh.tireRadius * dt / veh.mass;
    }
    const rho = ES_AIR_MOLAR_MASS * ES_ATM / (ES_R * ES_T25);
    const drag = veh.rollingResistance + 0.5 * rho * this.vehicleSpeed ** 2 * veh.dragCoefficient * veh.area + esClamp(c.brake) * veh.mass * 9.81;
    this.vehicleSpeed = Math.max(0, this.vehicleSpeed - drag * dt / veh.mass);

    // 3. Dynamometer (src/dynamometer.cpp): absorbs torque, or holds speed in hold mode.
    let dynoTorque = 0;
    if (c.dyno.enabled) {
      const target = c.dyno.rpm * U.rpm, max = 10000 * U.lbft;
      const needed = I * (target - w) / dt;
      dynoTorque = esClamp(needed, -max, c.dyno.hold ? max : 0);
      w += dynoTorque * dt / I;
    }

    // 4. Starter motor (src/starter_motor.cpp).
    if (c.starter) {
      const target = spec.starterSpeed * U.rpm;
      if (w < target) w += Math.min(spec.starterTorque, I * (target - w) / dt) * dt / I;
    }

    // 5. Crankshaft friction and external load, as a Coulomb torque that never reverses rotation.
    const coulomb = (spec.crankFriction + Math.max(0, c.load)) * dt / I;
    w = Math.abs(w) <= coulomb ? 0 : w - Math.sign(w) * coulomb;

    this.omega = w;
    this.theta = esMod(this.theta + w * dt, ES_FOUR_PI * 1024);

    // Cycle-averaged brake torque and dyno torque.
    const cycle = this.cycleAngle();
    const bin = Math.min(ES_TORQUE_BINS - 1, Math.floor(ES_TORQUE_BINS * cycle / ES_FOUR_PI));
    const brake = engineTorque - Math.sign(omega) * spec.crankFriction;
    if (bin !== this.lastBin) {
      for (let k = (this.lastBin + 1) % ES_TORQUE_BINS; k !== bin; k = (k + 1) % ES_TORQUE_BINS) { this.torqueBins[k] = brake; this.dynoBins[k] = -dynoTorque; }
      this.lastBin = bin;
    }
    this.torqueBins[bin] = brake; this.dynoBins[bin] = -dynoTorque;

    // 6. Ignition module (src/ignition_module.cpp), for positive rotation.
    for (let i = 0; i < n; i++) this.sparks[i] = Math.max(0, this.sparks[i] - dt);
    if (c.ignition && this.revLimitTimer === 0) {
      const advance = this.timingAdvance();
      const r0 = this.lastCycleAngle, r1 = cycle;
      for (let i = 0; i < n; i++) {
        const spark = esMod(this.delta[i] - advance, ES_FOUR_PI);
        const crossed = r1 >= r0 ? (spark >= r0 && spark < r1) : (spark >= r0 || spark < r1);
        if (w > 0 && crossed) { this.chambers[i].ignite(); this.sparks[i] = 0.025; this.ignitionEvents++; }
      }
    }
    this.revLimitTimer = Math.max(0, this.revLimitTimer - dt);
    if (Math.abs(this.rpm()) > spec.revLimit) this.revLimitTimer = spec.limiterDuration;
    this.lastCycleAngle = cycle;

    // 7. Gas dynamics (PistonEngineSimulator::simulateStep_), 8 fluid substeps.
    for (const ch of this.chambers) { ch.update(); ch.stepExhaustFlow = 0; ch.stepIntakeFlow = 0; }
    const fluidDt = dt / 8;
    for (let k = 0; k < 8; k++) {
      for (const ex of this.exhausts) ex.process(fluidDt);
      this.intake.process(fluidDt);
      for (const ch of this.chambers) ch.flow(fluidDt);
    }

    // Trace of cylinder 1 for the oscilloscopes.
    const ch1 = this.chambers[0], tb = Math.min(719, Math.floor(this.localAngle(0) / ES_FOUR_PI * 720)), t = this.trace;
    t.pressure[tb] = ch1.system.pressure(); t.volume[tb] = ch1.system.V; t.moles[tb] = ch1.system.n;
    t.temperature[tb] = ch1.system.temperature(); t.intake[tb] = ch1.stepIntakeFlow / dt; t.exhaust[tb] = ch1.stepExhaustFlow / dt;
    t.intakeLift[tb] = ch1.intakeLift; t.exhaustLift[tb] = ch1.exhaustLift;
  }

  // PistonEngineSimulator::writeToSynthesizer: one input sample per exhaust system.
  exhaustSignal(out) {
    out[0] = 0; out[1] = 0;
    const attenuation = Math.min(Math.abs(this.rpm()), 40) / 40, att3 = attenuation ** 3;
    const n = this.spec.cylinders;
    for (let i = 0; i < n; i++) {
      const ch = this.chambers[i], er = ch.exhaustRunner;
      const pulse = att3 * 1600 * ((er.pressure() - ES_ATM) + 0.1 * er.dynamicPressure(1, 0) + 0.1 * er.dynamicPressure(-1, 0));
      const d = this.delays[i], buf = d.buffer;
      const delayed = buf[d.index]; buf[d.index] = pulse; d.index = (d.index + 1) % buf.length;
      const k = ch.exhaust === this.exhausts[0] ? 0 : 1;
      out[k] += ch.soundAttenuation * (ch.exhaust.audioVolume * delayed / n) / (d.length * d.length);
    }
    return out;
  }

  // Readouts for the interface.
  manifoldPressure() { return this.intake.system.pressure(); }
  exhaustTemperature() { let s = 0; for (const ch of this.chambers) s += ch.exhaustRunner.temperature(); return s / this.chambers.length; }
}

if (typeof module !== 'undefined') {
  module.exports = { ES_ENGINES, ES_UNITS, ESEngineSimulation, ESSynthesizer, ESGasSystem, esFlow, esFlowRate, esKCarb, esK28, esHarmonicLobeTable };
}
