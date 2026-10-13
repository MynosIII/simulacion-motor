/* Engine-Sim-Physics.js
 * JavaScript port of the engine-sim simulation core and synthesizer,
 * Copyright 2022 AngeTheGreat (Ange Yaghi), MIT.
 * Source: https://github.com/ange-yaghi/engine-sim, commit 85f7c3b959a908ed5232ede4f1a4ac7eafe6b630.
 * See PROCEDENCIA.md.
 *
 * Ported closely: GasSystem, CombustionChamber, Intake, ExhaustSystem, Fuel,
 * Camshaft and the harmonic cam lobe, StandardValvetrain and VtecValvetrain,
 * IgnitionModule, StarterMotor, Dynamometer, Transmission, Vehicle, the
 * Simulator frame and step loop, PistonEngineSimulator::writeToSynthesizer and
 * the Synthesizer with its filters, convolution and leveler.
 *
 * Replaced: the scs rigid body solver. See ESEngine and
 * ESSimulator.processRigidBodies for what stands in for it.
 */
const ES_R = 8.31446261815324;
const ES_AIR_MOLAR_MASS = 0.02897;
const ES_ATM = 101325;
const ES_T25 = 298.15;
const ES_TAU = 2 * Math.PI;
const ES_FOUR_PI = 4 * Math.PI;
const ES_UNITS = {
  inch: 0.0254, thou: 0.0000254, mm: 0.001, cm: 0.01, cc: 1e-6, L: 1e-3, cm2: 1e-4,
  g: 0.001, lb: 0.45359237, lbft: 0.3048 * 4.44822, rpm: 0.104719755, mph: 0.44704, psi: 6894.757293168,
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

// Intake (src/intake.cpp).
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
    this.molecularAfr = p.molecularAfr || 12.5;
    this.idlePlate = p.idlePlate;
    this.throttle = 1;
    this.flow = 0; this.flowRate = 0; this.totalFuelInjected = 0;
  }
  throttlePlatePosition() { return this.idlePlate * this.throttle; }
  process(dt) {
    const idealAfr = 0.8 * this.molecularAfr * 4;
    const pAir = idealAfr / (1 + idealAfr);
    const pIdleAir = 2 / 3;
    const attenuation = Math.cos(this.throttlePlatePosition() * Math.PI / 2);
    this.atmosphere.reset(ES_ATM, ES_T25, 1 - pAir, pAir * 0.75, pAir * 0.25);
    this.flow = esFlow(attenuation * this.inputK, dt, 0, -1, 10, this.p.plenumArea, this.atmosphere, this.system);
    let idle = 0;
    if (this.idleK !== 0) {
      this.atmosphere.reset(ES_ATM, ES_T25, 1 - pIdleAir, pIdleAir * 0.75, pIdleAir * 0.25);
      idle = esFlow(this.idleK, dt, 0, -1, 10, this.p.plenumArea, this.atmosphere, this.system);
    }
    this.system.dissipateExcessVelocity();
    this.system.updateVelocity(dt, this.p.velocityDecay);
    if (this.flow > 0) this.totalFuelInjected += (1 - pAir) * this.flow;
    if (idle > 0) this.totalFuelInjected += (1 - pIdleAir) * idle;
  }
}

// ExhaustSystem (src/exhaust_system.cpp).
class ESExhaustSystem {
  constructor(p, index) {
    this.p = p; this.index = index;
    this.length = p.length; this.audioVolume = p.audioVolume; this.impulseResponse = p.impulseResponse;
    this.system = new ESGasSystem();
    this.system.initialize(ES_ATM, p.collectorArea * p.length, ES_T25);
    this.system.setGeometry(p.length, Math.sqrt(p.collectorArea), 1, 0);
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

// Fuel (src/fuel.cpp) with the defaults of the fuel node in es/objects/objects.mr.
class ESFuel {
  constructor(p = {}) {
    this.molecularMass = 0.1; this.energyDensity = 48.1e6; this.density = 0.755; this.molecularAfr = 12.5;
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

// GenerateHarmonicCamLobeNode::_evaluate (scripting/include/actions.h): the same
// samples, read back with Function::sampleTriangle, which for a filter radius
// equal to the spacing is linear interpolation clamped at the ends.
function esHarmonicCamLobe(p) {
  const steps = p.steps || 100, gamma = p.gamma, lift = p.lift;
  const angle = p.duration / 4;
  const s = Math.pow(2 * 50 * ES_UNITS.thou / lift, 1 / gamma) - 1;
  const k = Math.acos(s) / angle, extents = Math.PI / k, step = extents / (steps - 5);
  const n = 2 * steps - 1, xs = new Float64Array(n), ys = new Float64Array(n);
  for (let i = 0; i < steps; i++) {
    const x = i * step, y = x >= extents ? 0 : lift * Math.pow(0.5 + 0.5 * Math.cos(k * x), gamma);
    xs[steps - 1 + i] = x; ys[steps - 1 + i] = y;
    xs[steps - 1 - i] = -x; ys[steps - 1 - i] = y;
  }
  const table = new ESTable(xs, ys);
  table.lift = lift;
  return table;
}

// Camshaft (src/camshaft.cpp). Lobe centerlines are given in crank angle.
class ESCamshaft {
  constructor(engine, p) {
    this.engine = engine;
    this.advance = p.advance || 0; this.baseRadius = p.baseRadius;
    this.lobeProfile = esHarmonicCamLobe(p.lobe);
    this.lobeAngles = p.lobes.map(a => a / 2);
  }
  angle() { return esMod((this.engine.crankAngle() + this.advance) * 0.5, ES_TAU); }
  sampleLobe(theta) {
    let t = esMod(theta, ES_TAU);
    if (t >= Math.PI) t -= ES_TAU;
    return this.lobeProfile.sample(t);
  }
  valveLift(lobe) { return this.sampleLobe(this.angle() + this.lobeAngles[lobe]); }
}

// StandardValvetrain and VtecValvetrain (src/vtec_valvetrain.cpp).
class ESValvetrain {
  constructor(engine, p) {
    this.engine = engine;
    this.intakeCamshaft = new ESCamshaft(engine, p.intakeCam);
    this.exhaustCamshaft = new ESCamshaft(engine, p.exhaustCam);
    if (p.vtec) {
      this.vtec = p.vtec;
      this.vtecIntakeCamshaft = new ESCamshaft(engine, p.vtec.intakeCam);
      this.vtecExhaustCamshaft = new ESCamshaft(engine, p.vtec.exhaustCam);
    }
  }
  isVtecEnabled() {
    const v = this.vtec, e = this.engine;
    return !!v && e.manifoldPressure() > v.manifoldVacuum && e.speed() > v.minRpm && (1 - e.throttle) > v.minThrottlePosition;
  }
  activeIntakeCamshaft() { return this.isVtecEnabled() ? this.vtecIntakeCamshaft : this.intakeCamshaft; }
  activeExhaustCamshaft() { return this.isVtecEnabled() ? this.vtecExhaustCamshaft : this.exhaustCamshaft; }
  intakeValveLift(i) { return this.activeIntakeCamshaft().valveLift(i); }
  exhaustValveLift(i) { return this.activeExhaustCamshaft().valveLift(i); }
}

/* ------------------------------------------------------- Combustion chamber */

const ES_STATE_SAMPLES = 256;

// CombustionChamber (src/combustion_chamber.cpp).
class ESChamber {
  constructor(engine, cylinder) {
    this.engine = engine; this.cylinder = cylinder;
    this.bank = cylinder.bank; this.head = cylinder.bank.head;
    this.system = new ESGasSystem(); this.intakeRunner = new ESGasSystem(); this.exhaustRunner = new ESGasSystem();
    this.lit = false; this.litLastFrame = false;
    this.flame = { travelX: 0, travelY: 0, lastVolume: 0, efficiency: 1, flameSpeed: 0, fuel: 0, o2: 0 };
    this.intakeK = 0; this.exhaustK = 0; this.intakeLift = 0; this.exhaustLift = 0;
    this.exhaustFlow = 0; this.lastTimestepExhaustFlow = 0; this.lastTimestepIntakeFlow = 0;
    this.blowbyK = esK28(cylinder.blowby || 0);
    this.pistonSpeedSamples = new Float64Array(ES_STATE_SAMPLES);
    this.pressureSamples = new Float64Array(ES_STATE_SAMPLES);
  }
  initialize() {
    const e = this.engine, head = this.head, cyl = this.cylinder;
    this.intake = e.intake; this.exhaust = cyl.exhaust;
    this.area = this.bank.boreArea;
    const volume = this.volume();
    this.system.initialize(ES_ATM, volume, ES_T25);
    this.system.setGeometry(Math.sqrt(this.area), volume / this.area, 1, 0);
    const iArea = head.intakeRunnerArea, iVol = head.intakeRunnerVolume + iArea * this.intake.p.runnerLength;
    this.intakeRunner.initialize(ES_ATM, iVol, ES_T25);
    this.intakeRunner.setGeometry(iVol / iArea, Math.sqrt(iArea), 1, 0);
    const eArea = head.exhaustRunnerArea;
    const eVol = head.exhaustRunnerVolume + eArea * (this.exhaust.p.primaryTubeLength + cyl.primaryLength);
    this.exhaustRunner.initialize(ES_ATM, eVol, ES_T25);
    this.exhaustRunner.setGeometry(eVol / eArea, Math.sqrt(eArea), 1, 0);
  }
  // CombustionChamber::getVolume: the piston position s is along the bank axis.
  volume() {
    const piston = this.engine.pistonSpec, cyl = this.cylinder;
    return this.area * (this.bank.deckHeight - cyl.s - piston.compressionHeight)
      + this.head.chamberVolume - (piston.displacement || 0);
  }
  popLitLastFrame() { const lit = this.litLastFrame; this.litLastFrame = false; return lit; }
  pistonSpeed() { return this.cylinder.ds * this.engine.omega; }
  meanPistonSpeed() { let s = 0; for (const v of this.pistonSpeedSamples) s += v; return s / ES_STATE_SAMPLES; }
  updateCycleStates() {
    const i = Math.round(this.engine.cycleAngle() / ES_FOUR_PI * (ES_STATE_SAMPLES - 1));
    this.pistonSpeedSamples[i] = Math.abs(this.pistonSpeed());
    this.pressureSamples[i] = this.system.pressure();
  }
  ignite() {
    const sys = this.system;
    if (this.lit || sys.fuel === 0) return;
    const fuel = this.engine.fuel;
    const afr = sys.o2 / sys.fuel, equivalence = afr / fuel.molecularAfr;
    if (equivalence < 0.5 || equivalence > 1.9) return;
    const dilution = sys.inert / (sys.o2 / 0.7) - 1;
    const f = this.flame;
    f.lastVolume = this.volume(); f.travelX = 0; f.travelY = 0; f.fuel = sys.fuel; f.o2 = sys.o2;
    this.lit = true; this.litLastFrame = true;
    const turbulence = this.engine.meanPistonSpeedToTurbulence.sample(this.meanPistonSpeed());
    const mixing = 1 - esClamp(turbulence / fuel.maxTurbulenceEffect) * esClamp(1 - dilution / fuel.maxDilutionEffect);
    const r = fuel.lowEfficiencyAttenuation * ((1 - fuel.burningEfficiencyRandomness) + fuel.burningEfficiencyRandomness * Math.random());
    f.efficiency = (mixing * r + (1 - mixing)) * fuel.maxBurningEfficiency;
    f.flameSpeed = fuel.flameSpeed(turbulence, afr, sys.temperature(), sys.pressure());
  }
  update() {
    const head = this.head, index = this.cylinder.index;
    this.system.setVolume(this.volume());
    this.updateCycleStates();
    this.intakeLift = head.valvetrain.intakeValveLift(index);
    this.exhaustLift = head.valvetrain.exhaustValveLift(index);
    this.intakeK = head.intakePortFlow.sample(this.intakeLift);
    this.exhaustK = head.exhaustPortFlow.sample(this.exhaustLift);
  }
  flow(dt) {
    const sys = this.system, intake = this.intake, exhaust = this.exhaust, head = this.head;
    const volume = this.volume(), area = this.area, height = volume / area;
    const surface = height * Math.PI * this.bank.bore + area * 2;
    sys.changeEnergy((363.15 - sys.temperature()) * surface * 100 * dt);
    if (this.blowbyK !== 0) sys.flowToEnvironment(this.blowbyK, dt, ES_ATM, ES_T25);

    const iArea = head.intakeRunnerArea, eArea = head.exhaustRunnerArea;
    esFlow(intake.runnerK, dt, 1, 0, intake.p.plenumArea, iArea, intake.system, this.intakeRunner);
    this.intakeRunner.dissipateExcessVelocity();
    const intakeFlow = esFlow(this.intakeK, dt, 1, 0, iArea, volume / height, this.intakeRunner, sys);
    this.intakeRunner.dissipateExcessVelocity(); sys.dissipateExcessVelocity();
    const exhaustFlow = esFlow(this.exhaustK, dt, 1, 0, volume / height, eArea, sys, this.exhaustRunner);
    sys.dissipateExcessVelocity(); this.exhaustRunner.dissipateExcessVelocity();
    esFlow(exhaust.primaryK, dt, 1, 0, eArea, exhaust.p.collectorArea, this.exhaustRunner, exhaust.system);

    this.intakeRunner.updateVelocity(dt, intake.p.velocityDecay);
    sys.updateVelocity(dt, 0.5);
    this.exhaustRunner.updateVelocity(dt, exhaust.p.velocityDecay);

    if (Math.abs(intakeFlow) > 1e-9 && this.lit) this.lit = false;
    this.exhaustFlow = exhaustFlow;
    this.lastTimestepExhaustFlow += exhaustFlow; this.lastTimestepIntakeFlow += intakeFlow;

    if (this.lit) {
      const f = this.flame;
      const totalX = this.bank.bore / 2, totalY = volume / area;
      const lastX = f.travelX, lastY = f.travelY * (volume / f.lastVolume);
      f.travelX = Math.min(lastX + dt * f.flameSpeed, totalX);
      f.travelY = Math.min(lastY + dt * f.flameSpeed, totalY);
      if (lastX < f.travelX || lastY < f.travelY) {
        const burned = f.travelX * f.travelX * Math.PI * f.travelY;
        const previous = lastX * lastX * Math.PI * lastY;
        const n = ((burned - previous) / volume) * sys.n;
        const fuelBurned = sys.react(n * f.efficiency, f.fuel, f.o2);
        sys.changeEnergy(fuelBurned * this.engine.fuel.molecularMass * this.engine.fuel.energyDensity);
      } else this.lit = false;
      f.lastVolume = volume;
    }
  }
  // CombustionChamber::calculateFrictionForce with the default friction model.
  frictionForce(vs, wallForce) {
    const Fcoul = 0.06 * wallForce, vst = 0.1 * Math.SQRT2, vcoul = 0.1 / 10, Fbrk = 50;
    const v = Math.abs(vs), F1 = v / vst;
    return Math.SQRT2 * Math.E * (Fbrk - Fcoul) * Math.exp(-F1 * F1) * F1 + Fcoul * Math.tanh(v / vcoul) + 20 * v;
  }
}

/* ------------------------------------------------------------------- Engine */

// Engine, Crankshaft, CylinderBank, Piston, ConnectingRod and CylinderHead as one
// object built from a description that mirrors the .mr engine scripts.
//
// The original solves crankshaft, rods and pistons as rigid bodies held by
// constraints. Rigid links leave a single degree of freedom, so here the
// crankshaft angle drives exact kinematics of every piston and rod, and the
// equation of motion uses the configuration-dependent inertia
//   I(θ) = I_crank + Σ m_p s'² + m_r |r_cm'|² + I_r ψ'²
// with I(θ) ω' + ½ I'(θ) ω² = τ. Angles follow engine-sim: the crankshaft body
// angle θ, getAngle() = θ - tdc and getCycleAngle() = mod(-getAngle(), 4π).
class ESEngine {
  constructor(spec) {
    this.spec = spec;
    const crank = spec.crankshaft;
    this.name = spec.name;
    this.redline = spec.redline; this.starterTorque = spec.starterTorque; this.starterSpeed = spec.starterSpeed;
    this.axialOrder = spec.axialOrder || null;
    this.dynoMinSpeed = spec.dynoMinSpeed ?? 1000 * ES_UNITS.rpm;
    this.dynoMaxSpeed = spec.dynoMaxSpeed ?? spec.redline;
    this.dynoHoldStep = spec.dynoHoldStep ?? 100 * ES_UNITS.rpm;
    this.throttleGamma = spec.throttleGamma ?? 1;
    this.simulationFrequency = spec.simulationFrequency ?? 10000;
    this.hfGain = spec.hfGain ?? 0.01; this.jitter = spec.jitter ?? 0.5; this.noise = spec.noise ?? 1.0;
    this.fuel = new ESFuel(spec.fuel);
    this.pistonSpec = spec.piston; this.rodSpec = spec.rod;
    this.crank = crank;
    this.theta = 0; this.omega = 0; this.alpha = 0;
    this.throttle = 1; this.speedControl = 0;
    this.meanPistonSpeedToTurbulence = new ESTable(Array.from({ length: 30 }, (_, i) => i), Array.from({ length: 30 }, (_, i) => 0.5 * i));
    this.intake = new ESIntake(spec.intake);
    this.exhausts = spec.exhausts.map((p, i) => new ESExhaustSystem(p, i));

    const lifts = n => Array.from({ length: n }, (_, i) => i * 50 * ES_UNITS.thou);
    this.banks = spec.banks.map((b, index) => {
      const head = Object.assign({}, spec.head, b.head || {});
      const bank = {
        index, angle: b.angle, bore: b.bore ?? spec.bore, deckHeight: b.deckHeight ?? spec.deckHeight,
        x: b.x || 0, y: b.y || 0, displayDepth: b.displayDepth ?? 0.5,
        cylinders: [], head,
      };
      bank.dx = Math.cos(bank.angle + Math.PI / 2); bank.dy = Math.sin(bank.angle + Math.PI / 2);
      bank.boreArea = Math.PI * bank.bore * bank.bore / 4;
      head.flipDisplay = !!b.flip;
      head.intakePortFlow = new ESTable(lifts(head.intakeFlow.length), head.intakeFlow.map(esK28));
      head.exhaustPortFlow = new ESTable(lifts(head.exhaustFlow.length), head.exhaustFlow.map(esK28));
      head.valvetrain = new ESValvetrain(this, b.valvetrain);
      return bank;
    });
    this.cylinders = [];
    spec.banks.forEach((b, bi) => b.cylinders.forEach((c, ci) => {
      const cyl = {
        global: this.cylinders.length, index: ci, bank: this.banks[bi],
        journal: c.journal, journalAngle: crank.journals[c.journal],
        wire: c.wire, exhaust: this.exhausts[c.exhaust || 0],
        primaryLength: c.primaryLength || 0, soundAttenuation: c.soundAttenuation ?? 1, blowby: c.blowby || 0,
        s: 0, ds: 0, dds: 0, rodTheta: 0, dpsi: 0, pinX: 0, pinY: 0, journalX: 0, journalY: 0,
      };
      this.banks[bi].cylinders.push(cyl);
      this.cylinders.push(cyl);
    }));
    this.chambers = this.cylinders.map(c => new ESChamber(this, c));
    this.maxDepth = Math.max(...this.cylinders.map(c => c.journal)) + 1;
    this.computeGeometry();
    for (const ch of this.chambers) ch.initialize();
    this.displacement = this.calculateDisplacement();
  }

  // Exact slider-crank kinematics for every cylinder at the current angle, with
  // first and second derivatives against θ. Fills this.inertia and this.dInertia.
  computeGeometry() {
    const crank = this.crank, r = crank.throw, L = this.rodSpec.length;
    const mp = this.pistonSpec.mass, mr = this.rodSpec.mass, Ir = this.rodSpec.momentOfInertia;
    const cx = crank.x || 0, cy = crank.y || 0, theta = this.theta;
    let I = crank.momentOfInertia, dI = 0;
    for (const c of this.cylinders) {
      const bank = c.bank, ux = bank.dx, uy = bank.dy;
      const phi = theta + c.journalAngle, jx = r * Math.cos(phi), jy = r * Math.sin(phi);
      const wx = cx + jx - bank.x, wy = cy + jy - bank.y;
      const w1x = -jy, w1y = jx, w2x = -jx, w2y = -jy;
      const a = ux * wx + uy * wy, a1 = ux * w1x + uy * w1y, a2 = ux * w2x + uy * w2y;
      const ww = wx * wx + wy * wy, q = Math.sqrt(Math.max(L * L - ww + a * a, 1e-12));
      const wdw = wx * w1x + wy * w1y;
      const q1 = (-wdw + a * a1) / q;
      const q2 = (-(w1x * w1x + w1y * w1y + wx * w2x + wy * w2y) + a1 * a1 + a * a2 - q1 * q1) / q;
      const s = a + q, s1 = a1 + q1, s2 = a2 + q2;
      // Rod: d = P - J, its centre of mass at the midpoint (center_of_mass is 0 in every engine here).
      const dx = s * ux - wx, dy = s * uy - wy;
      const d1x = s1 * ux - w1x, d1y = s1 * uy - w1y, d2x = s2 * ux - w2x, d2y = s2 * uy - w2y;
      const psi1 = (dx * d1y - dy * d1x) / (L * L), psi2 = (dx * d2y - dy * d2x) / (L * L);
      const r1x = 0.5 * (s1 * ux + w1x), r1y = 0.5 * (s1 * uy + w1y);
      const r2x = 0.5 * (s2 * ux + w2x), r2y = 0.5 * (s2 * uy + w2y);
      I += mp * s1 * s1 + mr * (r1x * r1x + r1y * r1y) + Ir * psi1 * psi1;
      dI += 2 * (mp * s1 * s2 + mr * (r1x * r2x + r1y * r2y) + Ir * psi1 * psi2);
      c.s = s; c.ds = s1; c.dds = s2;
      c.journalX = cx + jx; c.journalY = cy + jy;
      c.pinX = bank.x + s * ux; c.pinY = bank.y + s * uy;
      c.rodTheta = Math.atan2(dy, dx) - Math.PI / 2;
      c.cosRod = (dx * ux + dy * uy) / L; c.sinRod = (dx * uy - dy * ux) / L;
    }
    this.inertia = I; this.dInertia = dI;
  }

  // Engine::calculateDisplacement, numerically like the original.
  calculateDisplacement() {
    const theta = this.theta, min = this.cylinders.map(() => Infinity), max = this.cylinders.map(() => -Infinity);
    for (let j = 0; j < 1000; j++) {
      this.theta = ES_TAU * j / 1000; this.computeGeometry();
      this.cylinders.forEach((c, i) => { min[i] = Math.min(min[i], c.s); max[i] = Math.max(max[i], c.s); });
    }
    this.theta = theta; this.computeGeometry();
    return this.cylinders.reduce((sum, c, i) => sum + c.bank.boreArea * (max[i] - min[i]), 0);
  }

  // Generalized torque of the gas and piston friction forces on the crankshaft.
  pistonTorque() {
    const w = this.omega, alpha = this.alpha, mp = this.pistonSpec.mass;
    let tau = 0;
    for (const ch of this.chambers) {
      const c = ch.cylinder, A = ch.area;
      const force = -A * (ch.system.pressure() - ES_ATM);
      const vs = c.ds * w;
      // Cylinder wall reaction from the rod, standing in for the line constraint force.
      const as = c.dds * w * w + c.ds * alpha;
      const rodForce = (mp * as - force) / (Math.abs(c.cosRod) > 1e-6 ? c.cosRod : 1e-6);
      const wall = Math.abs(rodForce * c.sinRod);
      const attenuation = Math.min(Math.abs(vs), 1e-3) / 1e-3;
      const F = ch.frictionForce(vs, wall) * attenuation;
      tau += (force + (vs > 0 ? -F : F)) * c.ds;
    }
    return tau;
  }

  crankAngle() { return this.theta - this.crank.tdc; }
  cycleAngle(offset = 0) { return esMod(-this.crankAngle() + offset, ES_FOUR_PI); }
  isSpinningCw() { return this.omega <= 0; }
  speed() { return Math.abs(this.omega); }
  rpm() { return Math.abs(this.omega) / ES_UNITS.rpm; }
  setSpeedControl(s) { this.speedControl = s; this.throttleLinkage = 1 - Math.pow(s, this.throttleGamma); }
  update() { this.throttle = this.throttleLinkage ?? 1; this.intake.throttle = this.throttle; }
  manifoldPressure() { return this.intake.system.pressure(); }
  throttlePlateAngle() { return (1 - this.intake.throttlePlatePosition()) * (Math.PI / 2); }
  intakeFlowRate() { return this.intake.flowRate; }
  intakeAfr() {
    const s = this.intake.system, o2 = s.o2 * s.n, fuel = s.fuel * s.n;
    return fuel === 0 ? 0 : (0.0319988 * o2 / 0.21) / (fuel * 0.11423);
  }
  exhaustO2() {
    let inert = 0, o2 = 0, fuel = 0;
    for (const e of this.exhausts) { const s = e.system; inert += s.inert * s.n; o2 += s.o2 * s.n; fuel += s.fuel * s.n; }
    return fuel === 0 ? 0 : (0.0319988 * o2) / (fuel * 0.11423 + 0.028014 * inert + 0.0319988 * o2);
  }
  totalFuelMassConsumed() { return this.intake.totalFuelInjected * this.fuel.molecularMass; }
  totalVolumeFuelConsumed() { return this.totalFuelMassConsumed() / this.fuel.density / 1000; }
}

/* ------------------------------------------------------------------ Ignition */

// IgnitionModule (src/ignition_module.cpp).
class ESIgnitionModule {
  constructor(engine, p) {
    this.engine = engine;
    this.timingCurve = new ESTable(p.timing.map(t => t[0]), p.timing.map(t => t[1]));
    this.revLimit = p.revLimit; this.limiterDuration = p.limiterDuration;
    this.plugs = engine.cylinders.map(c => ({ angle: p.wires[c.wire], enabled: true, ignitionEvent: false }));
    this.enabled = false; this.revLimitTimer = 0;
    this.lastCrankshaftAngle = engine.cycleAngle();
    // Engines that turn the other way give their timing as negative angles.
    // The sign also says which way the plug angles advance in time.
    this.direction = Math.sign(p.timing[p.timing.length - 1][1]) || 1;
    this.timingOffset = 0;
  }
  timingAdvance() { return this.timingCurve.sample(-this.engine.omega) + this.direction * this.timingOffset; }
  update(dt) {
    const e = this.engine, cycleAngle = e.cycleAngle();
    if (this.enabled && this.revLimitTimer === 0) {
      const advance = this.timingAdvance();
      for (const plug of this.plugs) {
        let adjusted = esMod(plug.angle - advance, ES_FOUR_PI);
        const r0 = this.lastCrankshaftAngle;
        let r1 = cycleAngle;
        if (e.omega < 0) {
          if (r1 < r0) { r1 += ES_FOUR_PI; adjusted += ES_FOUR_PI; }
          if (adjusted >= r0 && adjusted < r1) plug.ignitionEvent = plug.enabled;
        } else {
          if (r1 > r0) { r1 -= ES_FOUR_PI; adjusted -= ES_FOUR_PI; }
          if (adjusted >= r1 && adjusted < r0) plug.ignitionEvent = plug.enabled;
        }
      }
    }
    this.revLimitTimer -= dt;
    if (Math.abs(e.omega) > this.revLimit) this.revLimitTimer = this.limiterDuration;
    if (this.revLimitTimer < 0) this.revLimitTimer = 0;
    this.lastCrankshaftAngle = cycleAngle;
  }
  resetIgnitionEvents() { for (const p of this.plugs) p.ignitionEvent = false; }
}

/* ------------------------------------------------- Vehicle and transmission */

// Vehicle (src/vehicle.cpp) and Transmission (src/transmission.cpp). The
// transmission output is a rotating mass whose inertia stands for the car.
class ESVehicle {
  constructor(p) { Object.assign(this, p); this.travelledDistance = 0; }
}
class ESTransmission {
  constructor(p, vehicle, rotatingMass) {
    this.gearRatios = p.gears; this.maxClutchTorque = p.maxClutchTorque;
    this.vehicle = vehicle; this.rotatingMass = rotatingMass;
    this.gear = -1; this.clutchPressure = 0;
  }
  changeGear(newGear) {
    if (newGear < -1 || newGear >= this.gearRatios.length) return;
    if (newGear !== -1) {
      const v = this.vehicle, m = this.rotatingMass;
      const f = v.tireRadius / (v.diffRatio * this.gearRatios[newGear]);
      const newI = v.mass * f * f, E = 0.5 * m.I * m.v * m.v;
      m.v = (m.v < 0 ? -1 : 1) * Math.sqrt(E * 2 / newI);
      m.I = newI; m.m = v.mass;
    }
    this.gear = newGear;
  }
}

/* ---------------------------------------------------------------- Simulator */

const ES_DYNO_TORQUE_SAMPLES = 512;

class ESDelayFilter {
  constructor(delay, sampleRate) {
    this.latency = Math.round(delay * sampleRate);
    this.buffer = new Float64Array(this.latency + 1); this.write = 0; this.size = 0;
  }
  f(x) {
    const n = this.buffer.length;
    this.buffer[this.write] = x; this.write = (this.write + 1) % n;
    if (this.size <= this.latency) { this.size++; if (this.size <= this.latency) return 0; }
    return this.buffer[this.write];
  }
}

// Simulator + PistonEngineSimulator (src/simulator.cpp, src/piston_engine_simulator.cpp).
class ESSimulator {
  constructor(spec, synthesizerInput) {
    this.engine = new ESEngine(spec);
    this.vehicle = new ESVehicle(spec.vehicle);
    this.vehicleMass = { I: 1, m: 1, v: 0 };
    this.transmission = new ESTransmission(spec.transmission, this.vehicle, this.vehicleMass);
    this.ignitionModule = new ESIgnitionModule(this.engine, spec.ignition);
    this.starterMotor = { enabled: false, maxTorque: this.engine.starterTorque, rotationSpeed: -this.engine.starterSpeed };
    this.dyno = { enabled: false, hold: false, rotationSpeed: 0, maxTorque: 10000 * ES_UNITS.lbft, torque: 0 };
    this.simulationFrequency = this.engine.simulationFrequency;
    this.simulationSpeed = 1;
    this.fluidSimulationSteps = 8;
    this.targetSynthesizerLatency = 0.1;
    this.filteredEngineSpeed = 0;
    this.dynoTorqueSamples = new Float64Array(ES_DYNO_TORQUE_SAMPLES);
    this.lastDynoTorqueSample = 0;
    this.steps = 0; this.currentIteration = 0;
    this.delayFilters = this.engine.cylinders.map(c => new ESDelayFilter((c.primaryLength + c.exhaust.length) / 343, 10000));
    this.exhaustFlowStaging = new Float64Array(this.engine.exhausts.length);
    this.synthesizerInput = synthesizerInput || null;
    if (this.synthesizerInput) this.synthesizerInput.setInputSampleRate(this.simulationFrequency);
    this.physicsProcessingTime = 0;
  }
  timestep() { return 1 / this.simulationFrequency; }
  setSimulationFrequency(f) { this.simulationFrequency = f; }

  // Not in engine-sim: the firing order as cylinder indices (0 based),
  // starting at the first cylinder, read from the spark plug angles.
  firingOrder() {
    const plugs = this.ignitionModule.plugs, d = this.ignitionModule.direction;
    const idx = plugs.map((_, i) => i).sort((a, b) => d * (plugs[a].angle - plugs[b].angle));
    const k = idx.indexOf(0);
    return idx.slice(k).concat(idx.slice(0, k));
  }
  // Moves each cylinder's spark and cam lobes to the slot its position in
  // `order` asks for. The crankshaft is not changed, so an order that does not
  // match its throws fires cylinders away from top dead center, like the real
  // engine would. Returns false if `order` is not a permutation.
  setFiringOrder(order) {
    const plugs = this.ignitionModule.plugs, cyls = this.engine.cylinders, n = plugs.length;
    if (order.length !== n || new Set(order).size !== n || order.some(c => !(c >= 0 && c < n))) return false;
    const cams = c => { const vt = c.bank.head.valvetrain; return [vt.intakeCamshaft, vt.exhaustCamshaft, vt.vtecIntakeCamshaft, vt.vtecExhaustCamshaft].filter(Boolean); };
    const slots = this.firingOrder().map(i => ({ angle: plugs[i].angle, lobes: cams(cyls[i]).map(cam => cam.lobeAngles[cyls[i].index]) }));
    const k = order.indexOf(0), o = order.slice(k).concat(order.slice(0, k));
    o.forEach((i, j) => {
      plugs[i].angle = slots[j].angle;
      cams(cyls[i]).forEach((cam, m) => { cam.lobeAngles[cyls[i].index] = slots[j].lobes[m]; });
    });
    return true;
  }

  // Simulator::startFrame: the synthesizer input latency steers how many steps run.
  startFrame(dt, latency) {
    if (this.synthesizerInput) this.synthesizerInput.setInputSampleRate(this.simulationFrequency * this.simulationSpeed);
    this.currentIteration = 0;
    let steps = Math.round(dt * this.simulationSpeed / this.timestep());
    if (latency !== undefined) {
      if (latency < this.targetSynthesizerLatency) steps = Math.trunc((steps + 1) * 1.1);
      else if (latency > this.targetSynthesizerLatency) steps = Math.max(0, Math.trunc((steps - 1) * 0.9));
    }
    this.steps = steps;
    if (steps > 0) this.engine.intake.flowRate = 0;
    return steps;
  }

  // atg_scs::RigidBodySystem::process reduced to one degree of freedom plus the
  // vehicle's rotating mass. Constraints are velocity constraints with torque
  // limits solved by projected Gauss-Seidel, like the original's limits.
  processRigidBodies(dt) {
    const e = this.engine, vm = this.vehicleMass, t = this.transmission, v = this.vehicle;
    e.computeGeometry();
    const I = e.inertia, w0 = e.omega;
    let wc = w0 + dt * (e.pistonTorque() - 0.5 * e.dInertia * w0 * w0) / I;
    let wv = vm.v;

    const st = this.starterMotor, dy = this.dyno, cons = this._constraints || (this._constraints = Array.from({ length: 5 }, () => ({})));
    const set = (k, jc, jv, target, lo, hi) => { const c = cons[k]; c.jc = jc; c.jv = jv; c.target = target; c.lo = lo * dt; c.hi = hi * dt; c.lambda = 0; c.mEff = 1 / (jc * jc / I + jv * jv / vm.I); };
    // Starter motor (src/starter_motor.cpp).
    if (st.rotationSpeed < 0) set(0, 1, 0, st.rotationSpeed, st.enabled ? -st.maxTorque : 0, 0);
    else set(0, 1, 0, st.rotationSpeed, 0, st.enabled ? st.maxTorque : 0);
    // Dynamometer (src/dynamometer.cpp).
    const max = dy.maxTorque;
    if (w0 < 0) set(1, 1, 0, -dy.rotationSpeed, (dy.hold && dy.enabled) ? -max : 0, dy.enabled ? max : 0);
    else set(1, 1, 0, dy.rotationSpeed, dy.enabled ? -max : 0, (dy.hold && dy.enabled) ? max : 0);
    // Crankshaft friction (RotationFrictionConstraint).
    set(2, 1, 0, 0, -e.crank.frictionTorque, e.crank.frictionTorque);
    // Clutch (ClutchConstraint) between the crankshaft and the vehicle mass.
    const clutch = t.gear === -1 ? 0 : t.maxClutchTorque * t.clutchPressure;
    set(3, 1, -1, 0, -clutch, clutch);
    // Vehicle drag (VehicleDragConstraint), converted to a virtual torque.
    const speed = Math.sqrt(vm.I / v.mass) * Math.abs(wv);
    const rho = ES_AIR_MOLAR_MASS * ES_ATM / (ES_R * ES_T25);
    const drag = Math.sqrt(vm.I / v.mass) * (v.rollingResistance + 0.5 * rho * speed * speed * v.dragCoefficient * v.crossSectionArea);
    set(4, 0, 1, 0, -drag, drag);

    for (let iter = 0; iter < 8; iter++) {
      for (const c of cons) {
        if (c.lo === 0 && c.hi === 0) continue;
        const vel = c.jc * wc + c.jv * wv;
        const prev = c.lambda;
        c.lambda = esClamp(prev + (c.target - vel) * c.mEff, c.lo, c.hi);
        const dl = c.lambda - prev;
        wc += c.jc * dl / I; wv += c.jv * dl / vm.I;
      }
    }
    const dynoTorque = cons[1].lambda / dt;
    dy.torque = wc > 0 ? -dynoTorque : dynoTorque;
    this.starterTorque = cons[0].lambda / dt;
    e.alpha = (wc - w0) / dt;
    e.omega = wc; vm.v = wv;
    e.theta += wc * dt;
    // Crankshaft::resetAngle
    e.theta = e.theta % ES_FOUR_PI;
    e.computeGeometry();
  }

  // PistonEngineSimulator::endFrame: the intake flow summed over the frame becomes a rate.
  endFrame() {
    const t = this.steps * this.timestep();
    if (t > 0) this.engine.intake.flowRate /= t;
  }

  vehicleSpeed() { const vm = this.vehicleMass; return Math.sqrt(vm.I * vm.v * vm.v / this.vehicle.mass); }

  simulateStep() {
    if (this.currentIteration >= this.steps) return false;
    const dt = this.timestep(), e = this.engine;
    this.processRigidBodies(dt);
    e.update(dt);
    this.vehicle.travelledDistance += this.vehicleSpeed() * dt;

    const alpha = dt / (100 + dt);
    this.filteredEngineSpeed = alpha * this.filteredEngineSpeed + (1 - alpha) * e.rpm();

    const index = Math.min(ES_DYNO_TORQUE_SAMPLES - 1, Math.floor(ES_DYNO_TORQUE_SAMPLES * e.cycleAngle() / ES_FOUR_PI));
    const step = e.isSpinningCw() ? 1 : -1;
    const torque = this.dyno.torque, samples = this.dynoTorqueSamples;
    samples[index] = torque;
    if (this.lastDynoTorqueSample !== index) {
      for (let i = this.lastDynoTorqueSample + step; i !== index; i += step) {
        if (i >= ES_DYNO_TORQUE_SAMPLES) { i = -1; continue; }
        else if (i < 0) { i = ES_DYNO_TORQUE_SAMPLES; continue; }
        samples[i] = torque;
      }
      this.lastDynoTorqueSample = index;
    }

    this.simulateStep_(dt);
    this.writeToSynthesizer();
    this.currentIteration++;
    return true;
  }

  simulateStep_(dt) {
    const e = this.engine, im = this.ignitionModule;
    im.update(dt);
    for (let i = 0; i < e.chambers.length; i++) {
      if (im.plugs[i].ignitionEvent) e.chambers[i].ignite();
      e.chambers[i].update();
    }
    for (const ch of e.chambers) { ch.lastTimestepExhaustFlow = 0; ch.lastTimestepIntakeFlow = 0; }
    const fluidDt = dt / this.fluidSimulationSteps;
    for (let k = 0; k < this.fluidSimulationSteps; k++) {
      for (const ex of e.exhausts) ex.process(fluidDt);
      e.intake.process(fluidDt);
      e.intake.flowRate += e.intake.flow;
      for (const ch of e.chambers) ch.flow(fluidDt);
    }
    im.resetIgnitionEvents();
  }

  writeToSynthesizer() {
    const e = this.engine, staging = this.exhaustFlowStaging, n = e.cylinders.length;
    staging.fill(0);
    const attenuation = Math.min(Math.abs(this.filteredEngineSpeed), 40) / 40, att3 = attenuation ** 3;
    for (let i = 0; i < n; i++) {
      const c = e.cylinders[i], ch = e.chambers[i], er = ch.exhaustRunner;
      const exhaustLength = c.primaryLength + c.exhaust.length;
      const flow = att3 * 1600 * ((er.pressure() - ES_ATM) + 0.1 * er.dynamicPressure(1, 0) + 0.1 * er.dynamicPressure(-1, 0));
      const delayed = this.delayFilters[i].f(flow);
      staging[c.exhaust.index] += c.soundAttenuation * (c.exhaust.audioVolume * delayed / n) / (exhaustLength * exhaustLength);
    }
    if (this.synthesizerInput) this.synthesizerInput.writeInput(staging);
  }

  filteredDynoTorque() { let s = 0; for (const v of this.dynoTorqueSamples) s += v; return s / ES_DYNO_TORQUE_SAMPLES; }
  dynoPower() { return this.engine.speed() > 0 ? this.filteredDynoTorque() * this.engine.speed() : 0; }
  totalExhaustFlow() { let s = 0; for (const ch of this.engine.chambers) s += ch.lastTimestepExhaustFlow; return s; }
}

/* -------------------------------------------------------------- Synthesizer */

class ESButterworth {
  constructor(cutoff, sampleRate) { this.x0 = this.x1 = this.x2 = this.x3 = 0; this.y0 = this.y1 = this.y2 = this.y3 = 0; this.setCutoff(cutoff, sampleRate); }
  setCutoff(fc, rate) {
    const f = Math.tan(Math.PI * fc / rate), f2 = f * f, f3 = f2 * f, f4 = f2 * f2;
    const m = -2 * Math.cos(5 * Math.PI / 8), n = -2 * Math.cos(7 * Math.PI / 8);
    const a0 = 1 + (m + n) * f + (2 + n * m) * f2 + (m + n) * f3 + f4;
    this.a1 = (-4 - 2 * (n + m) * f + 2 * (m + n) * f3 + 4 * f4) / a0;
    this.a2 = (6 - 2 * (2 + m * n) * f2 + 6 * f4) / a0;
    this.a3 = (-4 + 2 * (m + n) * f - 2 * (m + n) * f3 + 4 * f4) / a0;
    this.a4 = (1 - (n + m) * f + (2 + m * n) * f2 - (m + n) * f3 + f4) / a0;
    this.g = f4 / a0;
  }
  // ButterworthLowPassFilter::fast_f; x3/y3 hold the newest previous values.
  f(x) {
    const y = this.g * (x + 4 * this.x3 + 6 * this.x2 + 4 * this.x1 + this.x0)
      - this.a1 * this.y3 - this.a2 * this.y2 - this.a3 * this.y1 - this.a4 * this.y0;
    this.x0 = this.x1; this.x1 = this.x2; this.x2 = this.x3; this.x3 = x;
    this.y0 = this.y1; this.y1 = this.y2; this.y2 = this.y3; this.y3 = y;
    return y;
  }
}

// JitterFilter (include/jitter_filter.h).
class ESJitter {
  constructor(maxJitter, cutoff, rate) { this.history = new Float64Array(maxJitter); this.max = maxJitter; this.offset = 0; this.scale = 0; this.noise = new ESButterworth(cutoff, rate); }
  f(sample) {
    const h = this.history, max = this.max;
    h[this.offset] = sample; if (++this.offset >= max) this.offset = 0;
    const s = this.noise.f(Math.random() * (max - 1) * this.scale);
    const i0 = Math.min(Math.max(Math.floor(s), 0), max - 1), i1 = Math.min(Math.max(Math.ceil(s), 0), max - 1);
    const frac = s - i0, j0 = i0 + this.offset, j1 = i1 + this.offset;
    return h[j1 >= max ? j1 - max : j1] * frac + h[j0 >= max ? j0 - max : j0] * (1 - frac);
  }
}

// Radix-2 complex FFT on separate real and imaginary arrays.
class ESFFT {
  constructor(n) {
    this.n = n; this.rev = new Uint32Array(n);
    const bits = Math.log2(n);
    for (let i = 0; i < n; i++) { let r = 0; for (let b = 0; b < bits; b++) r |= ((i >> b) & 1) << (bits - 1 - b); this.rev[i] = r; }
    this.cos = new Float64Array(n / 2); this.sin = new Float64Array(n / 2);
    for (let i = 0; i < n / 2; i++) { this.cos[i] = Math.cos(2 * Math.PI * i / n); this.sin[i] = -Math.sin(2 * Math.PI * i / n); }
  }
  transform(re, im, inverse) {
    const n = this.n, rev = this.rev;
    for (let i = 0; i < n; i++) { const j = rev[i]; if (j > i) { let t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; } }
    const sgn = inverse ? -1 : 1;
    for (let size = 2; size <= n; size <<= 1) {
      const half = size >> 1, step = n / size;
      for (let i = 0; i < n; i += size) {
        for (let j = 0, k = 0; j < half; j++, k += step) {
          const wr = this.cos[k], wi = sgn * this.sin[k];
          const a = i + j, b = a + half;
          const tr = re[b] * wr - im[b] * wi, ti = re[b] * wi + im[b] * wr;
          re[b] = re[a] - tr; im[b] = im[a] - ti; re[a] += tr; im[a] += ti;
        }
      }
    }
  }
}

// ConvolutionFilter (src/convolution_filter.cpp) computed block by block as a
// uniformly partitioned overlap-save convolution. The output is identical to the
// direct form; only the arithmetic is faster.
class ESConvolution {
  constructor(ir, block) {
    this.B = block; this.N = 2 * block; this.fft = new ESFFT(this.N);
    this.P = Math.max(1, Math.ceil(ir.length / block));
    this.hRe = []; this.hIm = [];
    for (let p = 0; p < this.P; p++) {
      const re = new Float64Array(this.N), im = new Float64Array(this.N);
      for (let i = 0; i < block; i++) { const k = p * block + i; re[i] = k < ir.length ? ir[k] : 0; }
      this.fft.transform(re, im, false);
      this.hRe.push(re); this.hIm.push(im);
    }
    this.xRe = Array.from({ length: this.P }, () => new Float64Array(this.N));
    this.xIm = Array.from({ length: this.P }, () => new Float64Array(this.N));
    this.head = 0; this.prev = new Float64Array(block);
    this.accRe = new Float64Array(this.N); this.accIm = new Float64Array(this.N);
  }
  process(input, output) {
    const B = this.B, N = this.N, P = this.P;
    this.head = (this.head + P - 1) % P;
    const re = this.xRe[this.head], im = this.xIm[this.head];
    for (let i = 0; i < B; i++) { re[i] = this.prev[i]; re[B + i] = input[i]; im[i] = 0; im[B + i] = 0; this.prev[i] = input[i]; }
    this.fft.transform(re, im, false);
    const aRe = this.accRe, aIm = this.accIm;
    aRe.fill(0); aIm.fill(0);
    for (let p = 0; p < P; p++) {
      const k = (this.head + p) % P, xr = this.xRe[k], xi = this.xIm[k], hr = this.hRe[p], hi = this.hIm[p];
      for (let i = 0; i < N; i++) { aRe[i] += xr[i] * hr[i] - xi[i] * hi[i]; aIm[i] += xr[i] * hi[i] + xi[i] * hr[i]; }
    }
    this.fft.transform(aRe, aIm, true);
    for (let i = 0; i < B; i++) output[i] = aRe[B + i] / N;
  }
}

// Synthesizer::initializeImpulseResponse: clip after the last sample louder than
// 100, keep at most 10000 samples and scale by the impulse response volume.
function esImpulseResponse(samples, volume) {
  let clipped = 0;
  for (let i = 0; i < samples.length; i++) if (Math.abs(samples[i]) > 100) clipped = i + 1;
  const n = Math.min(10000, clipped), out = new Float64Array(n);
  for (let i = 0; i < n; i++) out[i] = volume * samples[i] / 32767;
  return out;
}

// Synthesizer::writeInput: linear resampling of the simulation-rate exhaust
// signal to the audio rate, followed by the per-channel antialiasing filter.
class ESSynthesizerInput {
  constructor(channels, audioRate = 44100) {
    this.channels = channels; this.audioRate = audioRate; this.inputRate = 10000;
    this.offset = 0; this.index = 0; this.last = new Float64Array(channels);
    this.antialiasing = Array.from({ length: channels }, () => new ESButterworth(1900, audioRate));
    this.out = Array.from({ length: channels }, () => []);
  }
  setInputSampleRate(rate) { this.inputRate = rate; }
  writeInput(values) {
    const last = this.offset, ratio = this.audioRate / this.inputRate, next = last + ratio;
    for (; this.index <= next; this.index++) {
      const f = (this.index - last) / ratio;
      for (let c = 0; c < this.channels; c++) this.out[c].push(this.antialiasing[c].f(this.last[c] * (1 - f) + values[c] * f));
    }
    for (let c = 0; c < this.channels; c++) this.last[c] = values[c];
    this.offset = next;
    if (next > 1e6) { this.offset -= 1e6; this.index -= 1e6; }
  }
  drain() { const out = this.out.map(a => Float32Array.from(a)); this.out = Array.from({ length: this.channels }, () => []); return out; }
}

// Synthesizer::renderAudio, block by block. Takes per-channel input at the audio
// rate and returns int16-scaled samples divided by 32768.
const ES_LEVELER_LOOKAHEAD = 48;
// Linear up to the knee, then a smooth curve that never passes full scale.
function esSoftClip(x) {
  const knee = 0.8, a = Math.abs(x);
  if (a <= knee) return x;
  return Math.sign(x) * (knee + (1 - knee) * Math.tanh((a - knee) / (1 - knee)));
}

class ESSynthesizerCore {
  constructor(channels, impulseResponses, audioRate = 44100, block = 128) {
    this.channels = channels; this.audioRate = audioRate; this.block = block;
    this.params = { volume: 1, convolution: 1, dF_F_mix: 0.01, inputSampleNoise: 0.5, inputSampleNoiseFrequencyCutoff: 10000, airNoise: 1, airNoiseFrequencyCutoff: 2000, levelerTarget: 30000, levelerMaxGain: 1.9, levelerMinGain: 0.00001 };
    const dt = 1 / audioRate, rc = 1 / (10 * 2 * Math.PI);
    this.dcAlpha = dt / (rc + dt);
    this.filters = Array.from({ length: channels }, (_, i) => ({
      jitter: new ESJitter(10, 10000, audioRate), dc: 0, previous: 0,
      convolution: new ESConvolution(impulseResponses[i] && impulseResponses[i].length ? impulseResponses[i] : new Float64Array([0]), block),
    }));
    this.airNoise = new ESButterworth(2000, audioRate);
    this.antialiasing = new ESButterworth(audioRate * 0.45, audioRate);
    this.peak = 30000; this.attenuation = 1;
    this.vIn = new Float64Array(block); this.vConv = new Float64Array(block); this.sum = new Float64Array(block);
  }
  setParameters(p) { Object.assign(this.params, p); this.airNoise.setCutoff(this.params.airNoiseFrequencyCutoff, this.audioRate); }
  // inputs: one Float32Array of `block` samples per channel; output: Float32Array.
  render(inputs, output) {
    const p = this.params, B = this.block, dt = 1 / this.audioRate, sum = this.sum;
    const airNoise = p.airNoise, mix = p.dF_F_mix, conv = p.convolution;
    sum.fill(0);
    // The original draws the air noise from the filter of channel 0 for every
    // channel, so that filter advances once per channel per sample.
    const noise = this.noise || (this.noise = Array.from({ length: this.channels }, () => new Float64Array(B)));
    for (let i = 0; i < B; i++) for (let c = 0; c < this.channels; c++) noise[c][i] = this.airNoise.f(2 * Math.random() - 1);
    for (let c = 0; c < this.channels; c++) {
      const flt = this.filters[c], input = inputs[c], vIn = this.vIn;
      flt.jitter.scale = p.inputSampleNoise;
      for (let i = 0; i < B; i++) {
        const fIn = flt.jitter.f(input[i]);
        flt.dc = this.dcAlpha * fIn + (1 - this.dcAlpha) * flt.dc;
        const f = fIn - flt.dc;
        const fP = (fIn - flt.previous) / dt; flt.previous = fIn;
        const r = noise[c][i];
        const rMixed = airNoise * r + (1 - airNoise);
        vIn[i] = fP * mix + f * rMixed * (1 - mix);
      }
      flt.convolution.process(vIn, this.vConv);
      for (let i = 0; i < B; i++) sum[i] += conv * this.vConv[i] + (1 - conv) * vIn[i];
    }
    // LevelingFilter (src/leveling_filter.cpp). The gain follows the peak of
    // the live signal but is applied to a copy delayed by a few samples, so a
    // firing pulse is turned down before it reaches the output instead of
    // being hard clipped at int16 like the original does on loud engines.
    const look = this.lookahead || (this.lookahead = new Float64Array(ES_LEVELER_LOOKAHEAD));
    for (let i = 0; i < B; i++) {
      const signal = this.antialiasing.f(sum[i]);
      this.peak *= 0.999;
      if (Math.abs(signal) > this.peak) this.peak = Math.abs(signal);
      const delayed = look[this.lookN = ((this.lookN | 0) + 1) % look.length];
      look[this.lookN] = signal;
      if (this.peak !== 0) {
        const raw = Math.min(Math.max(p.levelerTarget / this.peak, p.levelerMinGain), p.levelerMaxGain);
        this.attenuation = 0.9 * this.attenuation + 0.1 * raw;
      }
      const v = delayed * this.attenuation * p.volume / 32768;
      if (v > 1 || v < -1) this.overs = (this.overs | 0) + 1;
      output[i] = esSoftClip(v);
    }
  }
}
/* ---------------------------------------------------------------- Vibration */

// Not in engine-sim, which is 2D: where each cylinder sits along the
// crankshaft. Cylinders go front to back in the engine's axialOrder, or by
// journal; ones sharing a journal sit one bore and a fifth apart, the next
// journal a quarter bore further. Returns metres from the middle, per cylinder.
function esAxialPositions(engine) {
  const cyls = engine.cylinders, unit = 1.2 * Math.max(...engine.banks.map(b => b.bore));
  const order = engine.axialOrder || cyls.map((c, i) => i).sort((a, b) => cyls[a].journal - cyls[b].journal || cyls[a].bank.index - cyls[b].bank.index);
  const z = new Float64Array(cyls.length);
  let pos = 0;
  order.forEach((i, k) => {
    if (k > 0) pos += cyls[i].journal === cyls[order[k - 1]].journal ? unit : 1.25 * unit;
    z[i] = pos;
  });
  for (let i = 0; i < z.length; i++) z[i] -= pos / 2;
  return { z, unit };
}

// Not in engine-sim. Free shaking forces and rocking moments of the
// reciprocating masses (piston plus a third of the rod), from the engine's own
// slider-crank geometry over one 720° cycle, split into crank orders. An order
// k vibrates at k × rpm / 60 Hz. The first order part that turns with the crank
// is left out, since crank counterweights cancel it; what is left needs a
// balance shaft or is felt. Moments use esAxialPositions.
// The firing pulses add a torque at order cylinders / 2.
function esVibrationAnalysis(engine) {
  const N = 720, maxHalfOrder = 24, cyls = engine.cylinders, nJ = engine.crank.journals.length;
  const m = engine.pistonSpec.mass + engine.rodSpec.mass / 3, r = engine.crank.throw;
  const axial = esAxialPositions(engine), pitch = axial.unit;
  const theta0 = engine.theta;
  const fx = new Float64Array(N), fy = new Float64Array(N), mx = new Float64Array(N), my = new Float64Array(N);
  for (let n = 0; n < N; n++) {
    engine.theta = 2 * ES_TAU * n / N;
    engine.computeGeometry();
    for (const c of cyls) {
      const f = m * c.dds, z = axial.z[c.global];
      const x = f * c.bank.dx, y = f * c.bank.dy;
      fx[n] += x; fy[n] += y; mx[n] -= z * y; my[n] += z * x;
    }
  }
  engine.theta = theta0; engine.computeGeometry();
  // Amplitudes of the parts of a 2D signal turning with and against the crank.
  const rotating = (a, b, k) => {
    let ar = 0, ai = 0, br = 0, bi = 0;
    for (let n = 0; n < N; n++) {
      const t = 2 * ES_TAU * n / N * k, c = Math.cos(t), s = Math.sin(t);
      ar += a[n] * c; ai -= a[n] * s; br += b[n] * c; bi -= b[n] * s;
    }
    ar *= 2 / N; ai *= 2 / N; br *= 2 / N; bi *= 2 / N;
    // X + iY and X - iY with X = ar + i ai, Y = br + i bi.
    return { with: Math.hypot(ar - bi, ai + br) / 2, against: Math.hypot(ar + bi, ai - br) / 2 };
  };
  const orders = [], forceUnit = m * r, momentUnit = m * r * pitch;
  for (let h = 1; h <= maxHalfOrder; h++) {
    const k = h / 2;
    for (const [kind, a, b, unit] of [['force', fx, fy, forceUnit], ['moment', mx, my, momentUnit]]) {
      const p = rotating(a, b, k), amp = (k === 1 ? p.against : p.with + p.against) / unit;
      if (amp > 0.05) orders.push({ order: k, kind, amplitude: amp });
    }
  }
  const firingOrder = cyls.length / 2;
  orders.sort((a, b) => b.amplitude - a.amplitude);
  const dominant = orders.length ? orders[0] : { order: firingOrder, kind: 'torque', amplitude: 0 };
  return { orders, firingOrder, dominant };
}

/* ------------------------------------------------------------------ Engines */

// Engine descriptions transcribed from the engine-sim scripts in assets/engines.
// Lengths in m, angles in rad, masses in kg, flow rates in scfm (k_carb and
// k_28inH2O are applied when the engine is built), port flow tables in CFM at
// 28 inH2O per 50 thou of lift.
const U = ES_UNITS;
const esDisk = (mass, radius) => 0.5 * mass * radius * radius;
const esRodMoment = (mass, length) => mass * length * length / 12;
const esCircleArea = r => Math.PI * r * r;
const ES_CYCLE = 4 * Math.PI;

function esTurbulence(scale) { return [0, 5, 10, 15, 20, 25, 30, 35, 40, 45].map((x, i) => i === 0 ? 3 : scale(x) * x); }

const ES_ENGINES = {};

// assets/engines/atg-video-2/01_subaru_ej25_eh.mr (the default engine in assets/main.mr).
ES_ENGINES.subaru_ej25 = (() => {
  const stroke = 79 * U.mm, bore = 99.5 * U.mm, rod = 5.142 * U.inch, rodMass = 535 * U.g, ch = 1 * U.inch;
  const crankMass = 9.39, flywheelMass = 6.8, flywheelRadius = 6 * U.inch;
  const ilc = 117 * U.deg, elc = 112 * U.deg, r360 = 2 * Math.PI;
  const intakeLobe = { duration: 232 * U.deg, gamma: 2, lift: 9.78 * U.mm, steps: 100 };
  const exhaustLobe = { duration: 236 * U.deg, gamma: 2, lift: 9.60 * U.mm, steps: 100 };
  const cam = (lobe, base, ks) => ({ lobe, baseRadius: 17 * U.mm, lobes: ks.map(k => base + k / 4 * ES_CYCLE) });
  return {
    name: 'Subaru EJ25', starterTorque: 70 * U.lbft, starterSpeed: 500 * U.rpm, redline: 6500 * U.rpm,
    // Not in the script (engine-sim is 2D): cylinders front to back, so that
    // the crank reads 0-180-180-0 with the banks alternating, as on the EJ.
    axialOrder: [0, 2, 1, 3],
    throttleGamma: 2, hfGain: 0.01, noise: 1.0, jitter: 0.5, simulationFrequency: 20000,
    fuel: { maxBurningEfficiency: 0.9, turbulenceToFlameSpeedRatio: esTurbulence(x => x < 15 ? 1.5 : x < 20 ? 1.1 * 1.5 : 1.25 * 1.5) },
    crankshaft: {
      throw: stroke / 2, mass: crankMass, flywheelMass, frictionTorque: 1 * U.lbft, tdc: 180 * U.deg,
      momentOfInertia: esDisk(crankMass, stroke / 2) + 2 * esDisk(flywheelMass, flywheelRadius) + esDisk(10, 6 * U.cm),
      journals: [0, 180 * U.deg, 0, 180 * U.deg],
    },
    piston: { mass: (414 + 152) * U.g, compressionHeight: ch, displacement: 0 },
    rod: { mass: rodMass, momentOfInertia: esRodMoment(rodMass, rod), length: rod },
    bore, deckHeight: stroke / 2 + rod + ch,
    head: {
      chamberVolume: 67 * U.cc, intakeRunnerVolume: 149.6 * U.cc, intakeRunnerArea: (1.75 * U.inch) ** 2,
      exhaustRunnerVolume: 50 * U.cc, exhaustRunnerArea: (1.25 * U.inch) ** 2,
      intakeFlow: [0, 58, 103, 156, 214, 249, 268, 280, 280, 281], exhaustFlow: [0, 37, 72, 113, 160, 196, 222, 235, 245, 246],
    },
    intake: { plenumVolume: 1.325 * U.L, plenumArea: 20 * U.cm2, flowRate: 400, runnerFlowRate: 100, runnerLength: 12 * U.inch, idleFlowRate: 0, idlePlate: 0.9978, velocityDecay: 1.0 },
    exhausts: [{ length: 500 * U.mm, collectorArea: esCircleArea(2 * U.inch), outletFlowRate: 1000, primaryTubeLength: 40 * U.inch, primaryFlowRate: 400, audioVolume: 0.5 * 0.02, velocityDecay: 1.0, impulseResponse: 'minimal_muffling_02' }],
    banks: [
      {
        angle: 90 * U.deg, flip: true,
        valvetrain: { intakeCam: cam(intakeLobe, r360 + ilc, [0, 1]), exhaustCam: cam(exhaustLobe, r360 - elc, [0, 1]) },
        cylinders: [
          { journal: 0, wire: 0, primaryLength: 2 * U.inch, soundAttenuation: 0.9, blowby: 0.001 },
          { journal: 3, wire: 2, primaryLength: 3 * U.inch, soundAttenuation: 1.0, blowby: 0.002 },
        ],
      },
      {
        angle: -90 * U.deg, flip: false,
        valvetrain: { intakeCam: cam(intakeLobe, r360 + ilc, [2, 3]), exhaustCam: cam(exhaustLobe, r360 - elc, [2, 3]) },
        cylinders: [
          { journal: 1, wire: 1, primaryLength: 3 * U.inch, soundAttenuation: 1.1, blowby: 0.001 },
          { journal: 2, wire: 3, primaryLength: 5 * U.inch, soundAttenuation: 0.9, blowby: 0.002 },
        ],
      },
    ],
    // Wire angles by wire number - 1.
    ignition: {
      timing: [[0, 25], [1000, 25], [2000, 30], [3000, 40], [4000, 40]].map(([r, a]) => [r * U.rpm, a * U.deg]),
      revLimit: 6800 * U.rpm, limiterDuration: 0.16,
      wires: [0, 2 / 4 * ES_CYCLE, 1 / 4 * ES_CYCLE, 3 / 4 * ES_CYCLE],
    },
    vehicle: { mass: 2700 * U.lb, dragCoefficient: 0.3, crossSectionArea: (72 * U.inch) * (56 * U.inch), diffRatio: 3.9, tireRadius: 10 * U.inch, rollingResistance: 0.015 * 2700 * U.lb * 9.81 },
    transmission: { maxClutchTorque: 300 * U.lbft, gears: [3.636, 2.375, 1.761, 1.346, 0.971, 0.756] },
  };
})();

// assets/engines/atg-video-1/05_honda_vtec.mr. This engine turns the other way
// (negative starter speed), with negative timing values to match.
ES_ENGINES.honda_b18c5 = (() => {
  const stroke = 87.2 * U.mm, bore = 81 * U.mm, rod = 5.43 * U.inch, ch = 1 * U.inch;
  const r360 = 2 * Math.PI, order = [0, 3, 1, 2];
  const lobe = (duration, gamma, lift) => ({ duration: duration * U.deg, gamma, lift: lift * U.mm, steps: 100 });
  const cam = (l, base) => ({ lobe: l, baseRadius: 500 * U.thou, lobes: order.map(k => base - k / 4 * ES_CYCLE) });
  const es = { collectorArea: esCircleArea(2 * U.inch), outletFlowRate: 1000, primaryTubeLength: 10 * U.inch, primaryFlowRate: 200, velocityDecay: 1.0, impulseResponse: 'mild_exhaust_0' };
  es.length = 100 * U.L / es.collectorArea;
  return {
    name: 'Honda B18C5 [VTEC, I4]', starterTorque: 70 * U.lbft, starterSpeed: -500 * U.rpm, redline: 8400 * U.rpm,
    throttleGamma: 2, hfGain: 0.002, noise: 0.253, jitter: 0.195, simulationFrequency: 20000,
    fuel: { maxTurbulenceEffect: 2.5, maxBurningEfficiency: 0.75 },
    crankshaft: {
      throw: stroke / 2, mass: 35.5 * U.lb, flywheelMass: 10 * U.lb, frictionTorque: 1 * U.lbft, tdc: 90 * U.deg,
      momentOfInertia: 0.22986844776863666 * 0.5, journals: [0, 180 * U.deg, 180 * U.deg, 0],
    },
    piston: { mass: 303.5 * U.g, compressionHeight: ch, displacement: 0 },
    rod: { mass: 395.837 * U.g, momentOfInertia: 0.0015884918028487504, length: rod },
    bore, deckHeight: stroke / 2 + rod + ch,
    head: {
      chamberVolume: 41.6 * U.cc, intakeRunnerVolume: 149.6 * U.cc, intakeRunnerArea: (1.35 * U.inch) ** 2,
      exhaustRunnerVolume: 50 * U.cc, exhaustRunnerArea: (1.25 * U.inch) ** 2,
      intakeFlow: [0, 50, 80, 125, 160, 190, 210, 225, 230, 250], exhaustFlow: [0, 50, 80, 110, 130, 150, 160, 170, 170, 170],
    },
    intake: { plenumVolume: 1.325 * U.L, plenumArea: 20 * U.cm2, flowRate: 800, runnerFlowRate: 250, runnerLength: 7 * U.inch, idleFlowRate: 0, idlePlate: 0.9989, velocityDecay: 0.5 },
    exhausts: [Object.assign({}, es, { audioVolume: 8 * 0.75 }), Object.assign({}, es, { audioVolume: 8 * 1.0 })],
    banks: [{
      angle: 0, flip: false,
      valvetrain: {
        intakeCam: cam(lobe(210, 1, 6.9), r360 - 116 * U.deg),
        exhaustCam: cam(lobe(190, 1, 6.5), r360 + 116 * U.deg),
        vtec: {
          intakeCam: cam(lobe(240, 0.5, 11.5), r360 - 100 * U.deg),
          exhaustCam: cam(lobe(232, 0.5, 10.5), r360 + 100 * U.deg),
          minRpm: 5800 * U.rpm, minSpeed: 10 * U.mph, manifoldVacuum: ES_ATM - 5 * U.inHg, minThrottlePosition: 0.3,
        },
      },
      cylinders: [
        { journal: 0, wire: 0, exhaust: 0, soundAttenuation: 0.9, blowby: 0.001 },
        { journal: 1, wire: 1, exhaust: 1, soundAttenuation: 1.1, blowby: 0.002 },
        { journal: 2, wire: 2, exhaust: 0, soundAttenuation: 0.8, blowby: 0.001 },
        { journal: 3, wire: 3, exhaust: 1, soundAttenuation: 0.9, blowby: 0.002 },
      ],
    }],
    ignition: {
      timing: [[0, -25], [1000, -25], [2000, -30], [3000, -40], [4000, -40]].map(([r, a]) => [r * U.rpm, a * U.deg]),
      revLimit: 9400 * U.rpm, limiterDuration: 0.05,
      wires: [0, 1 / 4 * ES_CYCLE, 3 / 4 * ES_CYCLE, 2 / 4 * ES_CYCLE],
    },
    vehicle: { mass: 2400 * U.lb, dragCoefficient: 0.2, crossSectionArea: (66 * U.inch) * (50 * U.inch), diffRatio: 3.55, tireRadius: 10 * U.inch, rollingResistance: 300 },
    transmission: { maxClutchTorque: 300 * U.lbft, gears: [3.23, 2.105, 1.458, 1.107, 0.848] },
  };
})();

// assets/engines/atg-video-2/04_60_degree_v6.mr
ES_ENGINES.v6_60 = (() => {
  const stroke = 3.48 * U.inch, bore = 3.5 * U.inch, rod = 5.142 * U.inch, rodMass = 535 * U.g, ch = 1 * U.inch;
  const r360 = 2 * Math.PI, ilc = 117 * U.deg, elc = 112 * U.deg, spacing = 5 * U.inch;
  const intakeLobe = { duration: 222 * U.deg, gamma: 1, lift: 400 * U.thou, steps: 100 };
  const exhaustLobe = { duration: 226 * U.deg, gamma: 1, lift: 300 * U.thou, steps: 100 };
  const cam = (lobe, base, offsets) => ({ lobe, baseRadius: 0.75 * U.inch, lobes: offsets.map(o => base + o * U.deg) });
  const es = { length: 100 * U.inch, collectorArea: esCircleArea(2 * U.inch), outletFlowRate: 1000, primaryTubeLength: 20 * U.inch, primaryFlowRate: 500, velocityDecay: 1.0, audioVolume: 1.0, impulseResponse: 'mild_exhaust_0_reverb' };
  return {
    name: 'Generic 60 deg. V6', starterTorque: 70 * U.lbft, starterSpeed: 500 * U.rpm, redline: 5500 * U.rpm, throttleGamma: 2,
    crankshaft: {
      throw: stroke / 2, mass: 50 * U.lb, flywheelMass: 30 * U.lb, frictionTorque: 20 * U.lbft, tdc: 120 * U.deg,
      momentOfInertia: esDisk(50 * U.lb, stroke) + esDisk(30 * U.lb, 7 * U.inch) + esDisk(10, 1 * U.cm),
      journals: [0, 60, 240, 300, 120, 180].map(a => a * U.deg),
    },
    piston: { mass: (414 + 152) * U.g, compressionHeight: ch, displacement: 0 },
    rod: { mass: rodMass, momentOfInertia: esRodMoment(rodMass, rod), length: rod },
    bore, deckHeight: stroke / 2 + rod + ch,
    head: {
      chamberVolume: 67 * U.cc, intakeRunnerVolume: 149.6 * U.cc, intakeRunnerArea: (1.35 * U.inch) ** 2,
      exhaustRunnerVolume: 50 * U.cc, exhaustRunnerArea: (2 * U.inch) ** 2,
      intakeFlow: [0, 58, 103, 156, 214, 249, 268, 280, 280, 281], exhaustFlow: [0, 37, 72, 113, 160, 196, 222, 235, 245, 246],
    },
    intake: { plenumVolume: 1.325 * U.L, plenumArea: 20 * U.cm2, flowRate: 400, runnerFlowRate: 250, runnerLength: 4 * U.inch, idleFlowRate: 0, idlePlate: 0.994, velocityDecay: 0.5 },
    exhausts: [Object.assign({}, es), Object.assign({}, es)],
    banks: [
      {
        angle: 30 * U.deg, flip: true,
        valvetrain: { intakeCam: cam(intakeLobe, r360 + ilc, [0, 240, 480]), exhaustCam: cam(exhaustLobe, r360 - elc, [0, 240, 480]) },
        cylinders: [
          { journal: 0, wire: 0, exhaust: 0, soundAttenuation: 0.9, primaryLength: 2 * spacing, blowby: 0.1 },
          { journal: 2, wire: 2, exhaust: 0, soundAttenuation: 0.95, primaryLength: spacing, blowby: 0.2 },
          { journal: 4, wire: 4, exhaust: 0, soundAttenuation: 1.0, primaryLength: 0, blowby: 0.2 },
        ],
      },
      {
        angle: -30 * U.deg, flip: false,
        valvetrain: { intakeCam: cam(intakeLobe, r360 + ilc, [120, 360, 600]), exhaustCam: cam(exhaustLobe, r360 - elc, [120, 360, 600]) },
        cylinders: [
          { journal: 1, wire: 1, exhaust: 1, soundAttenuation: 0.95, primaryLength: 2 * spacing, blowby: 0.1 },
          { journal: 3, wire: 3, exhaust: 1, soundAttenuation: 0.9, primaryLength: spacing, blowby: 0.2 },
          { journal: 5, wire: 5, exhaust: 1, soundAttenuation: 1.0, primaryLength: 0, blowby: 0.2 },
        ],
      },
    ],
    ignition: {
      timing: [[0, 12], [1000, 20], [2000, 25], [3000, 30], [4000, 30]].map(([r, a]) => [r * U.rpm, a * U.deg]),
      revLimit: 5600 * U.rpm, limiterDuration: 0.2,
      wires: [0, 120, 240, 360, 480, 600].map(a => a * U.deg),
    },
    vehicle: { mass: 2700 * U.lb, dragCoefficient: 0.3, crossSectionArea: (72 * U.inch) * (56 * U.inch), diffRatio: 3.9, tireRadius: 10 * U.inch, rollingResistance: 0.015 * 2700 * U.lb * 9.81 },
    transmission: { maxClutchTorque: 300 * U.lbft, gears: [3.636, 2.375, 1.761, 1.346, 0.971, 0.756] },
  };
})();

// assets/engines/atg-video-2/07_gm_ls.mr
ES_ENGINES.gm_ls = (() => {
  const stroke = 3.622 * U.inch, bore = 3.78 * U.inch, rod = 160 * U.mm, rodMass = 50 * U.g, ch = 1 * U.inch;
  const r360 = 2 * Math.PI, rot = 90 * U.deg, lc = 116 * U.deg, vAngle = 90 * U.deg, spacing = 2 * U.inch;
  const intakeLobe = { duration: 234 * U.deg, gamma: 1.1, lift: 551 * U.thou, steps: 256 };
  const exhaustLobe = { duration: 235 * U.deg, gamma: 1.1, lift: 551 * U.thou, steps: 256 };
  const cam = (lobe, base, ks) => ({ lobe, baseRadius: 1 * U.inch, lobes: ks.map(k => base + k * rot) });
  const es = { collectorArea: esCircleArea(2 * U.inch), outletFlowRate: 1000, primaryTubeLength: 29 * U.inch, primaryFlowRate: 500, velocityDecay: 1.0, audioVolume: 4.0, impulseResponse: 'default_0' };
  const cyl = (journal, wire, exhaust, primaryLength) => ({ journal, wire: wire - 1, exhaust, soundAttenuation: 1, primaryLength, blowby: 0 });
  return {
    name: 'GM LS', starterTorque: 200 * U.lbft, starterSpeed: 200 * U.rpm, redline: 6500 * U.rpm, throttleGamma: 2,
    hfGain: 0.01, noise: 1.0, jitter: 0.6, simulationFrequency: 10000,
    fuel: { maxBurningEfficiency: 1.0, turbulenceToFlameSpeedRatio: esTurbulence(x => x < 10 ? 1.5 : x < 15 ? 1.75 : 2.0) },
    crankshaft: {
      throw: stroke / 2, mass: 60 * U.lb, flywheelMass: 30 * U.lb, frictionTorque: 20 * U.lbft, tdc: 90 * U.deg - vAngle / 2,
      momentOfInertia: 1.5 * esDisk(60 * U.lb, stroke) + esDisk(30 * U.lb, 8 * U.inch) + esDisk(1, 1 * U.cm),
      journals: [0, 270, 90, 180].map(a => a * U.deg),
    },
    piston: { mass: 100 * U.g, compressionHeight: ch, displacement: 0 },
    rod: { mass: rodMass, momentOfInertia: esRodMoment(rodMass, rod), length: rod },
    bore, deckHeight: stroke / 2 + rod + ch,
    head: {
      chamberVolume: 90 * U.cc, intakeRunnerVolume: 149.6 * U.cc, intakeRunnerArea: (2.2 * U.inch) ** 2,
      exhaustRunnerVolume: 50 * U.cc, exhaustRunnerArea: (1.75 * U.inch) ** 2,
      intakeFlow: [0, 1, 103, 156, 214, 249, 268, 280, 280, 281], exhaustFlow: [0, 1, 72, 113, 160, 196, 222, 235, 245, 246],
    },
    intake: { plenumVolume: 1.325 * U.L, plenumArea: 20 * U.cm2, flowRate: 700, runnerFlowRate: 100, runnerLength: 12 * U.inch, idleFlowRate: 0, idlePlate: 0.996, velocityDecay: 0.5 },
    exhausts: [Object.assign({}, es, { length: 100 * U.inch }), Object.assign({}, es, { length: 172 * U.inch })],
    banks: [
      {
        angle: -vAngle / 2, flip: false,
        valvetrain: { intakeCam: cam(intakeLobe, r360 + lc, [0, 7, 5, 2]), exhaustCam: cam(exhaustLobe, r360 - lc, [0, 7, 5, 2]) },
        cylinders: [cyl(0, 1, 0, 3 * spacing + 2 * U.cm), cyl(1, 3, 0, 2 * spacing + 1 * U.cm), cyl(2, 5, 0, spacing + 3 * U.cm), cyl(3, 7, 0, 5 * U.cm)],
      },
      {
        angle: vAngle / 2, flip: true,
        valvetrain: { intakeCam: cam(intakeLobe, r360 + lc, [3, 6, 4, 1]), exhaustCam: cam(exhaustLobe, r360 - lc, [3, 6, 4, 1]) },
        cylinders: [cyl(0, 2, 1, 3 * spacing + 1 * U.cm), cyl(1, 4, 1, 2 * spacing + 5 * U.cm), cyl(2, 6, 1, spacing + 7 * U.cm), cyl(3, 8, 1, 0)],
      },
    ],
    ignition: {
      timing: [[0, 12], [1000, 12], [2000, 20], [3000, 30], [4000, 40], [5000, 40], [6000, 40], [7000, 40], [8000, 40]].map(([r, a]) => [r * U.rpm, a * U.deg]),
      revLimit: 6800 * U.rpm, limiterDuration: 0.2,
      // wire1..wire8 connected at 0, 3, 7, 6, 5, 4, 2, 1 times 90 degrees.
      wires: [0, 3, 7, 6, 5, 4, 2, 1].map(k => k * 90 * U.deg),
    },
    vehicle: { mass: 1614, dragCoefficient: 0.3, crossSectionArea: (72 * U.inch) * (50 * U.inch), diffRatio: 3.42, tireRadius: 10 * U.inch, rollingResistance: 200 },
    transmission: { maxClutchTorque: 500 * U.lbft, gears: [2.97, 2.07, 1.43, 1.0, 0.71, 0.57] },
  };
})();

const ES_ENGINE_ORDER = ['subaru_ej25', 'honda_b18c5', 'v6_60', 'gm_ls'];

if (typeof module !== 'undefined') {
  module.exports = {
    ES_ENGINES, ES_ENGINE_ORDER, ES_UNITS, ES_ATM, ESSimulator, ESEngine, ESSynthesizerInput, ESSynthesizerCore,
    ESConvolution, ESFFT, ESButterworth, ESJitter, esImpulseResponse, esVibrationAnalysis, esAxialPositions, ESGasSystem, esFlow, esFlowRate, esKCarb, esK28, esHarmonicCamLobe,
  };
}
