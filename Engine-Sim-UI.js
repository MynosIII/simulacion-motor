/* Engine-Sim-UI.js
 * JavaScript port of the engine-sim user interface and engine view,
 * Copyright 2022 AngeTheGreat (Ange Yaghi), MIT.
 * Source: https://github.com/ange-yaghi/engine-sim, commit 85f7c3b959a908ed5232ede4f1a4ac7eafe6b630.
 * See PROCEDENCIA.md.
 *
 * The original draws triangles from GeometryGenerator through delta-studio.
 * Here the same shapes are filled on a 2D canvas, in the original's pixel
 * coordinates (origin at the bottom left, y up). Colors are linear and mixed
 * in linear space, then written as sRGB like the delta-studio shader does.
 *
 * Ported: ui_math.h, UiElement, UiManager, Gauge, LabeledGauge, Oscilloscope,
 * OscilloscopeCluster, RightGaugeCluster, FiringOrderDisplay, ThrottleDisplay,
 * AfrCluster, FuelCluster, PerformanceCluster, LoadSimulationCluster,
 * MixerCluster, InfoCluster, EngineView, the simulation objects (connecting
 * rod, piston, combustion chamber, crankshaft, cylinder bank, cylinder head)
 * and EngineSimApplication's frame loop, controls and layouts.
 */

/* --------------------------------------------------------------- ui_math.h */

const ESRef = {
  center: [0.5, 0.5], tl: [0, 1], tr: [1, 0], tm: [0.5, 1],
  bl: [0, 0], br: [1, 0], bm: [0.5, 0], lm: [0, 0.5], rm: [1, 0.5],
};

class ESBounds {
  constructor(x0, y0, x1, y1) {
    this.x0 = Math.min(x0, x1); this.y0 = Math.min(y0, y1);
    this.x1 = Math.max(x0, x1); this.y1 = Math.max(y0, y1);
  }
  // Bounds(width, height, pos, ref)
  static sized(w, h, px, py, ref = ESRef.tl) { return new ESBounds(0, 0, w, h).moved(px - ref[0] * w, py - ref[1] * h); }
  width() { return this.x1 - this.x0; }
  height() { return this.y1 - this.y0; }
  pos(ref = ESRef.tl) { return [this.x0 + ref[0] * this.width(), this.y0 + ref[1] * this.height()]; }
  inset(a) { return new ESBounds(this.x0 + a, this.y0 + a, this.x1 - a, this.y1 - a); }
  moved(dx, dy) { return new ESBounds(this.x0 + dx, this.y0 + dy, this.x1 + dx, this.y1 + dy); }
  verticalSplit(s0, s1) {
    const a = Math.min(s0, s1), b = Math.max(s0, s1), h = this.height();
    return new ESBounds(this.x0, this.y0 + h * a, this.x1, this.y0 + h * b);
  }
  horizontalSplit(s0, s1) {
    const a = Math.min(s0, s1), b = Math.max(s0, s1), w = this.width();
    return new ESBounds(this.x0 + w * a, this.y0, this.x0 + w * b, this.y1);
  }
  overlaps(x, y) { return x >= this.x0 && y >= this.y0 && x <= this.x1 && y <= this.y1; }
}

// Grid { h_cells, v_cells }.get(): cells are counted from the top left.
function esGrid(a, hCells, vCells, x, y, w = 1, h = 1) {
  const cw = a.width() / hCells, ch = a.height() / vCells;
  const p = a.pos(ESRef.tl);
  return ESBounds.sized(cw * w, ch * h, p[0] + x * cw, p[1] - y * ch, ESRef.tl);
}

/* ------------------------------------------------------------------ Colors */

function esSrgbToLinear(u) { return u <= 0.04045 ? u / 12.92 : Math.pow((u + 0.055) / 1.055, 2.4); }
function esLinearToSrgb(u) { return u <= 0.0031308 ? 12.92 * u : 1.055 * Math.pow(Math.abs(u), 1 / 2.4) - 0.055; }
// ysColor::srgbiToLinear
function esColor(hex, a = 1) {
  return [(hex >> 16) & 255, (hex >> 8) & 255, hex & 255].map(c => esSrgbToLinear(c / 255)).concat([a]);
}
function esMix(a, b, s) { return a.map((v, i) => v * (1 - s) + b[i] * s); }
const esCssCache = new Map();
function esCss(c) {
  const key = c.join(',');
  let s = esCssCache.get(key);
  if (!s) {
    const v = c.slice(0, 3).map(x => Math.round(255 * Math.min(1, Math.max(0, esLinearToSrgb(x)))));
    s = `rgba(${v[0]},${v[1]},${v[2]},${c[3]})`;
    if (esCssCache.size > 4096) esCssCache.clear();
    esCssCache.set(key, s);
  }
  return s;
}

// ApplicationSettings defaults.
const ES_COLORS = {
  background: esColor(0x0E1012), foreground: esColor(0xFFFFFF), shadow: esColor(0x0E1012),
  highlight1: esColor(0xEF4545), highlight2: esColor(0xFFFFFF), pink: esColor(0xF394BE),
  red: esColor(0xEE4445), orange: esColor(0xF4802A), yellow: esColor(0xFDBD2E),
  blue: esColor(0x77CEE0), green: esColor(0xBDD869),
};

/* ----------------------------------------------------------------- Painter */

// Fills the shapes GeometryGenerator would produce. Every subshape is added
// counter-clockwise so overlapping pieces of one fill never cancel out.
class ESPainter {
  constructor(ctx) { this.ctx = ctx; this.fontSize = 0; }
  begin() { this.ctx.beginPath(); }
  fill(color) { this.ctx.fillStyle = esCss(color); this.ctx.fill('nonzero'); }
  quad(ax, ay, bx, by, cx, cy, dx, dy) {
    const c = this.ctx;
    const area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay) + (cx - ax) * (dy - ay) - (dx - ax) * (cy - ay);
    if (area >= 0) { c.moveTo(ax, ay); c.lineTo(bx, by); c.lineTo(cx, cy); c.lineTo(dx, dy); }
    else { c.moveTo(dx, dy); c.lineTo(cx, cy); c.lineTo(bx, by); c.lineTo(ax, ay); }
    c.closePath();
  }
  // generateLine2d
  line(x0, y0, x1, y1, w) {
    const dx = x1 - x0, dy = y1 - y0, l = Math.hypot(dx, dy);
    if (!(l > 0)) return;
    const px = -dy / l * w / 2, py = dx / l * w / 2;
    this.quad(x0 + px, y0 + py, x0 - px, y0 - py, x1 - px, y1 - py, x1 + px, y1 + py);
  }
  // generateCircle2d
  circle(x, y, r) { if (r > 0) { this.ctx.moveTo(x + r, y); this.ctx.arc(x, y, r, 0, ES_TAU, false); this.ctx.closePath(); } }
  // generateRing2d, from startAngle to endAngle.
  ring(x, y, ri, ro, a0, a1) {
    if (!(a1 > a0)) return;
    const c = this.ctx;
    c.moveTo(x + ro * Math.cos(a0), y + ro * Math.sin(a0));
    c.arc(x, y, ro, a0, a1, false);
    c.arc(x, y, ri, a1, a0, true);
    c.closePath();
  }
  // generateFrame
  frame(b, lw) {
    const x0 = b.x0, x1 = b.x1, y0 = b.y0, y1 = b.y1;
    this.line(x0 - lw / 2, y1, x1 + lw / 2, y1, lw);
    this.line(x0 - lw / 2, y0, x1 + lw / 2, y0, lw);
    this.line(x1, y0 - lw / 2, x1, y1 + lw / 2, lw);
    this.line(x0, y0 - lw / 2, x0, y1 + lw / 2, lw);
  }
  box(b) { this.ctx.rect(b.x0, b.y0, b.width(), b.height()); }
  model(path, color) { this.ctx.fillStyle = esCss(color); this.ctx.fill(path, 'nonzero'); }
  setFont(h) { if (h !== this.fontSize) { this.ctx.font = `${h}px Silkscreen`; this.fontSize = h; } }
  textWidth(s, h) { this.setFont(h); return this.ctx.measureText(s).width; }
  // TextRenderer::RenderText: the baseline starts at (x, y). Text goes through
  // the console renderer, which draws after all geometry, so it is queued here.
  text(s, x, y, h, color) { (this.queue || (this.queue = [])).push([s, x, y, h, color]); }
  flushText() {
    const c = this.ctx, q = this.queue || [];
    for (const [s, x, y, h, color] of q) {
      this.setFont(h);
      c.fillStyle = esCss(color);
      c.save(); c.translate(x, y); c.scale(1, -1); c.fillText(s, 0, 0); c.restore();
    }
    this.queue = [];
  }
}

/* --------------------------------------------------------------- UiElement */

class ESUiElement {
  constructor(app) {
    this.app = app; this.children = []; this.parent = null;
    this.bounds = new ESBounds(0, 0, 0, 0); this.mouseBounds = new ESBounds(0, 0, 0, 0);
    this.checkMouse = false; this.visible = true; this.signalTarget = null;
  }
  add(element, signalTarget = null) { element.parent = this; element.signalTarget = signalTarget; this.children.push(element); return element; }
  update(dt) { for (const c of this.children) c.update(dt); }
  render() { for (const c of this.children) if (c.visible) c.render(); }
  signal() { }
  onMouseDown() { }
  onMouseUp() { }
  onMouseClick() { if (this.signalTarget) this.signalTarget.signal(this, 'clicked'); }
  onDrag() { }
  onMouseScroll() { }
  mouseOver(x, y) {
    for (let i = this.children.length - 1; i >= 0; i--) {
      const c = this.children[i];
      if (!c.visible) continue;
      const hit = c.mouseOver(x, y);
      if (hit) return hit;
    }
    return this.checkMouse && this.mouseBounds.overlaps(x, y) ? this : null;
  }
  bringToFront(element) { this.children.splice(this.children.indexOf(element), 1); this.children.push(element); }
  activate() { if (this.parent) { this.parent.bringToFront(this); this.parent.activate(); } }

  get p() { return this.app.painter; }
  drawFrame(bounds, thickness, frameColor, fillColor, fill = true) {
    const p = this.p;
    if (fill) { p.begin(); p.box(bounds); p.fill(fillColor); }
    p.begin(); p.frame(bounds, thickness); p.fill(frameColor);
  }
  drawBox(bounds, color) { const p = this.p; p.begin(); p.box(bounds); p.fill(color); }
  drawText(s, bounds, h, ref = ESRef.tl) {
    const o = bounds.pos(ref);
    this.p.text(s, o[0], o[1] - h / 4, h, this.app.colors.foreground);
  }
  drawAlignedText(s, bounds, h, ref = ESRef.tl, refText = ESRef.tl) {
    const o = bounds.pos(ref), w = this.p.textWidth(s, h);
    const tb = ESBounds.sized(w, h, 0, h - h * 0.25, ESRef.tl), r = tb.pos(refText);
    this.p.text(s, o[0] - r[0], o[1] - r[1], h, this.app.colors.foreground);
  }
  drawCenteredText(s, bounds, h, ref = ESRef.tm) {
    const o = bounds.pos(ref), w = this.p.textWidth(s, h);
    this.p.text(s, o[0] - w / 2, o[1] - h / 4, h, this.app.colors.foreground);
  }
  drawModel(name, color, x, y, sx, sy) {
    const c = this.app.ctx;
    c.save(); c.translate(x, y); c.scale(sx, sy);
    this.p.model(this.app.models[name], color);
    c.restore();
  }
}

/* ------------------------------------------------------------------- Gauge */

class ESGauge extends ESUiElement {
  constructor(app) {
    super(app);
    this.thetaMin = Math.PI; this.thetaMax = 0; this.min = 0; this.max = 0;
    this.maxMinorTick = 2147483647; this.value = 0; this.minorStep = 1; this.majorStep = 10;
    this.minorTickWidth = 1; this.majorTickWidth = 2; this.minorTickLength = 5; this.majorTickLength = 10;
    this.outerRadius = 0; this.needleInnerRadius = 0; this.needleOuterRadius = 0; this.needleWidth = 1;
    this.needlePosition = 0; this.needleVelocity = 0; this.needleMaxVelocity = 2;
    this.needleKs = 1000; this.needleKd = 25; this.gamma = 1; this.center = [0, 0]; this.bands = [];
  }
  // Band: { color, start, end, width, radial_offset, shorten_start, shorten_end }
  setBands(bands) { this.bands = bands.map(b => ({ width: 0, radial_offset: 0, shorten_start: 0, shorten_end: 0, ...b })); }
  update(dt) {
    const v = Math.max(this.min, Math.min(this.max, this.value));
    const s = Math.pow((v - this.min) / Math.abs(this.max - this.min), this.gamma);
    const F = this.needleKs * (s - this.needlePosition) - this.needleKd * this.needleVelocity;
    this.needleVelocity = Math.min(this.needleMaxVelocity, Math.max(this.needleVelocity + F * dt, -this.needleMaxVelocity));
    this.needlePosition += this.needleVelocity * dt;
    this.needlePosition = Math.max(0, Math.min(1, this.needlePosition));
    if (!Number.isFinite(this.needlePosition)) { this.needlePosition = 0; this.needleVelocity = 0; }
  }
  angle(s) { return s * this.thetaMax + (1 - s) * this.thetaMin; }
  render() {
    const p = this.p, c = this.bounds.pos(ESRef.center);
    const ox = c[0] + this.center[0], oy = c[1] + this.center[1], R = this.outerRadius;
    const range = Math.abs(this.max - this.min), colors = this.app.colors;
    for (const band of this.bands) {
      const s0 = Math.pow((band.start - this.min) / range, this.gamma), s1 = Math.pow((band.end - this.min) / range, this.gamma);
      const a0 = this.angle(s0), a1 = this.angle(s1);
      p.begin();
      p.ring(ox, oy, R + band.radial_offset - band.width, R + band.radial_offset,
        Math.min(a0, a1) + band.shorten_end, Math.max(a0, a1) - band.shorten_start);
      p.fill(band.color);
    }
    p.begin();
    for (let i = 0; i <= range; i += this.minorStep) {
      const s = Math.pow(i / range, this.gamma), theta = this.angle(s);
      const major = i % this.majorStep === 0;
      const len = major ? this.majorTickLength : this.minorTickLength, w = major ? this.majorTickWidth : this.minorTickWidth;
      const dx = Math.cos(theta), dy = Math.sin(theta);
      if (major || i + this.minorStep <= this.maxMinorTick) p.line(dx * (R - len) + ox, dy * (R - len) + oy, dx * R + ox, dy * R + oy, w);
    }
    p.fill(colors.foreground);
    const a = this.angle(this.needlePosition), dx = Math.cos(a), dy = Math.sin(a);
    p.begin();
    p.line(dx * this.needleInnerRadius + ox, dy * this.needleInnerRadius + oy, dx * this.needleOuterRadius + ox, dy * this.needleOuterRadius + oy, this.needleWidth);
    p.fill(colors.highlight1);
  }
}

const ES_SHORTEN = Math.PI / 180;

class ESLabeledGauge extends ESUiElement {
  constructor(app, o = {}) {
    super(app);
    this.title = ''; this.precision = 2; this.unit = ''; this.spaceBeforeUnit = true;
    this.margin = 10; this.needleInnerRadiusRatio = 0.1; this.needleOuterRadiusRatio = 0.7;
    this.gauge = this.add(new ESGauge(app));
    this.gauge.center = [0, -20];
    const { gauge, ...rest } = o;
    Object.assign(this, rest);
    Object.assign(this.gauge, gauge || {});
  }
  render() {
    const colors = this.app.colors;
    this.drawFrame(this.bounds, 1, colors.foreground, colors.background);
    const bounds = this.bounds.inset(this.margin);
    const title = bounds.verticalSplit(1, 0.9), gaugeBounds = bounds.verticalSplit(0, 0.9);
    this.drawCenteredText(this.title, title.inset(10), 24);
    const text = esFixed(this.gauge.value, this.precision) + (this.spaceBeforeUnit && this.unit.length > 0 ? ' ' : '') + this.unit;
    this.drawAlignedText(text, gaugeBounds.verticalSplit(0, 2 / 8), gaugeBounds.height() / 8, ESRef.bm, ESRef.bm);
    this.gauge.bounds = gaugeBounds;
    this.gauge.outerRadius = Math.min(gaugeBounds.width(), gaugeBounds.height()) / 2;
    this.gauge.needleOuterRadius = this.gauge.outerRadius * this.needleOuterRadiusRatio;
    this.gauge.needleInnerRadius = -this.gauge.outerRadius * this.needleInnerRadiusRatio;
    super.render();
  }
}

// std::fixed << std::setprecision(n)
function esFixed(v, n) {
  if (Number.isNaN(v)) return '-nan(ind)';
  if (!Number.isFinite(v)) return v > 0 ? 'inf' : '-inf';
  return v.toFixed(n);
}

// The gauge setup every cluster repeats: a 1.2π to -0.2π sweep with a 4 px needle.
function esDial(o) {
  return { thetaMin: Math.PI * 1.2, thetaMax: -Math.PI * 0.2, needleWidth: 4, gamma: 1, needleKs: 1000, needleKd: 20, ...o };
}

/* ------------------------------------------------------------ Oscilloscope */

class ESOscilloscope extends ESUiElement {
  constructor(app, size, o = {}) {
    super(app);
    this.xMin = 0; this.xMax = 0; this.yMin = 0; this.yMax = 0; this.lineWidth = 1;
    this.drawReverse = true; this.checkMouse = true; this.drawZero = true;
    this.dynamicallyResizeX = false; this.dynamicallyResizeY = true;
    this.color = app.colors.red;
    Object.assign(this, o);
    this.bufferSize = size; this.xs = new Float64Array(size); this.ys = new Float64Array(size);
    this.writeIndex = 0; this.pointCount = 0;
    this.px = new Float64Array(size); this.py = new Float64Array(size);
  }
  update(dt) { super.update(dt); this.mouseBounds = this.bounds; }
  render() { this.renderIn(this.bounds); }
  renderIn(bounds) {
    const p = this.p, n = this.pointCount, N = this.bufferSize;
    const w = bounds.width(), h = bounds.height(), bx = bounds.x0, by = bounds.y0;
    const sx = w / (this.xMax - this.xMin), sy = h / (this.yMax - this.yMin);
    if (!Number.isFinite(sx) || !Number.isFinite(sy)) return;
    const px = this.px, py = this.py;
    for (let i = 0; i < n; i++) {
      const k = (this.writeIndex - n + i + N) % N;
      px[i] = bx + (this.xs[k] - this.xMin) * sx; py[i] = by + (this.ys[k] - this.yMin) * sy;
    }
    const colors = this.app.colors;
    if (this.drawZero) {
      const zy = by + (0 - this.yMin) * sy;
      p.begin(); p.line(bx, zy, bx + w, zy, 0.5); p.fill(esMix(colors.foreground, colors.background, 0.95));
    }
    if (n < 2) return;
    // GeometryGenerator::startPath and generatePathSegment. Each connected run
    // of quads is filled as one strip outline (left edge out, right edge back),
    // which covers the same area and is far cheaper than separate quads.
    const L = this.lx || (this.lx = new Float64Array(N)), Ly = this.ly || (this.ly = new Float64Array(N));
    const R = this.rx || (this.rx = new Float64Array(N)), Ry = this.ry || (this.ry = new Float64Array(N));
    const linked = this.linked || (this.linked = new Uint8Array(N));
    let dirX = px[1] - px[0], dirY = py[1] - py[0], len = Math.hypot(dirX, dirY);
    if (len > 0) { dirX /= len; dirY /= len; } else { dirX = 1; dirY = 0; }
    let perpX = -dirY, perpY = dirX, pdirX = dirX, pdirY = dirY;
    let width = 0.5 * this.lineWidth;
    L[0] = px[0] + perpX * width / 2; Ly[0] = py[0] + perpY * width / 2; R[0] = px[0] - perpX * width / 2; Ry[0] = py[0] - perpY * width / 2;
    let lastDetached = false;
    for (let i = 1; i < n; i++) {
      const s = i / n;
      width = this.lineWidth * Math.max(s, 0.5);
      if (s > 0.95) width += ((s - 0.95) / 0.05) * 2;
      const detached = px[i - 1] > px[i] || Math.abs(px[i] - px[i - 1]) > 100;
      if (i < n - 1) {
        let dx = px[i + 1] - px[i], dy = py[i + 1] - py[i];
        const l = Math.hypot(dx, dy);
        if (l > 0) { dx /= l; dy /= l; } else { dx = pdirX; dy = pdirY; }
        let qx = -dy - pdirY, qy = dx + pdirX;
        const ql = Math.hypot(qx, qy);
        if (ql === 0) { qx = -dy; qy = dx; } else { qx /= ql; qy /= ql; }
        perpX = qx; perpY = qy; pdirX = dx; pdirY = dy;
      }
      L[i] = px[i] + perpX * width / 2; Ly[i] = py[i] + perpY * width / 2; R[i] = px[i] - perpX * width / 2; Ry[i] = py[i] - perpY * width / 2;
      linked[i] = !((detached || lastDetached) && !this.drawReverse) ? 1 : 0;
      lastDetached = detached;
    }
    const ctx = this.app.ctx;
    ctx.beginPath();
    let a = 0;
    while (a < n - 1) {
      if (!linked[a + 1]) { a++; continue; }
      let b = a + 1;
      while (b + 1 < n && linked[b + 1]) b++;
      ctx.moveTo(L[a], Ly[a]);
      for (let i = a + 1; i <= b; i++) ctx.lineTo(L[i], Ly[i]);
      for (let i = b; i >= a; i--) ctx.lineTo(R[i], Ry[i]);
      ctx.closePath();
      a = b;
    }
    p.fill(this.color);
  }
  addDataPoint(x, y) {
    this.xs[this.writeIndex] = x; this.ys[this.writeIndex] = y;
    this.writeIndex = (this.writeIndex + 1) % this.bufferSize;
    this.pointCount = Math.min(this.bufferSize, this.pointCount + 1);
    if (this.dynamicallyResizeY) {
      if (y + Math.abs(0.1 * y) >= this.yMax) this.yMax = y + Math.abs(0.1 * y);
      else if (y - Math.abs(0.1 * y) <= this.yMin) this.yMin = y - Math.abs(0.1 * y);
    }
    if (this.dynamicallyResizeX) {
      if (x + Math.abs(0.1 * x) >= this.xMax) this.xMax = x + Math.abs(0.1 * x);
      else if (x - Math.abs(0.1 * x) <= this.xMin) this.xMin = x - Math.abs(0.1 * x);
    }
  }
  reset() { this.writeIndex = 0; this.pointCount = 0; }
}

/* ----------------------------------------------------- OscilloscopeCluster */

class ESOscilloscopeCluster extends ESUiElement {
  constructor(app) {
    super(app);
    const c = app.colors, U = ES_UNITS, cycle = 4 * Math.PI, flow10 = 10 * U.scfm, thou10 = 10 * U.thou;
    const sqrtPsi = Math.sqrt(U.psi);
    const scope = (size, o) => this.add(new ESOscilloscope(app, size, { lineWidth: 2, drawReverse: false, ...o }), this);
    this.torqueScope = scope(100, { xMin: 0, yMin: 0, yMax: 0, dynamicallyResizeX: true, color: c.orange });
    this.powerScope = scope(100, { xMin: 0, yMin: 0, yMax: 0, dynamicallyResizeX: true, color: c.pink });
    this.exhaustFlowScope = scope(1024, { xMin: 0, xMax: cycle, yMin: -flow10, yMax: flow10, color: c.orange });
    this.totalExhaustFlowScope = scope(1024, { xMin: 0, xMax: cycle, yMin: -flow10, yMax: flow10, color: c.orange });
    this.intakeFlowScope = scope(1024, { xMin: 0, xMax: cycle, yMin: -flow10, yMax: flow10, color: c.blue });
    this.audioWaveformScope = scope(44100 / 50, { xMin: 0, xMax: 44100 / 10, yMin: -1.5, yMax: 1.5, color: c.blue });
    this.intakeValveLiftScope = scope(1024, { xMin: 0, xMax: cycle, yMin: -thou10, yMax: thou10, color: c.blue });
    this.exhaustValveLiftScope = scope(1024, { xMin: 0, xMax: cycle, yMin: -thou10, yMax: thou10, color: c.orange });
    this.cylinderPressureScope = scope(1024, { xMin: 0, xMax: cycle, yMin: -sqrtPsi, yMax: sqrtPsi, color: c.orange });
    this.sparkAdvanceScope = scope(1024, { xMin: 0, xMax: 10000 * U.rpm, yMin: -30 * U.deg, yMax: 60 * U.deg, drawReverse: true, color: c.orange });
    this.cylinderMoleculesScope = scope(1024, { xMin: 0, xMax: cycle, yMin: -0.05, yMax: 0.2, lineWidth: 4, color: c.foreground });
    this.pvScope = scope(1024, { xMin: 0, xMax: 0.1 * U.L, yMin: -sqrtPsi, yMax: sqrtPsi, drawReverse: true, dynamicallyResizeX: true, color: c.orange });
    // Never given bounds by render(), as in the original.
    this.cylinderPressureScope.visible = false; this.sparkAdvanceScope.visible = false;
    this.focus = [this.totalExhaustFlowScope];
    this.torque = 0; this.power = 0; this.updatePeriod = 0.25; this.updateTimer = 0;
  }
  signal(e) {
    if (e === this.audioWaveformScope) this.focus = [this.audioWaveformScope];
    else if (e === this.powerScope || e === this.torqueScope) this.focus = [this.torqueScope, this.powerScope];
    else if (e === this.totalExhaustFlowScope) this.focus = [this.totalExhaustFlowScope];
    else if (e === this.intakeValveLiftScope || e === this.exhaustValveLiftScope) this.focus = [this.intakeValveLiftScope, this.exhaustValveLiftScope];
    else if (e === this.pvScope) this.focus = [this.pvScope];
    else if (e === this.intakeFlowScope || e === this.exhaustFlowScope || e === this.cylinderMoleculesScope) this.focus = [this.intakeFlowScope, this.exhaustFlowScope, this.cylinderMoleculesScope];
    else if (e === this.cylinderPressureScope) this.focus = [this.cylinderPressureScope];
    else if (e === this.sparkAdvanceScope) this.focus = [this.sparkAdvanceScope];
  }
  update(dt) {
    const sim = this.app.simulator;
    if (sim) {
      const torque = sim.filteredDynoTorque() / ES_UNITS.lbft, power = sim.dynoPower() / 745.699872;
      this.torque = this.torque * 0.95 + 0.05 * torque;
      this.power = this.power * 0.95 + 0.05 * power;
      const e = sim.engine;
      if (this.updateTimer <= 0 && sim.dyno.enabled) {
        this.updateTimer = this.updatePeriod;
        this.torqueScope.addDataPoint(e.rpm(), this.torque);
        this.powerScope.addDataPoint(e.rpm(), this.power);
      }
      this.sparkAdvanceScope.addDataPoint(-e.omega, sim.ignitionModule.timingAdvance());
    }
    this.updateTimer -= dt;
    super.update(dt);
  }
  render() {
    const B = this.bounds, g = (x, y, w, h) => esGrid(B, 3, 4, x, y, w, h);
    const hpTorque = g(0, 3);
    this.renderScope(this.torqueScope, hpTorque, 'Torque/Power');
    this.renderScope(this.powerScope, hpTorque, '', true);
    const valveLift = g(2, 2);
    this.renderScope(this.intakeValveLiftScope, valveLift, 'Valve Lift');
    this.renderScope(this.exhaustValveLiftScope, valveLift, '', true);
    const flow = g(2, 3);
    this.renderScope(this.intakeFlowScope, flow, 'Flow');
    this.renderScope(this.exhaustFlowScope, flow, '', true);
    this.renderScope(this.cylinderMoleculesScope, flow, '', true);
    this.renderScope(this.audioWaveformScope, g(0, 2), 'Waveform');
    this.renderScope(this.pvScope, g(1, 3), 'pressure-volume');
    this.renderScope(this.totalExhaustFlowScope, g(1, 2), 'Total Exhaust Flow');
    const focus = g(0, 0, 3, 2), colors = this.app.colors;
    const focusTitle = new ESBounds(focus.x0, focus.y1 - 39, focus.x1, focus.y1);
    const focusBody = new ESBounds(focus.x0, focus.y0, focus.x1, focus.y1 - 39);
    this.drawFrame(focusTitle, 1, colors.foreground, colors.background);
    this.drawFrame(focusBody, 1, colors.foreground, colors.background);
    for (const s of this.focus) s.renderIn(focusBody);
    super.render();
  }
  renderScope(osc, bounds, title, overlay = false) {
    if (!overlay) this.drawFrame(bounds, 1, this.app.colors.foreground, this.app.colors.background);
    if (osc === this.focus[0]) this.drawText(title, esGrid(this.bounds, 3, 4, 0, 0, 3, 2).inset(20), 24, ESRef.tl);
    osc.bounds = bounds;
  }
  // OscilloscopeCluster::sample, called after every simulation step.
  sample() {
    const sim = this.app.simulator, e = sim.engine, ch = e.chambers[0], dt = sim.timestep();
    if (sim.currentIteration % 2 === 0) {
      let cycleAngle = e.cycleAngle();
      if (!e.isSpinningCw()) cycleAngle = 4 * Math.PI - cycleAngle;
      const vt = ch.head.valvetrain, idx = ch.cylinder.index;
      this.totalExhaustFlowScope.addDataPoint(cycleAngle, sim.totalExhaustFlow() / dt);
      this.cylinderPressureScope.addDataPoint(e.cycleAngle(Math.PI), Math.sqrt(Math.max(0, ch.system.pressure() + ch.system.dynamicPressure(-1, 0))));
      this.exhaustFlowScope.addDataPoint(cycleAngle, ch.lastTimestepExhaustFlow / dt);
      this.intakeFlowScope.addDataPoint(cycleAngle, ch.lastTimestepIntakeFlow / dt);
      this.cylinderMoleculesScope.addDataPoint(cycleAngle, ch.system.n);
      this.exhaustValveLiftScope.addDataPoint(cycleAngle, vt.exhaustValveLift(idx));
      this.intakeValveLiftScope.addDataPoint(cycleAngle, vt.intakeValveLift(idx));
      this.pvScope.addDataPoint(ch.volume(), Math.sqrt(Math.max(0, ch.system.pressure())));
    }
    const i = this.intakeFlowScope, x = this.exhaustFlowScope, t = this.torqueScope, pw = this.powerScope;
    x.yMin = i.yMin = Math.min(i.yMin, x.yMin); x.yMax = i.yMax = Math.max(i.yMax, x.yMax);
    t.yMin = pw.yMin = Math.min(t.yMin, pw.yMin); t.yMax = pw.yMax = Math.max(t.yMax, pw.yMax);
    pw.xMax = t.xMax = Math.max(pw.xMax, e.rpm());
  }
  resetScopes() { for (const c of this.children) c.reset(); this.torque = 0; this.power = 0; }
}

/* ------------------------------------------------------ RightGaugeCluster */

class ESFiringOrderDisplay extends ESUiElement {
  constructor(app) { super(app); this.cylinderLit = []; }
  update(dt) {
    super.update(dt);
    const e = this.app.simulator && this.app.simulator.engine;
    if (!e) return;
    if (this.cylinderLit.length !== e.chambers.length) this.cylinderLit = e.chambers.map(() => 0);
    e.chambers.forEach((ch, i) => {
      if (ch.popLitLastFrame() || ch.lit) this.cylinderLit[i] = 0.05 + 0.95 * this.cylinderLit[i];
      else this.cylinderLit[i] *= dt / (dt + 0.01);
    });
  }
  render() {
    const colors = this.app.colors, p = this.p;
    this.drawFrame(this.bounds, 1, colors.foreground, colors.background);
    const title = this.bounds.verticalSplit(1, 0.9), body = this.bounds.verticalSplit(0, 0.9);
    this.drawCenteredText('Ignition', title.inset(20), 24);
    const e = this.app.simulator && this.app.simulator.engine;
    if (!e) return;
    const banks = e.banks.length, bg = colors.background;
    const hot = esMix(bg, colors.foreground, 1), fixed = esMix(bg, colors.foreground, 0.02), cold = esMix(bg, colors.foreground, 0.001);
    const ordered = e.banks.slice().sort((a, b) => a.angle - b.angle);
    e.cylinders.forEach((cyl, i) => {
      const bank = cyl.bank, bankIndex = ordered.indexOf(bank);
      const b = esGrid(body, banks, 1, banks - bankIndex - 1, 0);
      const n = bank.cylinders.length;
      const bc = esGrid(b, 1, n, 0, n - cyl.index - 1).inset(5);
      const c = bc.pos(ESRef.center), r = Math.min(bc.height() / 2, bc.width() / 2);
      p.begin(); p.ring(c[0], c[1], r * 0.8, r * 0.85, 0, ES_TAU); p.fill(fixed);
      p.begin(); p.circle(c[0], c[1], r * 0.75); p.fill(esMix(cold, hot, this.cylinderLit[i] || 0));
    });
  }
}

class ESThrottleDisplay extends ESUiElement {
  render() {
    super.render();
    const colors = this.app.colors;
    this.drawFrame(this.bounds, 1, colors.foreground, colors.background);
    const bounds = this.bounds.inset(10);
    this.drawCenteredText('THROTTLE', bounds.verticalSplit(1, 0.9).inset(10), 24);
    this.renderThrottle(bounds.verticalSplit(0.05, 0.9));
    this.renderSpeedControl(bounds.verticalSplit(0, 0.05));
  }
  renderThrottle(bounds) {
    const p = this.p, colors = this.app.colors, e = this.app.simulator && this.app.simulator.engine;
    const size = Math.min(bounds.width(), bounds.height()), o = bounds.pos(ESRef.center);
    const carbBore = size * 0.4, carbHeight = size * 0.5, plateWidth = carbBore * 0.8, lw = size * 0.01;
    p.begin();
    for (const sx of [1, -1]) {
      const x = o[0] + sx * carbBore / 2, y0 = o[1] - carbHeight / 2, y1 = o[1] + carbHeight / 2;
      p.line(x, y0, x, y1, lw); p.circle(x, y1, lw / 2); p.circle(x, y0, lw / 2);
    }
    const angle = e ? e.throttlePlateAngle() : 0, c = Math.cos(angle), s = Math.sin(angle);
    const x0 = o[0] - c * plateWidth / 2, y0 = o[1] - s * plateWidth / 2, x1 = o[0] + c * plateWidth / 2, y1 = o[1] + s * plateWidth / 2;
    p.line(x0, y0, x1, y1, lw); p.circle(x0, y0, lw / 2); p.circle(x1, y1, lw / 2);
    p.fill(colors.foreground);
    p.begin(); p.circle(o[0], o[1], 2 * size * 0.01); p.fill(colors.background);
    p.begin(); p.circle(o[0], o[1], size * 0.01); p.fill(colors.foreground);
  }
  renderSpeedControl(bounds) {
    const colors = this.app.colors, e = this.app.simulator && this.app.simulator.engine;
    const b = esGrid(bounds, 3, 1, 1, 0), lm = b.pos(ESRef.lm), s = e ? e.speedControl : 0;
    const bar = ESBounds.sized(b.width(), 2, lm[0], lm[1], ESRef.lm);
    const speedControlBar = ESBounds.sized(b.width() * s, 2, lm[0], lm[1], ESRef.lm);
    this.drawFrame(bar, 1, esMix(colors.background, colors.foreground, 0.001), esMix(colors.background, colors.red, 0.01));
    this.drawBox(speedControlBar, colors.red);
  }
}

class ESAfrCluster extends ESUiElement {
  constructor(app) {
    super(app);
    this.intakeAfrGauge = this.add(new ESLabeledGauge(app, { title: 'IN. AFR', precision: 1, gauge: esDial({ min: 0, max: 50, minorStep: 1, majorStep: 5, maxMinorTick: 7000 }) }));
    this.exhaustAfrGauge = this.add(new ESLabeledGauge(app, { title: 'EX. O2', precision: 1, gauge: esDial({ min: 0, max: 100, minorStep: 5, majorStep: 10, maxMinorTick: 200 }) }));
  }
  render() {
    const e = this.app.simulator && this.app.simulator.engine;
    this.intakeAfrGauge.bounds = this.bounds.verticalSplit(0.5, 1);
    this.intakeAfrGauge.gauge.value = e ? e.intakeAfr() : 0;
    this.exhaustAfrGauge.bounds = this.bounds.verticalSplit(0, 0.5);
    this.exhaustAfrGauge.gauge.value = e ? e.exhaustO2() * 100 : 0;
    super.render();
  }
}

class ESFuelCluster extends ESUiElement {
  render() {
    const sim = this.app.simulator, e = sim && sim.engine;
    const bounds = this.bounds.inset(10), body = bounds.verticalSplit(0, 0.9);
    this.drawCenteredText('FUEL', bounds.verticalSplit(1, 0.9).inset(10), 24);
    const g = (y, h = 1) => esGrid(body, 1, 10, 0, y, 1, h);
    const consumed = e ? e.totalVolumeFuelConsumed() : 0, gallons = consumed / 0.003785411784;
    this.drawText(esFixed(consumed / ES_UNITS.L, 3) + ' L', g(1, 2), 32, ESRef.lm);
    this.drawText(esFixed(gallons, 3) + ' gal', g(3), 16, ESRef.lm);
    this.drawText('$' + esFixed(4.761 * gallons, 2) + ' USD', g(4), 16, ESRef.lm);
    const travelled = sim ? sim.vehicle.travelledDistance : 0;
    this.drawText(esFixed(travelled / 1609.344 / gallons, 2) + ' MPG', g(6), 16, ESRef.lm);
    const lp100km = travelled !== 0 ? (consumed / ES_UNITS.L) / (travelled / 1000 / 100) : 0;
    this.drawText(esFixed(lp100km > 100 ? 100 : lp100km, 2) + ' L/100 KM', g(7), 12, ESRef.lm);
    super.render();
  }
}

class ESRightGaugeCluster extends ESUiElement {
  constructor(app) {
    super(app);
    const c = app.colors, s = ES_SHORTEN;
    this.tachometer = this.add(new ESLabeledGauge(app, { title: 'ENGINE SPEED', unit: 'rpm', precision: 0, gauge: esDial({ min: 0, max: 7000, minorStep: 100, majorStep: 1000 }) }));
    this.speedometer = this.add(new ESLabeledGauge(app, { title: 'VEHICLE SPEED', unit: 'mph', precision: 0, gauge: esDial({ min: 0, max: 200, minorStep: 5, majorStep: 10, maxMinorTick: 200 }) }));
    this.manifoldVacuumGauge = this.add(new ESLabeledGauge(app, { title: 'MANIFOLD PRESSURE', unit: 'inHg', precision: 0, gauge: esDial({ min: -30, max: 5, minorStep: 1, majorStep: 5, maxMinorTick: 200, needleKd: 50 }) }));
    this.intakeCfmGauge = this.add(new ESLabeledGauge(app, { title: 'AIR SCFM', precision: 1, gauge: esDial({ min: 0, max: 1200, minorStep: 20, majorStep: 100, maxMinorTick: 1200, needleKd: 50 }) }));
    this.volumetricEffGauge = this.add(new ESLabeledGauge(app, { title: 'VOLUMETRIC EFF.', unit: '%', spaceBeforeUnit: false, precision: 1, gauge: esDial({ min: 0, max: 120, minorStep: 5, majorStep: 10, maxMinorTick: 200, needleKd: 50 }) }));
    this.combustionChamberStatus = this.add(new ESFiringOrderDisplay(app));
    this.throttleDisplay = this.add(new ESThrottleDisplay(app));
    this.afrCluster = this.add(new ESAfrCluster(app));
    this.fuelCluster = this.add(new ESFuelCluster(app));
    this.manifoldVacuumGauge.gauge.setBands([
      { color: c.red, start: -5, end: -1, width: 3, radial_offset: 6, shorten_start: s, shorten_end: s },
      { color: c.foreground, start: -1, end: 1, width: 3, radial_offset: 6, shorten_start: s, shorten_end: s },
      { color: c.orange, start: -10, end: -5, width: 3, radial_offset: 6, shorten_start: s, shorten_end: s },
      { color: c.blue, start: -22, end: -10, width: 3, radial_offset: 6, shorten_start: s, shorten_end: s },
      { color: c.foreground, start: -30, end: -22, width: 3, radial_offset: 6, shorten_start: s, shorten_end: s },
    ]);
    this.volumetricEffGauge.gauge.setBands([
      { color: c.blue, start: 30, end: 80, width: 3, radial_offset: 6, shorten_start: 0, shorten_end: s },
      { color: c.green, start: 80, end: 100, width: 3, radial_offset: 6, shorten_start: s, shorten_end: s },
      { color: c.red, start: 100, end: 120, width: 3, radial_offset: 6, shorten_start: s, shorten_end: -s },
    ]);
  }
  render() {
    this.drawFrame(this.bounds, 1, this.app.colors.foreground, this.app.colors.background);
    this.renderTachSpeedCluster(this.bounds.verticalSplit(0.5, 1));
    this.renderFuelAirCluster(this.bounds.verticalSplit(0, 0.5));
    super.render();
  }
  renderTachSpeedCluster(bounds) {
    const sim = this.app.simulator, e = sim && sim.engine, c = this.app.colors, s = ES_SHORTEN;
    const left = bounds.horizontalSplit(0, 0.5), right = bounds.horizontalSplit(0.5, 1);
    this.tachometer.bounds = left.verticalSplit(0.5, 1);
    this.tachometer.gauge.value = e ? Math.abs(e.rpm()) : 0;
    const redlineRpm = e ? e.redline / ES_UNITS.rpm : 0;
    const maxRpm = Math.ceil(redlineRpm * 1.25 / 1000) * 1000;
    const redline = Math.ceil(redlineRpm / 500) * 500, warning = Math.floor(redlineRpm * 0.9 / 500) * 500;
    this.tachometer.gauge.max = maxRpm;
    this.tachometer.gauge.setBands([
      { color: c.foreground, start: 400, end: 1000, width: 3, radial_offset: 6 },
      { color: c.orange, start: warning, end: redline, width: 3, radial_offset: 6, shorten_start: -s, shorten_end: s },
      { color: c.red, start: redline, end: maxRpm, width: 3, radial_offset: 6, shorten_start: s, shorten_end: -s },
    ]);
    this.speedometer.bounds = left.verticalSplit(0, 0.5);
    this.speedometer.gauge.value = sim ? Math.abs(sim.vehicleSpeed()) / ES_UNITS.mph : 0;
    this.combustionChamberStatus.bounds = right;
  }
  renderFuelAirCluster(bounds) {
    const sim = this.app.simulator, e = sim && sim.engine;
    const left = bounds.horizontalSplit(0, 0.5), right = bounds.horizontalSplit(0.5, 1);
    this.throttleDisplay.bounds = left.verticalSplit(0.5, 1);
    const fuelSection = left.verticalSplit(0, 0.5);
    this.afrCluster.bounds = fuelSection.horizontalSplit(0, 0.5);
    this.fuelCluster.bounds = fuelSection.horizontalSplit(0.5, 1);
    this.manifoldVacuumGauge.bounds = esGrid(right, 1, 3, 0, 0);
    const vacuum = Math.min((e ? e.manifoldPressure() : ES_ATM) - ES_ATM, 0) / ES_UNITS.inHg;
    this.manifoldVacuumGauge.gauge.value = vacuum > -0.5 ? 0 : vacuum;
    const rpm = Math.max(e ? e.rpm() : 0, 0);
    const theoreticalPerRev = e ? 0.5 * (ES_ATM * e.displacement) / (ES_R * ES_T25) : 0;
    const theoreticalPerSecond = theoreticalPerRev * rpm / 60, actual = e ? e.intakeFlowRate() : 0;
    const ve = Math.abs(theoreticalPerSecond) < 1e-3 ? 0 : actual / theoreticalPerSecond;
    this.intakeCfmGauge.bounds = esGrid(right, 1, 3, 0, 1);
    this.intakeCfmGauge.gauge.value = actual / ES_UNITS.scfm;
    this.volumetricEffGauge.bounds = esGrid(right, 1, 3, 0, 2);
    this.volumetricEffGauge.gauge.value = 100 * ve;
  }
}

/* ------------------------------------------------------ PerformanceCluster */

class ESPerformanceCluster extends ESUiElement {
  constructor(app) {
    super(app);
    const c = app.colors, s = ES_SHORTEN, band = (color, start, end, ss, se, width = 3, radial_offset = 6) => ({ color, start, end, width, radial_offset, shorten_start: ss, shorten_end: se });
    this.timePerTimestepGauge = this.add(new ESLabeledGauge(app, { title: 'RT/dT', precision: 1, gauge: esDial({ min: 0, max: 200, minorStep: 5, majorStep: 10, maxMinorTick: 1000000 }) }));
    this.timePerTimestepGauge.gauge.setBands([band(c.orange, 50, 100, s, s), band(c.red, 100, 200, s, -s), band(c.blue, 0, 50, -s, s)]);
    this.fpsGauge = this.add(new ESLabeledGauge(app, { title: 'FPS', precision: 1, gauge: esDial({ min: 0, max: 120, minorStep: 1, majorStep: 15, maxMinorTick: 60, gamma: 0.6 }) }));
    this.fpsGauge.gauge.setBands([band(c.green, 58, 62, s, s), band(c.blue, 62, 120, s, -s), band(c.orange, 30, 58, s, s), band(c.red, 0, 30, -s, s), band(c.foreground, 60, 120, -s, s, 3, 0)]);
    this.simSpeedGauge = this.add(new ESLabeledGauge(app, { title: '1 / SPEED', spaceBeforeUnit: false, precision: 1, gauge: esDial({ min: 0, max: 1000, minorStep: 50, majorStep: 100, maxMinorTick: 1000 }) }));
    this.audioLagGauge = this.add(new ESLabeledGauge(app, { title: 'LATENCY', spaceBeforeUnit: false, precision: 1, gauge: esDial({ min: 50, max: 150, minorStep: 5, majorStep: 10, maxMinorTick: 1000, thetaMin: Math.PI * 0.8, thetaMax: Math.PI * 0.2 }) }));
    this.audioLagGauge.gauge.setBands([band(c.foreground, 90, 110, s, s), band(c.red, 50, 90, -s, s), band(c.orange, 110, 150, s, -s)]);
    this.inputSamplesGauge = this.add(new ESLabeledGauge(app, { title: 'IN. BUFFER', spaceBeforeUnit: false, precision: 1, gauge: esDial({ min: 0, max: 200, minorStep: 5, majorStep: 10, maxMinorTick: 1000 }) }));
    this.inputSamplesGauge.gauge.setBands([band(c.foreground, 90, 110, s, s), band(c.red, 0, 10, -s, s), band(c.orange, 150, 200, s, -s)]);
    this.simulationFrequencyGauge = this.add(new ESLabeledGauge(app, { title: 'FREQUENCY', unit: 'hz', precision: 0, gauge: esDial({ min: 1000, max: 51000, minorStep: 1000, majorStep: 10000, maxMinorTick: 50000, gamma: 0.9 }) }));
    this.simulationFrequencyGauge.gauge.setBands([band(c.foreground, 11025, 44100, s, s)]);
    this.timePerTimestep = 0; this.filteredSimulationFrequency = 0; this.inputBufferUsage = 0; this.audioLatency = 0;
  }
  update(dt) {
    super.update(dt);
    const sim = this.app.simulator;
    this.filteredSimulationFrequency = 0.9 * this.filteredSimulationFrequency + 0.1 * sim.simulationFrequency * sim.simulationSpeed;
  }
  render() {
    const B = this.bounds, g = (x, y) => esGrid(B, 3, 2, x, y), sim = this.app.simulator;
    this.timePerTimestepGauge.bounds = g(1, 0);
    this.timePerTimestepGauge.gauge.value = this.timePerTimestep / (1 / this.filteredSimulationFrequency) * 100;
    this.fpsGauge.bounds = g(0, 0);
    this.fpsGauge.gauge.value = this.app.averageFramerate;
    this.simSpeedGauge.bounds = g(2, 0);
    this.simSpeedGauge.gauge.value = 1 / sim.simulationSpeed;
    this.audioLagGauge.bounds = g(0, 1);
    this.audioLagGauge.gauge.value = this.audioLatency * 100;
    this.inputSamplesGauge.bounds = g(1, 1);
    this.inputSamplesGauge.gauge.value = this.inputBufferUsage * 100;
    this.simulationFrequencyGauge.bounds = g(2, 1);
    this.simulationFrequencyGauge.gauge.value = sim.simulationFrequency;
    super.render();
  }
  addTimePerTimestepSample(v) { this.timePerTimestep = 0.95 * this.timePerTimestep + 0.05 * v; }
  addAudioLatencySample(v) { this.audioLatency = 0.95 * this.audioLatency + 0.05 * v; }
  addInputBufferUsageSample(v) { this.inputBufferUsage = 0.95 * this.inputBufferUsage + 0.05 * v; }
}

/* --------------------------------------------------- LoadSimulationCluster */

class ESLoadSimulationCluster extends ESUiElement {
  constructor(app) {
    super(app);
    const dial = o => esDial({ thetaMin: Math.PI * 0.8, thetaMax: Math.PI * 0.2, needleKd: 5, maxMinorTick: 2147483647, ...o });
    this.dynoSpeedGauge = this.add(new ESLabeledGauge(app, { title: 'DYNO. SPEED', unit: 'RPM', precision: 0, gauge: dial({ min: 0, max: 100, minorStep: 500, majorStep: 1000 }) }));
    this.torqueGauge = this.add(new ESLabeledGauge(app, { title: 'TORQUE', unit: 'lb-ft', precision: 0, gauge: dial({ min: 0, max: 1000, minorStep: 50, majorStep: 100 }) }));
    this.hpGauge = this.add(new ESLabeledGauge(app, { title: 'POWER', unit: 'hp', precision: 0, gauge: dial({ min: 0, max: 1000, minorStep: 50, majorStep: 100 }) }));
    this.clutchPressureGauge = this.add(new ESLabeledGauge(app, { title: 'CLUTCH', spaceBeforeUnit: false, precision: 0, gauge: dial({ min: 0, max: 100, minorStep: 10, majorStep: 50, maxMinorTick: 200, needleMaxVelocity: 10 }) }));
    this.resetReadings();
  }
  resetReadings() {
    this.filteredHorsepower = 0; this.filteredTorque = 0; this.peakHorsepower = 0; this.peakTorque = 0;
    this.peakHorsepowerRpm = 0; this.peakTorqueRpm = 0; this.systemStatusLights = [0, 0, 0, 0];
  }
  update(dt) {
    super.update(dt);
    const sim = this.app.simulator, app = this.app;
    const statuses = [
      sim.ignitionModule.enabled ? 1 : 0.01, sim.starterMotor.enabled ? 1 : 0.01, sim.dyno.enabled ? 1 : 0.01,
      sim.dyno.hold ? (sim.dyno.enabled ? 1 : 0.25) : 0.01,
    ];
    const alpha = dt / (dt + 0.08);
    for (let i = 0; i < 4; i++) this.systemStatusLights[i] = (1 - alpha) * this.systemStatusLights[i] + alpha * statuses[i];
    if (app.keyPressed('KeyI')) {
      app.infoCluster.setLogMessage(`${esFixed(this.peakHorsepower, 0)}hp @ ${esFixed(this.peakHorsepowerRpm, 0)}rpm | ${esFixed(this.peakTorque, 0)}lb-ft @ ${esFixed(this.peakTorqueRpm, 0)}rpm`);
    }
    const a = dt / (dt + 0.1);
    this.filteredTorque = (1 - a) * this.filteredTorque + a * sim.filteredDynoTorque() / ES_UNITS.lbft;
    this.filteredHorsepower = (1 - a) * this.filteredHorsepower + a * sim.dynoPower() / 745.699872;
    if (this.filteredTorque > this.peakTorque) { this.peakTorque = this.filteredTorque; this.peakTorqueRpm = sim.engine.rpm(); }
    if (this.filteredHorsepower > this.peakHorsepower) { this.peakHorsepower = this.filteredHorsepower; this.peakHorsepowerRpm = sim.engine.rpm(); }
  }
  render() {
    const B = this.bounds, g = (x, y) => esGrid(B, 3, 2, x, y), sim = this.app.simulator, colors = this.app.colors;
    this.drawCurrentGear(g(2, 0));
    this.clutchPressureGauge.bounds = g(1, 0);
    this.clutchPressureGauge.gauge.value = sim.transmission.clutchPressure * 100;
    this.drawSystemStatus(g(0, 0));
    this.dynoSpeedGauge.gauge.value = Math.abs(sim.dyno.rotationSpeed) / ES_UNITS.rpm;
    this.dynoSpeedGauge.bounds = g(0, 1);
    const redline = sim.engine.redline / ES_UNITS.rpm, maxRpm = Math.floor(redline / 500) * 500;
    this.dynoSpeedGauge.gauge.max = maxRpm;
    this.dynoSpeedGauge.gauge.setBands([{ color: colors.red, start: redline, end: maxRpm, width: 3, radial_offset: 6, shorten_start: ES_SHORTEN, shorten_end: -ES_SHORTEN }]);
    this.torqueGauge.gauge.value = sim.dyno.enabled ? this.filteredTorque : this.peakTorque;
    this.torqueGauge.bounds = g(1, 1);
    this.hpGauge.gauge.value = sim.dyno.enabled ? this.filteredHorsepower : this.peakHorsepower;
    this.hpGauge.bounds = g(2, 1);
    super.render();
  }
  drawCurrentGear(bounds) {
    const inset = bounds.inset(10), colors = this.app.colors;
    this.drawFrame(bounds, 1, colors.foreground, colors.background);
    this.drawCenteredText('Gear', inset.verticalSplit(0.9, 1).inset(10), 24);
    const gear = this.app.simulator.transmission.gear;
    this.drawCenteredText(gear !== -1 ? String(gear + 1) : 'N', inset.verticalSplit(0, 0.9), 64, ESRef.center);
  }
  drawSystemStatus(bounds) {
    const colors = this.app.colors;
    const left = bounds.horizontalSplit(0, 0.6), right = bounds.horizontalSplit(0.6, 1);
    this.drawFrame(bounds, 1, colors.foreground, colors.background);
    ['Ignition', 'Starter', 'Dyno.', 'Hold'].forEach((label, i) => this.drawText(label, esGrid(left, 1, 4, 0, i).inset(10), 20, ESRef.lm));
    for (let i = 0; i < 4; i++) {
      const raw = esGrid(right, 1, 4, 0, i), width = Math.max(raw.width(), raw.height()), c = raw.pos(ESRef.center);
      this.drawFrame(ESBounds.sized(width - 20, 5, c[0], c[1], ESRef.center), 1,
        esMix(colors.background, colors.foreground, 0.001), esMix(colors.background, colors.red, this.systemStatusLights[i]));
    }
  }
}

/* ------------------------------------------------------------ MixerCluster */

class ESMixerCluster extends ESUiElement {
  constructor(app) {
    super(app);
    const pct = title => this.add(new ESLabeledGauge(app, { title, precision: 1, gauge: esDial({ min: 0, max: 100, minorStep: 5, majorStep: 10, maxMinorTick: 1000000 }) }));
    this.volumeGauge = pct('Vol.');
    this.levelerGauge = pct('Lvl.');
    this.convolutionGauge = pct('Conv.');
    this.highFreqFilterGauge = this.add(new ESLabeledGauge(app, { title: '+HF', precision: 2, gauge: esDial({ min: 0, max: 10, minorStep: 1, majorStep: 2, maxMinorTick: 10 }) }));
    this.noise0Gauge = pct('~ LF');
    this.noise1Gauge = pct('~ HF');
  }
  render() {
    const B = this.bounds, g = x => esGrid(B, 6, 1, x, 0), p = this.app.audioParams;
    this.volumeGauge.bounds = g(0); this.volumeGauge.gauge.value = p.volume * 100;
    this.convolutionGauge.bounds = g(1); this.convolutionGauge.gauge.value = p.convolution * 100;
    this.highFreqFilterGauge.bounds = g(2); this.highFreqFilterGauge.gauge.value = p.dF_F_mix * 1000;
    this.noise0Gauge.bounds = g(3); this.noise0Gauge.gauge.value = p.airNoise * 100;
    this.noise1Gauge.bounds = g(4); this.noise1Gauge.gauge.value = p.inputSampleNoise * 100;
    this.levelerGauge.bounds = g(5);
    this.levelerGauge.gauge.value = 100 * ((this.app.levelerGain - p.levelerMinGain) / p.levelerMaxGain);
    super.render();
  }
}

/* ------------------------------------------------------------- InfoCluster */

class ESInfoCluster extends ESUiElement {
  constructor(app) { super(app); this.logMessage = 'Started'; this.checkMouse = true; }
  setLogMessage(s) { this.logMessage = s; }
  update(dt) { super.update(dt); this.mouseBounds = esGrid(this.bounds, 6, 4, 0, 2, 6, 1); }
  // Web port: clicking the engine name loads the next engine.
  onMouseClick() { this.app.nextEngine(1); }
  render() {
    const B = this.bounds, colors = this.app.colors, e = this.app.simulator && this.app.simulator.engine;
    const logo = esGrid(B, 6, 4, 0, 0, 1, 2);
    this.drawFrame(logo, 1, colors.foreground, colors.background);
    const c = logo.pos(ESRef.center);
    this.drawModel('Logo', colors.foreground, c[0], c[1], logo.height() * 0.75, logo.height() * 0.75);
    const title = esGrid(B, 6, 4, 1, 0, 5, 2);
    this.drawFrame(title, 1, colors.foreground, colors.background);
    this.drawAlignedText('ENGINE SIMULATOR', esGrid(title, 1, 3, 0, 0).inset(10).moved(0, -21), 42, ESRef.bl, ESRef.bl);
    this.drawAlignedText('YOUTUBE/ANGETHEGREAT', esGrid(title, 1, 3, 0, 1).inset(10).moved(0, 5), 24, ESRef.tl, ESRef.tl);
    this.drawAlignedText('BUILD: v0.1.12a // WEB PORT', esGrid(title, 1, 3, 0, 2).inset(10).moved(0, 10), 16, ESRef.tl, ESRef.tl);
    const info = esGrid(B, 6, 4, 0, 2, 6, 1);
    this.drawFrame(info, 1, colors.foreground, colors.background);
    this.drawAlignedText(e ? e.name : '<NO ENGINE>', info.inset(10), 24, ESRef.lm, ESRef.lm);
    let d = 'N/A';
    if (e) {
      d = (e.displacement < ES_UNITS.L ? esFixed(e.displacement / ES_UNITS.cc, 0) + ' cc -- ' : esFixed(e.displacement / ES_UNITS.L, 1) + ' L -- ')
        + esFixed(e.displacement / (ES_UNITS.inch ** 3), 0) + ' CI';
    }
    this.drawAlignedText(d, info.inset(10), 24, ESRef.rm, ESRef.rm);
    const messages = esGrid(B, 6, 4, 0, 3, 6, 1);
    this.drawFrame(messages, 1, colors.foreground, colors.background);
    this.drawAlignedText(this.logMessage, messages.inset(10), 24, ESRef.lm, ESRef.lm);
  }
}

/* -------------------------------------------------------------- EngineView */

class ESEngineView extends ESUiElement {
  constructor(app) {
    super(app);
    this.pan = [0, -6 * ES_UNITS.inch]; this.dragStart = [0, 0];
    this.checkMouse = true; this.zoom = 1; this.drawFrameOn = true;
  }
  update() { this.mouseBounds = this.bounds; }
  render() { if (this.drawFrameOn) this.drawFrame(this.bounds, 1, this.app.colors.foreground, this.app.colors.background, false); }
  onMouseDown() { this.dragStart = this.pan.slice(); }
  onDrag(mouse0, mouse) {
    const f = this.app.pixelsToUnits(1);
    this.pan = [this.dragStart[0] + (mouse[0] - mouse0[0]) * f, this.dragStart[1] + (mouse[1] - mouse0[1]) * f];
  }
  onMouseScroll(scroll) {
    const f = Math.pow(2, scroll / 500), prev = this.camera();
    this.zoom *= f;
    const next = this.camera(), dx = next[0] - prev[0], dy = next[1] - prev[1];
    this.pan[0] += dx * this.zoom; this.pan[1] += dy * this.zoom;
    this.dragStart[0] += dx * this.zoom; this.dragStart[1] += dy * this.zoom;
  }
  camera() { return [-this.pan[0] / this.zoom, -this.pan[1] / this.zoom]; }
}

/* ------------------------------------------------------- Simulation objects */

// GeometryGenerator::generateCam: the outline traced by the roller follower.
function esCamOutline(camshaft, padding, rollerRadius) {
  const base = camshaft.baseRadius + padding, maxEdge = 50 * ES_UNITS.thou;
  let angle = Math.asin(maxEdge / (2 * base)) * 2;
  angle = Math.min(angle, Math.PI - Math.PI * 0.95);
  const n = Math.max(3, Math.ceil(ES_TAU / angle)), step = ES_TAU / n, rollerBase = base + rollerRadius;
  const xs = new Float64Array(n), ys = new Float64Array(n);
  let lastX = 0, lastY = 0;
  for (let i = 0; i < n; i++) {
    const a = step * i, x = Math.cos(a + Math.PI / 2), y = Math.sin(a + Math.PI / 2);
    const r = rollerBase + camshaft.lobeProfile.sample(a - Math.PI);
    const rx = x * r, ry = y * r;
    let dx = -1, dy = 0;
    if (i !== 0) { dx = rx - lastX; dy = ry - lastY; const m = Math.hypot(dx, dy); dx /= m; dy /= m; }
    lastX = rx; lastY = ry;
    xs[i] = rx - dy * rollerRadius; ys[i] = ry + dx * rollerRadius;
  }
  const path = new Path2D();
  for (let i = 0; i < n; i++) {
    const a = (i - 1 + n) % n, b = (i + 1) % n, px = (xs[a] + xs[b]) / 2, py = (ys[a] + ys[b]) / 2;
    if (i === 0) path.moveTo(px, py); else path.lineTo(px, py);
  }
  path.closePath();
  return path;
}

class ESScene {
  constructor(app) { this.app = app; this.cams = new Map(); }
  tint(col, layers) {
    let r = col;
    for (let i = 0; i < layers; i++) r = r.map((v, k) => k === 3 ? v : v * 0.3 + this.app.colors.background[k] * 0.7);
    return r;
  }
  foremost(bank, layer) {
    let best = null;
    for (const c of bank.cylinders) if (c.journal >= layer && (best === null || c.journal < best.journal)) best = c;
    return best;
  }
  camPath(cam, padding, roller) {
    const key = cam;
    let entry = this.cams.get(key);
    if (!entry) { entry = {}; this.cams.set(key, entry); }
    const k = padding.toFixed(9);
    if (!entry[k]) entry[k] = esCamOutline(cam, padding, roller);
    return entry[k];
  }
  // EngineSimApplication::render for the simulation objects: three sublayer
  // passes over rods, pistons and chambers, crankshafts, banks and heads, then
  // drawn in layer order like delta-studio's sorted draw calls.
  render(view) {
    const app = this.app, ctx = app.ctx, e = app.simulator.engine, colors = app.colors, p = app.painter;
    const b = view.bounds, ppu = b.height() * view.zoom / app.displayHeight, cosA = Math.cos(app.displayAngle);
    const cam = view.camera(), c = b.pos(ESRef.center);
    ctx.save();
    ctx.beginPath(); ctx.rect(b.x0, b.y0, b.width(), b.height()); ctx.clip();
    ctx.transform(ppu * cosA, 0, 0, ppu, c[0] - cam[0] * ppu * cosA, c[1] - cam[1] * ppu);
    const cmds = [];
    const push = (layer, fn) => cmds.push([layer, cmds.length, fn]);
    const at = (x, y, theta, scale, fn) => () => { ctx.save(); ctx.translate(x, y); ctx.rotate(theta); ctx.scale(scale, scale); fn(); ctx.restore(); };
    const model = (name, color) => p.model(app.models[name], color);
    const L0 = app.layer0, L1 = app.layer1, px5 = app.pixelsToUnits(5);
    const crank = e.crank, throwR = crank.throw, rodL = e.rodSpec.length, ch = e.pistonSpec.compressionHeight, wp = e.pistonSpec.wristPinPosition || 0;
    const grey = s => esMix(colors.background, colors.foreground, s);

    for (let sub = 0; sub < 3; sub++) {
      e.cylinders.forEach((cyl, i) => {
        const bank = cyl.bank, layer = cyl.journal, inLayer = layer <= L1 && layer >= L0;
        // ConnectingRodObject
        if (sub === 0 && inLayer) {
          const color = this.tint(bank.index % 2 === 0 ? grey(0.9333) : grey(0.8667), layer - L0);
          const mx = (cyl.journalX + cyl.pinX) / 2, my = (cyl.journalY + cyl.pinY) / 2;
          push(0x32 - layer, at(mx, my, cyl.rodTheta, 1, () => {
            ctx.save(); ctx.translate(0, -rodL / 2); ctx.scale(throwR, throwR); model('ConnectingRod', color); ctx.restore();
            p.begin(); p.line(0, -rodL / 2 + throwR * 0.6, 0, rodL / 2, throwR * 0.5); p.fill(color);
          }));
        }
        // PistonObject
        if (inLayer) {
          const col = this.tint(colors.foreground, layer - L0), hole = this.tint(colors.background, layer - L0), s = bank.bore / 2;
          push(0x32 - layer, at(cyl.pinX, cyl.pinY, bank.angle + Math.PI, 1, () => {
            ctx.save(); ctx.translate(0, -ch - wp); ctx.scale(s, s); model('Piston', col); ctx.restore();
            p.begin(); p.circle(0, wp, bank.bore / 10 * 0.75); p.fill(hole);
          }));
        }
        // CombustionChamberObject
        const chamber = e.chambers[i];
        if (this.foremost(bank, L0) === cyl && chamber.lit) {
          const h = bank.head.chamberVolume / bank.boreArea, f = chamber.flame;
          const top = bank.deckHeight + h, bottom = bank.deckHeight + h - f.travelY;
          push(0x35, () => {
            p.begin();
            p.line(bank.x + bank.dx * top, bank.y + bank.dy * top, bank.x + bank.dx * bottom, bank.y + bank.dy * bottom, f.travelX * 2);
            p.fill(colors.orange.slice(0, 3).concat([0.6]));
          });
        }
      });
      // CrankshaftObject
      if (sub === 2) {
        const cx = crank.x || 0, cy = crank.y || 0;
        crank.journals.forEach((angle, layer) => {
          if (layer > L1 || layer < L0) return;
          const col = this.tint(grey(0.7), layer - L0);
          push(0x32 - layer, at(cx, cy, e.theta + angle, throwR, () => model('Crankshaft', col)));
        });
        push(0x32, at(cx, cy, e.theta, throwR, () => model('CrankSnout', grey(0.6))));
        push(0x32, at(cx, cy, e.theta, throwR, () => model('CrankSnoutThreads', grey(0.2))));
      }
      if (sub === 0) for (const bank of e.banks) { this.renderBank(bank, push); this.renderHead(bank, push, px5, at, model); }
    }
    cmds.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    for (const cmd of cmds) cmd[2]();
    ctx.restore();
  }
  // CylinderBankObject
  renderBank(bank, push) {
    const p = this.app.painter, h = bank.head.chamberVolume / bank.boreArea;
    const depth = 1 - bank.displayDepth, lw = bank.bore * 0.1, margin = lw * 0.25;
    const dx = -bank.dy * (margin + bank.bore / 2 + lw / 2), dy = bank.dx * (margin + bank.bore / 2 + lw / 2);
    const tx = bank.x + bank.dx * (bank.deckHeight + h), ty = bank.y + bank.dy * (bank.deckHeight + h);
    const bx = bank.x + bank.dx * (depth * bank.deckHeight), by = bank.y + bank.dy * (depth * bank.deckHeight);
    push(0, () => {
      p.begin();
      p.line(tx + dx, ty + dy, bx + dx, by + dy, lw); p.circle(bx + dx, by + dy, lw / 2);
      p.line(tx - dx, ty - dy, bx - dx, by - dy, lw); p.circle(bx - dx, by - dy, lw / 2);
      p.fill(this.app.colors.pink);
    });
  }
  // CylinderHeadObject
  renderHead(bank, push, px5, at, model) {
    const app = this.app, ctx = app.ctx, p = app.painter, colors = app.colors, head = bank.head, vt = head.valvetrain;
    const s = bank.bore / 2, h = head.chamberVolume / bank.boreArea;
    const front = this.foremost(bank, app.layer0);
    if (!front) return;
    const x = bank.x + bank.dx * (bank.deckHeight + h), y = bank.y + bank.dy * (bank.deckHeight + h);
    const headFrame = fn => at(x, y, bank.angle, 1, fn);
    const ivp = head.flipDisplay ? 0.5 : -0.5, layer = front.index;
    const roller = 300 * ES_UNITS.thou;
    push(0, headFrame(() => { ctx.scale(s, s); model('CylinderHead', colors.pink); }));
    push(1, headFrame(() => {
      ctx.scale(s, s);
      p.begin();
      let x0 = -0.689352, x1 = -0.88632;
      const y0 = 0.085, y1 = -0.077975;
      p.line(x0, y0, x1, y1, 0.2);
      p.line(-(x0 + 0.5) - 0.5, y0, -(x1 + 0.5) - 0.5, y1, 0.2);
      x0 = 0.689352; x1 = 0.88632;
      p.line(x0, y0, x1, y1, 0.2);
      p.line(-(x0 - 0.5) + 0.5, y0, -(x1 - 0.5) + 0.5, y1, 0.2);
      const stem = 0.0917 + px5 / s;
      p.line(-0.5, 0.2, -0.5, 1.5, stem); p.line(0.5, 0.2, 0.5, 1.5, stem);
      p.fill(colors.background);
    }));
    const valve = (pos, lift, color) => push(0x33, headFrame(() => {
      ctx.scale(s, s); ctx.translate(pos, -lift / s);
      model('Valve', color);
      p.begin(); p.circle(0, 1.99, (roller + px5 / 2) / s); p.fill(colors.background);
      p.begin(); p.circle(0, 1.99, roller / s); p.fill(color);
      p.begin(); p.circle(0, 1.99, roller * 0.25 / s); p.fill(colors.background);
    }));
    valve(ivp, vt.intakeValveLift(layer), colors.blue);
    valve(-ivp, vt.exhaustValveLift(layer), colors.yellow);
    const intakeCam = vt.activeIntakeCamshaft(), exhaustCam = vt.activeExhaustCamshaft();
    const camCenter = () => { p.begin(); p.circle(0, 0, roller * 0.25); p.fill(colors.background); };
    push(0, headFrame(() => {
      ctx.translate(-ivp * s, px5 / 2 + 1.99 * s + exhaustCam.baseRadius + roller);
      ctx.rotate(exhaustCam.angle() + exhaustCam.lobeAngles[layer]);
      p.model(this.camPath(exhaustCam, 0, roller), colors.yellow);
      camCenter();
    }));
    push(0, headFrame(() => {
      ctx.translate(ivp * s, roller + px5 / 2 + 1.99 * s + intakeCam.baseRadius);
      ctx.rotate(intakeCam.angle() + intakeCam.lobeAngles[layer]);
      p.model(this.camPath(intakeCam, px5 / 2, roller), colors.background);
      p.model(this.camPath(intakeCam, 0, roller), colors.blue);
      camCenter();
    }));
  }
}

/* ------------------------------------------------------- Crankshaft view */

// Not in engine-sim: the engine seen with the crankshaft lying across the
// screen, so every cylinder is visible at once. Each bank is turned to face
// the viewer. Positions are the simulator's, projected on the bank's cylinder
// axis; journals are spaced one bore and a quarter apart.
class ESCrankView extends ESUiElement {
  constructor(app) { super(app); }
  render() {
    const app = this.app, e = app.simulator.engine, colors = app.colors, p = this.p, ctx = app.ctx, b = this.bounds;
    this.drawFrame(b, 1, colors.foreground, colors.background);
    const banks = e.banks.slice(0, 2), crank = e.crank, r = crank.throw;
    const cx = crank.x || 0, cy = crank.y || 0, nJ = crank.journals.length;
    const bore = Math.max(...banks.map(k => k.bore)), pitch = bore * 1.25, lw = bore * 0.1;
    const headH = bore * 0.35, ch = e.pistonSpec.compressionHeight, skirt = bore * 0.45;
    const top = Math.max(...banks.map(k => k.deckHeight + k.head.chamberVolume / k.boreArea)) + headH;
    const inner = b.inset(14), iw = inner.width(), ih = inner.height() - 40;
    // Two banks either face each other across one crankshaft (tall), or sit
    // side by side, each over its own copy of it (wide); the one that draws
    // larger in this frame wins.
    const bankLength = nJ * pitch + pitch * 0.4, gap = pitch * 0.6, below = r * 1.8;
    const mirrored = { w: bankLength, h: banks.length > 1 ? 2 * top : top + below };
    const sideBySide = { w: banks.length * bankLength + (banks.length - 1) * gap, h: top + below };
    const fit = l => Math.min(iw / l.w, ih / l.h);
    const split = banks.length > 1 && fit(sideBySide) > fit(mirrored);
    const lay = split ? sideBySide : mirrored, ppu = fit(lay);
    const yMin = lay === mirrored && banks.length > 1 ? -top : -below;
    const ox = (inner.x0 + inner.x1) / 2, oy = inner.y0 + (ih - lay.h * ppu) / 2 - yMin * ppu;
    const grey = v => esMix(colors.background, colors.foreground, v);
    const labels = [];
    const shaft = x0 => {
      const z = j => x0 + (j - (nJ - 1) / 2) * pitch;
      p.begin(); p.line(z(0) - pitch * 0.7, 0, z(nJ - 1) + pitch * 0.7, 0, r * 0.45); p.fill(grey(0.6));
      p.begin(); p.line(z(nJ - 1) + pitch * 0.55, -r * 1.7, z(nJ - 1) + pitch * 0.55, r * 1.7, pitch * 0.18); p.fill(grey(0.45));
    };

    ctx.save();
    ctx.beginPath(); ctx.rect(b.x0, b.y0, b.width(), b.height()); ctx.clip();
    ctx.translate(ox, oy); ctx.scale(ppu, ppu);
    if (!split) shaft(0);

    banks.forEach((bank, bi) => {
      const sign = split || bi === 0 ? 1 : -1;
      const x0 = split ? (bi - (banks.length - 1) / 2) * (bankLength + gap) : 0;
      const z = j => x0 + (j - (nJ - 1) / 2) * pitch;
      if (split) shaft(x0);
      const along = (x, y) => (x - cx) * bank.dx + (y - cy) * bank.dy;
      const h = bank.head.chamberVolume / bank.boreArea, base = along(bank.x || 0, bank.y || 0);
      const deck = base + bank.deckHeight, roof = deck + h, liner = base + (1 - bank.displayDepth) * bank.deckHeight;
      const js = bank.cylinders.map(c => c.journal);
      const z0 = z(Math.min(...js)) - pitch / 2, z1 = z(Math.max(...js)) + pitch / 2;
      // Cylinder head across the bank.
      p.begin(); p.line(z0, sign * (roof + headH / 2), z1, sign * (roof + headH / 2), headH); p.fill(grey(0.3));
      for (const cyl of bank.cylinders) {
        const i = cyl.global, x = z(cyl.journal), chamber = e.chambers[i];
        const pin = along(cyl.pinX, cyl.pinY), crankPin = along(cyl.journalX, cyl.journalY);
        const wall = bank.bore / 2 + lw / 2;
        // Liner walls.
        p.begin();
        p.line(x - wall, sign * liner, x - wall, sign * roof, lw);
        p.line(x + wall, sign * liner, x + wall, sign * roof, lw);
        p.fill(colors.pink);
        // Crank web and counterweight, then the crank pin.
        p.begin(); p.line(x, -sign * r * 0.75, x, sign * crankPin, bank.bore * 0.32); p.fill(grey(0.7));
        p.begin(); p.circle(x, sign * crankPin, r * 0.32); p.fill(grey(0.8));
        // Connecting rod.
        p.begin(); p.line(x, sign * crankPin, x, sign * pin, r * 0.45); p.fill(grey(0.9333));
        // Flame between the crown and the head.
        const crown = pin + ch;
        if (chamber.lit) {
          const depth = Math.min(chamber.flame.travelY, roof - crown), width = Math.min(chamber.flame.travelX * 2, bank.bore);
          p.begin(); p.line(x, sign * roof, x, sign * (roof - depth), width); p.fill(colors.orange.slice(0, 3).concat([0.6]));
        }
        // Piston and wrist pin.
        p.begin(); p.line(x, sign * (pin - skirt), x, sign * crown, bank.bore * 0.96); p.fill(colors.foreground);
        p.begin(); p.circle(x, sign * pin, bank.bore / 10 * 0.75); p.fill(colors.background);
        labels.push([String(i + 1), ox + x * ppu, oy + sign * (roof + headH + bore * 0.12) * ppu + (sign > 0 ? 0 : -18), chamber.lit]);
      }
    });
    ctx.restore();
    for (const [t, x, y, lit] of labels) {
      const w = p.textWidth(t, 18);
      p.text(t, x - w / 2, y, 18, lit ? colors.orange : colors.foreground);
    }
    this.drawText('CRANKSHAFT VIEW', b.inset(10), 16);
    const order = app.simulator.firingOrder().map(i => i + 1).join('-');
    this.drawAlignedText('FIRING ORDER ' + order, b.inset(10), 16, [1, 1], [1, 1]);
  }
}

/* ------------------------------------------------------------ Audio output */

// The synthesizer's audio thread runs in an AudioWorklet. Its classes are
// copied from Engine-Sim-Physics.js into the worklet source.
const ES_WORKLET_PROCESSOR = `
class ESSynthProcessor extends AudioWorkletProcessor {
  constructor(options) {
    super();
    this.core = null;
    this.port.onmessage = e => this.message(e.data);
    this.message(options.processorOptions);
  }
  message(d) {
    if (d.type === 'load') {
      this.epoch = d.epoch; this.channels = d.channels;
      this.core = new ESSynthesizerCore(d.channels, d.irs, sampleRate, 128);
      this.core.setParameters(d.params);
      this.cap = 1 << 17; this.mask = this.cap - 1;
      this.buf = Array.from({ length: d.channels }, () => new Float32Array(this.cap));
      this.ins = Array.from({ length: d.channels }, () => new Float32Array(128));
      this.r = 0; this.w = 0; this.consumed = 0; this.blocks = 0; this.underruns = 0; this.clipped = 0;
      this.wave = new Float32Array(256); this.waveN = 0;
    } else if (d.type === 'input' && this.core && d.epoch === this.epoch) {
      const n = d.data[0].length;
      for (let c = 0; c < this.channels; c++) { const b = this.buf[c], src = d.data[c]; for (let i = 0; i < n; i++) b[(this.w + i) & this.mask] = src[i]; }
      this.w += n;
      if (this.w - this.r > this.cap) this.r = this.w - this.cap;
    } else if (d.type === 'params' && this.core) this.core.setParameters(d.params);
  }
  process(inputs, outputs) {
    const out = outputs[0][0];
    if (!out || !this.core) return true;
    if (this.w - this.r >= out.length) {
      for (let c = 0; c < this.channels; c++) { const b = this.buf[c], dst = this.ins[c]; for (let i = 0; i < out.length; i++) dst[i] = b[(this.r + i) & this.mask]; }
      this.core.render(this.ins, out);
      this.r += out.length; this.consumed += out.length;
    } else { out.fill(0); if (this.consumed > 0) this.underruns++; }
    for (let i = 0; i < out.length; i += 4) this.wave[this.waveN++] = out[i];
    if (++this.blocks % 4 === 0) {
      this.port.postMessage({ epoch: this.epoch, consumed: this.consumed, queued: this.w - this.r, underruns: this.underruns, clipped: this.core.overs | 0, gain: this.core.attenuation, wave: this.wave.slice(0, this.waveN) });
      this.waveN = 0;
    }
    return true;
  }
}
registerProcessor('es-synth', ESSynthProcessor);
`;

function esWorkletSource() {
  return `const ES_LEVELER_LOOKAHEAD = ${ES_LEVELER_LOOKAHEAD};\n` + [esSoftClip, ESButterworth, ESJitter, ESFFT, ESConvolution, ESSynthesizerCore].map(c => c.toString()).join('\n') + ES_WORKLET_PROCESSOR;
}

function esDecodeInt16(base64) {
  const bin = atob(base64), bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return new Int16Array(bytes.buffer);
}

/* ------------------------------------------------------- Page controls */

// Not in engine-sim: an engine bar above the simulator and a panel of live
// controls below it, so the engine can be driven without the keyboard. They
// write the same state the keys do, and show what the keys changed.
const ES_ENGINE_BAR_SLOTS = 6;
// The engines ask for 10 or 20 kHz; the page runs them at 8 kHz by default,
// which halves the physics cost of the 20 kHz engines with the same torque
// and idle within a few percent. 0 keeps each engine's own rate.
const ES_DEFAULT_SIM_FREQUENCY = 8000;
const ES_SIM_FREQUENCIES = [[0, 'Original del motor'], [12000, '12 kHz'], [10000, '10 kHz'], [8000, '8 kHz · liviana'], [6000, '6 kHz · muy liviana']];
const ES_SIM_SPEEDS = [[1, 'Tiempo real'], [1 / 10, '1/10'], [1 / 100, '1/100'], [1 / 200, '1/200'], [1 / 500, '1/500'], [1 / 1000, '1/1000']];

function esEl(tag, attrs = {}, children = []) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'text') el.textContent = v;
    else if (k.startsWith('on')) el.addEventListener(k.slice(2), v);
    else el.setAttribute(k, v);
  }
  for (const c of [].concat(children)) if (c) el.append(c);
  return el;
}

class ESEngineBar {
  constructor(app, host) {
    this.app = app; this.buttons = [];
    const list = esEl('div', { class: 'engine-list', role: 'tablist', 'aria-label': 'Motores' });
    ES_ENGINE_ORDER.forEach((key, i) => {
      const b = esEl('button', { type: 'button', role: 'tab', class: 'engine', onclick: () => app.selectEngine(i) },
        [esEl('span', { class: 'engine-name', text: ES_ENGINES[key].name }), esEl('span', { class: 'engine-meta', text: esEngineSummary(ES_ENGINES[key]) })]);
      this.buttons.push(b); list.append(b);
    });
    for (let i = ES_ENGINE_ORDER.length; i < ES_ENGINE_BAR_SLOTS; i++) {
      list.append(esEl('button', { type: 'button', class: 'engine', disabled: '' },
        [esEl('span', { class: 'engine-name', text: 'Próximamente' }), esEl('span', { class: 'engine-meta', text: 'Motor ' + (i + 1) })]));
    }
    this.views = ['Corte', 'Cigüeñal', 'Ambas'].map((t, i) => esEl('button', { type: 'button', text: t, onclick: () => app.setViewMode(i) }));
    const views = esEl('div', { class: 'views', role: 'group', 'aria-label': 'Vista · O' }, [esEl('span', { text: 'Vista' }), ...this.views]);
    host.append(list, views, esEl('a', { class: 'to-controls', href: '#controls', text: 'Controles ↓' }));
    this.update();
  }
  update() {
    this.buttons.forEach((b, i) => b.setAttribute('aria-selected', String(i === this.app.engineIndex)));
    this.views.forEach((b, i) => b.setAttribute('aria-pressed', String(i === this.app.viewMode)));
  }
}

function esEngineSummary(spec) {
  const n = spec.banks.reduce((s, b) => s + b.cylinders.length, 0);
  return `${n} cil. · ${Math.round(Math.abs(spec.redline) / ES_UNITS.rpm)} rpm`;
}

class ESControlPanel {
  constructor(app, host) {
    this.app = app; this.host = host; this.lastSync = 0;
    const range = (label, min, max, step, value, oninput, fmt) => {
      const out = esEl('output'), input = esEl('input', { type: 'range', min, max, step, value });
      const show = () => { out.textContent = fmt(+input.value); };
      input.addEventListener('input', () => { oninput(+input.value); show(); });
      show();
      return { input, out, show, row: esEl('label', { class: 'row' }, [esEl('span', { text: label }), input, out]) };
    };
    const toggle = (label, onclick) => {
      const b = esEl('button', { type: 'button', class: 'toggle', 'aria-pressed': 'false', text: label, onclick });
      return b;
    };
    const group = (title, hint, rows) => esEl('section', { class: 'group' }, [esEl('h2', { text: title }), hint && esEl('p', { class: 'hint', text: hint }), ...rows]);

    // Driving
    this.ignition = toggle('Encendido · A', () => this.app.setIgnition(!this.app.simulator.ignitionModule.enabled));
    this.starter = esEl('button', { type: 'button', class: 'toggle', text: 'Arranque · mantener S' });
    const hold = on => ev => { ev.preventDefault(); this.app.panel.starter = on; };
    this.starter.addEventListener('pointerdown', hold(true));
    for (const ev of ['pointerup', 'pointerleave', 'pointercancel']) this.starter.addEventListener(ev, hold(false));
    this.throttle = range('Acelerador', 0, 100, 1, 0, v => { this.app.panel.throttle = v / 100; }, v => v + ' %');
    this.simSpeed = esEl('select', { onchange: () => { this.app.panel.simSpeed = +this.simSpeed.value; } },
      ES_SIM_SPEEDS.map(([v, t]) => esEl('option', { value: v, text: t })));
    this.simFrequency = esEl('select', { onchange: () => this.app.setSimulationFrequency(+this.simFrequency.value) },
      ES_SIM_FREQUENCIES.map(([v, t]) => esEl('option', { value: v, text: t })));
    this.simFrequency.value = this.app.panel.simFrequency;
    this.simFrequencyOut = esEl('output');
    const drive = group('Conducción', 'Q, W, E y R aceleran mientras se mantienen. El acelerador de acá queda fijo.', [
      esEl('div', { class: 'buttons' }, [this.ignition, this.starter]),
      this.throttle.row,
      esEl('label', { class: 'row' }, [esEl('span', { text: 'Velocidad de simulación' }), this.simSpeed, esEl('output')]),
      esEl('label', { class: 'row' }, [esEl('span', { text: 'Frecuencia de simulación' }), this.simFrequency, this.simFrequencyOut]),
      esEl('p', { class: 'hint', text: 'Es la frecuencia de la física, y de ella sale el sonido. Más baja pesa menos para la computadora y suena un poco más opaca.' }),
    ]);

    // Load
    this.dyno = toggle('Dinamómetro · D', () => this.app.setDyno(!this.app.simulator.dyno.enabled));
    this.hold = toggle('Mantener RPM · H', () => this.app.setHold(!this.app.simulator.dyno.hold));
    this.holdRpm = range('RPM objetivo', 500, 9000, 100, 3000, v => this.app.setDynoSpeed(v * ES_UNITS.rpm), v => v + ' rpm');
    this.gear = esEl('select', { onchange: () => this.app.setGear(+this.gear.value) });
    this.clutch = range('Embrague', 0, 100, 1, 100, v => { this.app.panel.clutch = v / 100; }, v => v === 100 ? 'acoplado' : v === 0 ? 'pisado' : v + ' %');
    const load = group('Carga del motor', 'Con el dinamómetro y "mantener RPM" el banco frena el motor a las RPM objetivo y mide el torque.', [
      esEl('div', { class: 'buttons' }, [this.dyno, this.hold]),
      this.holdRpm.row,
      esEl('label', { class: 'row' }, [esEl('span', { text: 'Marcha · ↑ ↓' }), this.gear, esEl('output')]),
      this.clutch.row,
    ]);

    // Ignition
    this.order = esEl('input', { type: 'text', autocomplete: 'off', spellcheck: 'false', 'aria-label': 'Orden de encendido' });
    this.orderNote = esEl('p', { class: 'hint' });
    const applyOrder = () => {
      const order = this.order.value.split(/[^0-9]+/).filter(Boolean).map(x => +x - 1);
      this.orderNote.textContent = this.app.simulator.setFiringOrder(order)
        ? 'Aplicado. Si no coincide con el cigüeñal, el motor falla o no arranca.'
        : `Escribí cada cilindro del 1 al ${this.app.simulator.engine.cylinders.length} una vez, por ejemplo ${this.defaultOrder}.`;
      this.syncOrder(true);
    };
    this.order.addEventListener('keydown', ev => { if (ev.key === 'Enter') applyOrder(); });
    this.plugs = esEl('div', { class: 'plugs' });
    this.advance = range('Avance extra', -20, 20, 1, 0, v => { this.app.panel.timingOffset = v; this.app.simulator.ignitionModule.timingOffset = v * ES_UNITS.deg; }, v => (v > 0 ? '+' : '') + v + '°');
    const ignition = group('Encendido', 'Cambiar el orden mueve la chispa y las levas de cada cilindro. Apagá bujías para escuchar un cilindro muerto.', [
      esEl('div', { class: 'row stack' }, [esEl('span', { text: 'Orden de encendido' }), esEl('div', { class: 'order' }, [this.order,
        esEl('button', { type: 'button', text: 'Aplicar', onclick: applyOrder }),
        esEl('button', { type: 'button', text: 'Original', onclick: () => { this.order.value = this.defaultOrder; applyOrder(); } })])]),
      this.orderNote,
      esEl('div', { class: 'row' }, [esEl('span', { text: 'Bujías' }), this.plugs]),
      this.advance.row,
    ]);

    // Sound
    const audio = (label, name, max, fmt) => range(label, 0, max, max / 100, this.app.audioParams[name], v => this.app.setAudioParameters({ [name]: v }), fmt);
    this.audioRanges = {
      volume: audio('Volumen · Z', 'volume', 1, v => Math.round(v * 100) + ' %'),
      convolution: audio('Convolución · X', 'convolution', 1, v => Math.round(v * 100) + ' %'),
      airNoise: audio('Ruido de baja frec. · V', 'airNoise', 1, v => Math.round(v * 100) + ' %'),
      inputSampleNoise: audio('Ruido de alta frec. · B', 'inputSampleNoise', 1, v => Math.round(v * 100) + ' %'),
    };
    this.audioStatus = esEl('p', { class: 'hint' });
    const sound = group('Sonido', null, [...Object.values(this.audioRanges).map(r => r.row), this.audioStatus]);

    this.readout = esEl('div', { class: 'readout', 'aria-live': 'off' });
    host.append(esEl('div', { class: 'panel-head' }, [esEl('h1', { text: 'Controles' }), this.readout]),
      esEl('div', { class: 'groups' }, [drive, load, ignition, sound]));
    this.engineChanged();
  }

  // Called after an engine is loaded.
  engineChanged() {
    const app = this.app, sim = app.simulator, e = sim.engine;
    this.defaultOrder = sim.firingOrder().map(i => i + 1).join('-');
    this.order.value = this.defaultOrder; this.orderNote.textContent = '';
    this.gear.replaceChildren(esEl('option', { value: -1, text: 'Neutral' }),
      ...sim.transmission.gearRatios.map((r, i) => esEl('option', { value: i, text: `${i + 1}ª (${r.toFixed(2)})` })));
    this.plugs.replaceChildren(...e.cylinders.map((c, i) => {
      const b = esEl('button', { type: 'button', class: 'toggle plug', 'aria-pressed': 'true', text: String(i + 1), title: 'Bujía del cilindro ' + (i + 1) });
      b.addEventListener('click', () => {
        const plug = app.simulator.ignitionModule.plugs[i];
        plug.enabled = !plug.enabled; b.setAttribute('aria-pressed', String(plug.enabled));
      });
      return b;
    }));
    const max = Math.ceil(Math.abs(e.redline) / ES_UNITS.rpm / 500) * 500;
    this.holdRpm.input.max = max;
    sim.ignitionModule.timingOffset = app.panel.timingOffset * ES_UNITS.deg;
    for (const [name, r] of Object.entries(this.audioRanges)) { r.input.value = app.audioParams[name]; r.show(); }
    this.sync(true);
  }
  syncOrder(force) {
    if (!force && document.activeElement === this.order) return;
    this.order.value = this.app.simulator.firingOrder().map(i => i + 1).join('-');
  }
  // Shows state the keyboard changed. Runs a few times a second.
  sync(force) {
    const now = performance.now();
    if (!force && now - this.lastSync < 150) return;
    this.lastSync = now;
    const app = this.app, sim = app.simulator, e = sim.engine, t = sim.transmission;
    const press = (b, on) => b.setAttribute('aria-pressed', String(on));
    press(this.ignition, sim.ignitionModule.enabled);
    press(this.starter, sim.starterMotor.enabled);
    press(this.dyno, sim.dyno.enabled);
    press(this.hold, sim.dyno.hold);
    if (document.activeElement !== this.gear) this.gear.value = t.gear;
    this.simFrequencyOut.textContent = Math.round(sim.simulationFrequency) + ' Hz';
    for (const [name, r] of Object.entries(this.audioRanges)) if (document.activeElement !== r.input && +r.input.value !== app.audioParams[name]) { r.input.value = app.audioParams[name]; r.show(); }
    const torque = sim.filteredDynoTorque() / ES_UNITS.lbft, power = sim.dynoPower() / 745.699872;
    this.readout.textContent = `${Math.abs(e.rpm()).toFixed(0)} rpm · ${(e.manifoldPressure() / 1000).toFixed(0)} kPa` +
      (sim.dyno.enabled ? ` · ${torque.toFixed(0)} lb-ft · ${power.toFixed(0)} hp` : '');
    const a = app.audio;
    this.audioStatus.textContent = !a ? 'El sonido arranca con el primer clic o tecla.'
      : `Audio a ${a.ctx.sampleRate} Hz · ${((a.sent - a.consumed) / a.rate * 1000).toFixed(0)} ms en cola · cortes ${a.underruns || 0} · saturación ${a.clipped || 0}`;
  }
}

/* -------------------------------------------------- EngineSimApplication */

const ES_SPEED_KEYS = { Digit1: 1 / 10, Digit2: 1 / 100, Digit3: 1 / 200, Digit4: 1 / 500, Digit5: 1 / 1000 };

class ESApp {
  constructor(canvas, assets) {
    this.canvas = canvas; this.ctx = canvas.getContext('2d', { alpha: false });
    this.painter = new ESPainter(this.ctx); this.colors = ES_COLORS; this.assets = assets;
    this.models = {};
    for (const [name, loops] of Object.entries(assets.models)) {
      const path = new Path2D();
      for (const l of loops) { path.moveTo(l[0], l[1]); for (let i = 2; i < l.length; i += 2) path.lineTo(l[i], l[i + 1]); path.closePath(); }
      this.models[name] = path;
    }
    this.displayHeight = 2 * 12 * ES_UNITS.inch; this.displayAngle = 0; this.screen = 0;
    this.width = 1920; this.height = 1080; this.averageFramerate = 60;
    this.keys = new Set(); this.pressed = new Set();
    this.mouse = [0, 0]; this.mouseDown = false; this.mousePressed = false; this.mouseReleased = false; this.wheel = 0;
    this.lastMouseWheel = 0; this.uiLastScroll = 0; this.hover = null; this.dragStart = null; this.mouse0 = [0, 0];
    this.audio = null; this.levelerGain = 1; this.oscillatorSampleOffset = 0;
    this.panel = { throttle: 0, starter: false, clutch: 1, simSpeed: 1, timingOffset: 0, simFrequency: ES_DEFAULT_SIM_FREQUENCY };

    this.root = new ESUiElement(this);
    this.engineView = this.root.add(new ESEngineView(this));
    this.crankView = this.root.add(new ESCrankView(this));
    // 0: cross-section (the original), 1: crankshaft view, 2: both.
    this.viewMode = 2;
    this.rightGaugeCluster = this.root.add(new ESRightGaugeCluster(this));
    this.oscCluster = this.root.add(new ESOscilloscopeCluster(this));
    this.performanceCluster = this.root.add(new ESPerformanceCluster(this));
    this.loadSimulationCluster = this.root.add(new ESLoadSimulationCluster(this));
    this.mixerCluster = this.root.add(new ESMixerCluster(this));
    this.infoCluster = this.root.add(new ESInfoCluster(this));
    this.scene = new ESScene(this);

    const params = new URLSearchParams(location.search);
    const requested = params.get('engine');
    this.engineIndex = Math.max(0, ES_ENGINE_ORDER.indexOf(requested));
    this.loadEngine(ES_ENGINE_ORDER[this.engineIndex]);
    const bar = document.getElementById('engine-bar'), controls = document.getElementById('controls');
    this.engineBar = bar ? new ESEngineBar(this, bar) : null;
    this.controlPanel = controls ? new ESControlPanel(this, controls) : null;
    this.bindInput();
    this.resize();
  }

  // EngineSimApplication::loadEngine
  loadEngine(key) {
    const spec = ES_ENGINES[key];
    this.engineKey = key;
    this.synthInput = new ESSynthesizerInput(spec.exhausts.length);
    this.simulator = new ESSimulator(spec, this.synthInput);
    const e = this.simulator.engine;
    if (this.panel.simFrequency) this.simulator.setSimulationFrequency(this.panel.simFrequency);
    this.layer0 = 0; this.layer1 = e.crank.journals.length;
    this.audioParams = {
      volume: 1, convolution: 1, dF_F_mix: e.hfGain, inputSampleNoise: e.jitter, inputSampleNoiseFrequencyCutoff: 10000,
      airNoise: e.noise, airNoiseFrequencyCutoff: 2000, levelerTarget: 30000, levelerMaxGain: 1.9, levelerMinGain: 0.00001,
    };
    this.targetSpeedSetting = 0; this.speedSetting = 0; this.dynoSpeed = 0;
    this.targetClutchPressure = 1; this.clutchPressure = 1;
    this.oscCluster.resetScopes(); this.loadSimulationCluster.resetReadings();
    if (this.audio) this.sendEngineToAudio();
    if (this.engineBar) this.engineBar.update();
    if (this.controlPanel) this.controlPanel.engineChanged();
  }
  impulseResponses() {
    return this.simulator.engine.exhausts.map(ex => {
      const ir = this.assets.irs[ex.impulseResponse];
      return esImpulseResponse(esDecodeInt16(ir.samples), ir.volume);
    });
  }
  sendEngineToAudio() {
    const a = this.audio;
    a.epoch++; a.sent = 0; a.consumed = 0; a.queued = 0;
    a.node.port.postMessage({ type: 'load', epoch: a.epoch, channels: this.synthInput.channels, irs: this.impulseResponses(), params: this.audioParams });
  }
  nextEngine(step) { this.selectEngine((this.engineIndex + step + ES_ENGINE_ORDER.length) % ES_ENGINE_ORDER.length); }
  selectEngine(index) {
    this.engineIndex = index;
    this.loadEngine(ES_ENGINE_ORDER[index]);
    this.infoCluster.setLogMessage('Loaded ' + this.simulator.engine.name);
    const url = new URL(location.href);
    url.searchParams.set('engine', ES_ENGINE_ORDER[index]);
    history.replaceState(null, '', url);
  }
  // The keys and the control panel change the simulator through these.
  setIgnition(on) {
    this.simulator.ignitionModule.enabled = on;
    this.infoCluster.setLogMessage(on ? 'IGNITION ENABLED' : 'IGNITION DISABLED');
  }
  setDyno(on) {
    this.simulator.dyno.enabled = on;
    this.infoCluster.setLogMessage(on ? 'DYNOMOMETER ENABLED' : 'DYNOMOMETER DISABLED');
  }
  setHold(on) {
    const sim = this.simulator;
    sim.dyno.hold = on;
    if (on && this.controlPanel) this.setDynoSpeed(+this.controlPanel.holdRpm.input.value * ES_UNITS.rpm);
    this.infoCluster.setLogMessage(on ? (sim.dyno.enabled ? 'HOLD ENABLED' : 'HOLD ON STANDBY [ENABLE DYNO. FOR HOLD]') : 'HOLD DISABLED');
  }
  setDynoSpeed(w) {
    const e = this.simulator.engine;
    this.dynoSpeed = esClamp(w, e.dynoMinSpeed, e.dynoMaxSpeed);
  }
  setSimulationFrequency(f) {
    this.panel.simFrequency = f;
    const sim = this.simulator;
    sim.setSimulationFrequency(f || sim.engine.simulationFrequency);
    this.infoCluster.setLogMessage('Set simulation freq to ' + sim.simulationFrequency.toFixed(0));
  }
  setGear(gear) {
    const t = this.simulator.transmission;
    t.changeGear(gear);
    this.infoCluster.setLogMessage(t.gear === -1 ? 'SHIFTED TO NEUTRAL' : 'SHIFTED TO ' + (t.gear + 1));
  }
  async startAudio() {
    if (this.audio || this.audioStarting) return;
    this.audioStarting = true;
    try {
      const ctx = new AudioContext({ sampleRate: 44100, latencyHint: 'interactive' });
      // Resume inside the click or key press that started the audio; some
      // browsers refuse once an await has ended the user gesture.
      const resumed = ctx.resume();
      const url = URL.createObjectURL(new Blob([esWorkletSource()], { type: 'text/javascript' }));
      try { await ctx.audioWorklet.addModule(url); }
      catch (err) { await ctx.audioWorklet.addModule('data:text/javascript;charset=utf-8,' + encodeURIComponent(esWorkletSource())); }
      finally { URL.revokeObjectURL(url); }
      const epoch = 1;
      const node = new AudioWorkletNode(ctx, 'es-synth', {
        numberOfInputs: 0, numberOfOutputs: 1, outputChannelCount: [1],
        processorOptions: { type: 'load', epoch, channels: this.synthInput.channels, irs: this.impulseResponses(), params: this.audioParams },
      });
      node.connect(ctx.destination);
      this.audio = { ctx, node, epoch, rate: ctx.sampleRate, sent: 0, consumed: 0, queued: 0, underruns: 0, clipped: 0 };
      node.port.onmessage = ev => this.audioMessage(ev.data);
      await resumed;
    } catch (err) {
      this.infoCluster.setLogMessage('Audio unavailable: ' + err.message);
      console.error(err);
    }
    this.audioStarting = false;
  }
  audioMessage(d) {
    const a = this.audio;
    if (d.epoch !== a.epoch) return;
    a.consumed = d.consumed; a.queued = d.queued; a.underruns = d.underruns; a.clipped = d.clipped; this.levelerGain = d.gain;
    const scope = this.oscCluster.audioWaveformScope;
    for (let i = 0; i < d.wave.length; i++) {
      scope.addDataPoint(this.oscillatorSampleOffset, d.wave[i]);
      this.oscillatorSampleOffset = (this.oscillatorSampleOffset + 4) % (44100 / 10);
    }
  }
  setAudioParameters(p) {
    Object.assign(this.audioParams, p);
    if (this.audio) this.audio.node.port.postMessage({ type: 'params', params: this.audioParams });
  }
  synthesizerInputLatency() { return this.audio ? (this.audio.sent - this.audio.consumed) / this.audio.rate : 0; }
  pixelsToUnits(px) { return px * this.displayHeight / this.engineView.bounds.height(); }

  /* --------------------------------------------------------------- input */
  bindInput() {
    const prevent = new Set(['Space', 'Tab', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'F1', 'F2', 'F3', 'Enter', 'PageUp', 'PageDown']);
    const typing = ev => ev.target instanceof Element && ev.target.matches('input[type=text], select, textarea');
    addEventListener('keydown', ev => {
      this.startAudio();
      if (typing(ev)) return;
      if (prevent.has(ev.code)) ev.preventDefault();
      if (!ev.repeat) this.pressed.add(ev.code);
      this.keys.add(ev.code);
    });
    addEventListener('pointerdown', () => this.startAudio());
    addEventListener('keyup', ev => { this.keys.delete(ev.code); });
    addEventListener('blur', () => this.keys.clear());
    const toLogical = ev => {
      const r = this.canvas.getBoundingClientRect();
      return [(ev.clientX - r.left) * this.width / r.width, this.height - (ev.clientY - r.top) * this.height / r.height];
    };
    this.canvas.addEventListener('pointerdown', ev => {
      this.mouse = toLogical(ev); this.mouseDown = true; this.mousePressed = true;
      this.canvas.setPointerCapture(ev.pointerId);
    });
    this.canvas.addEventListener('pointermove', ev => { this.mouse = toLogical(ev); });
    this.canvas.addEventListener('pointerup', ev => { this.mouse = toLogical(ev); this.mouseDown = false; this.mouseReleased = true; });
    // The page scrolls down to the controls, so the wheel only reaches the
    // simulator with one of its keys held, or Ctrl or Alt to zoom the engine.
    const wheelKeys = ['Space', 'KeyZ', 'KeyX', 'KeyC', 'KeyV', 'KeyB', 'KeyN', 'KeyG'];
    this.canvas.addEventListener('wheel', ev => {
      if (!ev.ctrlKey && !ev.altKey && !wheelKeys.some(k => this.keys.has(k))) return;
      ev.preventDefault();
      // Windows reports 120 per notch; browsers report about 100 pixels.
      const unit = ev.deltaMode === 1 ? 40 : ev.deltaMode === 2 ? 800 : 1;
      this.wheel += -ev.deltaY * unit * 1.2;
    }, { passive: false });
    addEventListener('resize', () => this.resize());
  }
  keyDown(code) { return this.keys.has(code); }
  keyPressed(code) { const p = this.pressed.has(code); this.pressed.delete(code); return p; }
  shiftDown() { return this.keys.has('ShiftLeft') || this.keys.has('ShiftRight'); }
  resize() {
    // The simulator fills the window below the engine bar; the controls sit
    // under it and the page scrolls to them.
    const bar = document.getElementById('engine-bar');
    const cw = document.documentElement.clientWidth || innerWidth, dpr = devicePixelRatio || 1;
    const ch = Math.max(360, innerHeight - (bar ? bar.offsetHeight : 0));
    // The original lays out in screen pixels with fixed text sizes, so small
    // windows get a larger logical resolution that is scaled down to fit.
    const k = Math.max(1, 1920 / cw, 1080 / ch);
    this.width = cw * k; this.height = ch * k;
    this.canvas.style.width = cw + 'px'; this.canvas.style.height = ch + 'px';
    this.canvas.width = Math.round(cw * dpr); this.canvas.height = Math.round(ch * dpr);
    this.pixelScale = dpr / k;
  }

  // EngineSimApplication::processEngineInput
  processEngineInput(dt) {
    const sim = this.simulator, e = sim.engine, info = this.infoCluster;
    const fine = this.keyDown('Space');
    const wheelDelta = this.wheel - this.lastMouseWheel;
    this.lastMouseWheel = this.wheel;
    let fineInUse = false;
    const adjust = (key, name, rate, fineRate, label) => {
      const v = esClamp(this.audioParams[name] + wheelDelta * (fine ? fineRate : rate) * dt);
      this.setAudioParameters({ [name]: v });
      info.setLogMessage(`[${key}] - Set ${label} to ${v.toFixed(6)}`);
      fineInUse = true;
    };
    if (this.keyDown('KeyZ')) adjust('Z', 'volume', 0.01, 0.001, 'volume');
    else if (this.keyDown('KeyX')) adjust('X', 'convolution', 0.01, 0.001, 'convolution level');
    else if (this.keyDown('KeyC')) adjust('C', 'dF_F_mix', 0.001, 0.00001, 'high freq. gain');
    else if (this.keyDown('KeyV')) adjust('V', 'airNoise', 0.01, 0.001, 'low freq. noise');
    else if (this.keyDown('KeyB')) adjust('B', 'inputSampleNoise', 0.01, 0.001, 'high freq. noise');
    else if (this.keyDown('KeyN')) {
      sim.setSimulationFrequency(esClamp(sim.simulationFrequency + wheelDelta * (fine ? 10 : 100) * dt, 400, 400000));
      fineInUse = true;
      info.setLogMessage('[N] - Set simulation freq to ' + sim.simulationFrequency.toFixed(6));
    } else if (this.keyDown('KeyG') && sim.dyno.hold) {
      if (wheelDelta > 0) this.dynoSpeed += e.dynoHoldStep;
      else if (wheelDelta < 0) this.dynoSpeed -= e.dynoHoldStep;
      this.dynoSpeed = esClamp(this.dynoSpeed, e.dynoMinSpeed, e.dynoMaxSpeed);
      info.setLogMessage('[G] - Set dyno speed to ' + (this.dynoSpeed / ES_UNITS.rpm).toFixed(6));
      fineInUse = true;
    }
    const prevTarget = this.targetSpeedSetting;
    this.targetSpeedSetting = fine ? this.targetSpeedSetting : this.panel.throttle;
    if (this.keyDown('KeyQ')) this.targetSpeedSetting = 0.01;
    else if (this.keyDown('KeyW')) this.targetSpeedSetting = 0.1;
    else if (this.keyDown('KeyE')) this.targetSpeedSetting = 0.2;
    else if (this.keyDown('KeyR')) this.targetSpeedSetting = 1;
    else if (fine && !fineInUse) this.targetSpeedSetting = esClamp(this.targetSpeedSetting + wheelDelta * 0.0001);
    if (prevTarget !== this.targetSpeedSetting) info.setLogMessage('Speed control set to ' + this.targetSpeedSetting.toFixed(6));
    this.speedSetting = this.targetSpeedSetting * 0.5 + 0.5 * this.speedSetting;
    e.setSpeedControl(this.speedSetting);

    const maxDepth = e.crank.journals.length;
    if (this.keyPressed('KeyM')) {
      if (this.layer0 + 1 < maxDepth) this.layer0++;
      info.setLogMessage('[M] - Set render layer to ' + this.layer0);
    }
    if (this.keyPressed('Comma')) {
      if (this.layer0 - 1 >= 0) this.layer0--;
      info.setLogMessage('[,] - Set render layer to ' + this.layer0);
    }
    if (this.keyPressed('KeyD')) this.setDyno(!sim.dyno.enabled);
    if (this.keyPressed('KeyH')) this.setHold(!sim.dyno.hold);
    if (sim.dyno.enabled) {
      if (!sim.dyno.hold) {
        if (sim.filteredDynoTorque() > ES_UNITS.lbft) this.dynoSpeed += 500 * ES_UNITS.rpm * dt;
        else this.dynoSpeed *= 1 / (1 + dt);
        if (this.dynoSpeed > e.redline) { sim.dyno.enabled = false; this.dynoSpeed = 0; }
      }
    } else if (!sim.dyno.hold) this.dynoSpeed = 0;
    this.dynoSpeed = esClamp(this.dynoSpeed, e.dynoMinSpeed, e.dynoMaxSpeed);
    sim.dyno.rotationSpeed = this.dynoSpeed;

    const prevStarter = sim.starterMotor.enabled;
    sim.starterMotor.enabled = this.keyDown('KeyS') || this.panel.starter;
    if (prevStarter !== sim.starterMotor.enabled) info.setLogMessage(sim.starterMotor.enabled ? 'STARTER ENABLED' : 'STARTER DISABLED');
    if (this.keyPressed('KeyA')) this.setIgnition(!sim.ignitionModule.enabled);
    const t = sim.transmission;
    if (this.keyPressed('ArrowUp')) {
      t.changeGear(t.gear + 1);
      info.setLogMessage('UPSHIFTED TO ' + (t.gear + 1));
    } else if (this.keyPressed('ArrowDown')) {
      t.changeGear(t.gear - 1);
      info.setLogMessage(t.gear !== -1 ? 'DOWNSHIFTED TO ' + (t.gear + 1) : 'SHIFTED TO NEUTRAL');
    }
    if (this.keyDown('KeyT')) this.targetClutchPressure -= 0.2 * dt;
    else if (this.keyDown('KeyU')) this.targetClutchPressure += 0.2 * dt;
    else if (this.shiftDown()) { this.targetClutchPressure = 0; info.setLogMessage('CLUTCH DEPRESSED'); }
    else if (!this.keyDown('KeyY')) this.targetClutchPressure = this.panel.clutch;
    this.targetClutchPressure = esClamp(this.targetClutchPressure);
    const clutchRC = fine ? 1 : 0.001, s = dt / (dt + clutchRC);
    this.clutchPressure = this.clutchPressure * (1 - s) + this.targetClutchPressure * s;
    t.clutchPressure = this.clutchPressure;
  }

  // EngineSimApplication::process
  process(frameDt) {
    frameDt = esClamp(frameDt, 1 / 200, 1 / 30);
    const sim = this.simulator;
    let speed = this.panel.simSpeed;
    for (const [code, s] of Object.entries(ES_SPEED_KEYS)) if (this.keyDown(code)) { speed = s; break; }
    if (this.keyDown('F1')) this.displayAngle += frameDt;
    else if (this.keyDown('F2')) this.displayAngle -= frameDt;
    else if (this.keyPressed('F3')) this.displayAngle = 0;
    sim.simulationSpeed = speed;
    const avg = esClamp(this.averageFramerate, 30, 1000);
    sim.startFrame(1 / avg, this.audio ? this.synthesizerInputLatency() : undefined);
    const t0 = performance.now(), iterations = sim.steps;
    while (sim.simulateStep()) this.oscCluster.sample();
    const t1 = performance.now();
    sim.endFrame();
    if (iterations > 0) this.performanceCluster.addTimePerTimestepSample((t1 - t0) / 1000 / iterations);
    const data = this.synthInput.drain();
    const n = data[0].length;
    if (this.audio && n > 0) {
      this.audio.node.port.postMessage({ type: 'input', epoch: this.audio.epoch, data }, data.map(d => d.buffer));
      this.audio.sent += n;
    }
    const target = sim.targetSynthesizerLatency;
    this.performanceCluster.addInputBufferUsageSample(this.synthesizerInputLatency() / target);
    const ctx = this.audio && this.audio.ctx;
    const deviceLatency = ctx ? (ctx.baseLatency || 0) + (ctx.outputLatency || 0) : 0;
    this.performanceCluster.addAudioLatencySample(this.audio ? (this.synthesizerInputLatency() + deviceLatency) / target : 0);
  }

  // UiManager::update
  updateUi(dt) {
    this.root.update(dt);
    const m = this.mouse, hover = this.root.mouseOver(m[0], m[1]);
    this.hover = hover;
    if (this.mousePressed) {
      this.dragStart = hover; this.mouse0 = m.slice();
      if (hover) hover.onMouseDown(m);
    }
    // A press and release can land in the same frame; both are handled.
    if (this.mouseReleased) {
      if (this.dragStart) this.dragStart.onMouseUp(m);
      if (hover && hover === this.dragStart) hover.onMouseClick(m);
      this.dragStart = null;
    }
    this.mousePressed = false; this.mouseReleased = false;
    if (this.wheel !== this.uiLastScroll) {
      if (hover) hover.onMouseScroll(this.wheel - this.uiLastScroll);
      this.uiLastScroll = this.wheel;
    }
    if (this.dragStart) this.dragStart.onDrag(this.mouse0, m);
  }

  // EngineSimApplication::renderScene layouts
  layout() {
    const W = this.width, H = this.height, win = new ESBounds(0, 0, W, H);
    const all = [this.engineView, this.rightGaugeCluster, this.oscCluster, this.performanceCluster, this.loadSimulationCluster, this.mixerCluster, this.infoCluster];
    if (this.screen === 0) {
      this.engineView.drawFrameOn = true;
      this.engineView.bounds = esGrid(win, 3, 2, 1, 0);
      this.rightGaugeCluster.bounds = esGrid(win, 3, 2, 2, 0, 1, 2);
      this.oscCluster.bounds = esGrid(win, 3, 2, 1, 1);
      this.performanceCluster.bounds = esGrid(win, 3, 3, 0, 1);
      this.loadSimulationCluster.bounds = esGrid(win, 3, 3, 0, 2);
      const top = esGrid(win, 3, 3, 0, 0);
      this.mixerCluster.bounds = esGrid(top, 1, 3, 0, 2);
      this.infoCluster.bounds = esGrid(top, 1, 3, 0, 0, 1, 2);
      for (const el of all) el.visible = true;
      this.oscCluster.activate();
    } else if (this.screen === 1) {
      this.engineView.drawFrameOn = false;
      this.engineView.bounds = win;
      this.engineView.activate();
      for (const el of all) el.visible = el === this.engineView;
    } else {
      this.engineView.drawFrameOn = true;
      this.engineView.bounds = esGrid(win, 3, 1, 0, 0, 2, 1);
      this.engineView.activate();
      this.rightGaugeCluster.bounds = esGrid(win, 3, 1, 2, 0);
      for (const el of all) el.visible = el === this.engineView || el === this.rightGaugeCluster;
    }
    // The crankshaft view takes the engine view's place, or its lower part.
    const area = this.engineView.bounds, crank = this.crankView;
    crank.visible = this.viewMode !== 0;
    if (this.viewMode === 1) { crank.bounds = area; this.engineView.visible = false; }
    else if (this.viewMode === 2) {
      const split = area.y0 + area.height() * 0.45;
      crank.bounds = new ESBounds(area.x0, area.y0, area.x1, split);
      this.engineView.bounds = new ESBounds(area.x0, split, area.x1, area.y1);
    }
  }
  setViewMode(mode) {
    this.viewMode = mode;
    this.infoCluster.setLogMessage(['CROSS-SECTION VIEW', 'CRANKSHAFT VIEW', 'CROSS-SECTION + CRANKSHAFT VIEW'][mode]);
    if (this.engineBar) this.engineBar.update();
  }
  renderScene() {
    const ctx = this.ctx, s = this.pixelScale;
    this.layout();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = esCss(this.colors.shadow);
    ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    // y up, origin at the bottom left, in the original's pixel units.
    ctx.setTransform(s, 0, 0, -s, 0, this.height * s);
    if (this.engineView.visible) this.scene.render(this.engineView);
    this.root.render();
    if (!this.audio) this.renderStartPrompt();
    this.painter.flushText();
  }
  // Web port: browsers start audio only after a click or a key press.
  renderStartPrompt() {
    const b = this.engineView.bounds, c = b.pos(ESRef.center), colors = this.colors;
    const box = ESBounds.sized(Math.min(b.width() - 40, 760), 150, c[0], b.y0 + 110, ESRef.center);
    this.root.drawFrame(box, 1, colors.foreground, colors.background);
    this.root.drawCenteredText('CLICK OR PRESS A KEY FOR SOUND', box.inset(20), 24, ESRef.tm);
    this.root.drawCenteredText('HAZ CLIC O PULSA UNA TECLA PARA EL SONIDO', box.inset(20).moved(0, -40), 16, ESRef.tm);
    this.root.drawCenteredText('A IGNITION  S STARTER  Q W E R THROTTLE  D DYNO', box.inset(20).moved(0, -80), 16, ESRef.tm);
  }

  // EngineSimApplication::run, one frame per animation frame.
  frame(now) {
    const dt = this.lastFrame === undefined ? 1 / 60 : Math.min(0.25, (now - this.lastFrame) / 1000);
    this.lastFrame = now;
    if (dt > 0) this.averageFramerate = 0.95 * this.averageFramerate + 0.05 / dt;
    if (this.keyPressed('Enter')) { this.loadEngine(this.engineKey); this.infoCluster.setLogMessage('Reloaded ' + this.simulator.engine.name); }
    if (this.keyPressed('PageUp')) this.nextEngine(-1);
    if (this.keyPressed('PageDown')) this.nextEngine(1);
    if (this.keyPressed('Tab')) this.screen = (this.screen + 1) % 3;
    if (this.keyPressed('KeyO')) this.setViewMode((this.viewMode + 1) % 3);
    if (this.keyPressed('KeyF')) {
      if (!document.fullscreenElement) { document.documentElement.requestFullscreen().catch(() => { }); this.infoCluster.setLogMessage('Entered fullscreen mode'); }
      else { document.exitFullscreen(); this.infoCluster.setLogMessage('Exited fullscreen mode'); }
    }
    const t0 = performance.now();
    this.processEngineInput(dt);
    this.process(dt);
    const t1 = performance.now();
    this.updateUi(dt);
    this.renderScene();
    const t2 = performance.now();
    this.timing = { process: t1 - t0, render: t2 - t1 };
    if (this.controlPanel) this.controlPanel.sync();
    this.pressed.clear();
  }
  run() {
    const loop = t => { this.frame(t); requestAnimationFrame(loop); };
    requestAnimationFrame(loop);
  }
}

if (typeof module !== 'undefined') module.exports = { ESApp, ESBounds, esGrid, esCamOutline, ES_COLORS };
