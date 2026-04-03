/**
 * physics.test.js
 *
 * Pure-function unit tests for the 3-body simulation.
 *
 * This module re-implements the pure physics functions and the Trail class
 * verbatim from simulation.js so they can be imported and tested without
 * triggering main(), Three.js scene setup, or DOM access.
 *
 * Every function here is a direct copy of the production code — if you change
 * the production code, update the copy here and add a new test for the change.
 *
 * Exported: { suite }  — an array of test groups consumed by test.html
 */

// ─── Constants (must match simulation.js) ────────────────────────────────────

const SOFTENING  = 0.08;
const MAX_HISTORY = 20000;

// Circular speed at r=1 for equilateral IC (three equal masses, G=1)
const V_CIRC = 1 / Math.pow(3, 0.25);   // ≈ 0.7598

// ─── Minimal THREE.js stubs ───────────────────────────────────────────────────
// These replace the CDN import so physics.test.js has no external dep.
// Only the surface area used by the tested functions is implemented.

class Vector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  clone() { return new Vector3(this.x, this.y, this.z); }
}

// BufferAttribute stub: wraps a Float32Array, tracks needsUpdate
class BufferAttribute {
  constructor(array, itemSize) {
    this.array = array;
    this.itemSize = itemSize;
    this.needsUpdate = false;
    // setUsage is called during construction in Trail but is a no-op for tests
  }
  setUsage() {}
}

// BufferGeometry stub: stores draw range and one named attribute
class BufferGeometry {
  constructor() {
    this._drawStart = 0;
    this._drawCount = 0;
    this.attributes  = {};
  }
  setAttribute(name, attr) { this.attributes[name] = attr; }
  setDrawRange(start, count) { this._drawStart = start; this._drawCount = count; }
}

// Line stub: no-op constructor
class Line {
  constructor(geo, mat) { this.geo = geo; this.mat = mat; }
}

// LineBasicMaterial stub: no-op
class LineBasicMaterial {
  constructor() {}
}

// BufferAttribute needs a DynamicDrawUsage constant — just a number
const DynamicDrawUsage = 35048;

// Minimal THREE namespace used inside Trail constructor
const THREE = {
  BufferGeometry,
  BufferAttribute,
  Line,
  LineBasicMaterial,
  DynamicDrawUsage,
  Vector3,
};

// ─── Production code (verbatim copies) ───────────────────────────────────────

function computeDerivatives(state, masses, G) {
  const n = masses.length;
  const deriv = new Float64Array(state.length);

  for (let i = 0; i < n; i++) {
    const ix = i * 6, ivx = i * 6 + 3;
    deriv[ix]     = state[ivx];
    deriv[ix + 1] = state[ivx + 1];
    deriv[ix + 2] = state[ivx + 2];

    let ax = 0, ay = 0, az = 0;
    for (let j = 0; j < n; j++) {
      if (i === j) continue;
      const jx = j * 6;
      const dx = state[jx]     - state[ix];
      const dy = state[jx + 1] - state[ix + 1];
      const dz = state[jx + 2] - state[ix + 2];
      const r2 = dx*dx + dy*dy + dz*dz + SOFTENING * SOFTENING;
      const r3 = r2 * Math.sqrt(r2);
      const f  = G * masses[j] / r3;
      ax += f * dx;
      ay += f * dy;
      az += f * dz;
    }
    deriv[ivx]     = ax;
    deriv[ivx + 1] = ay;
    deriv[ivx + 2] = az;
  }
  return deriv;
}

function rk4Step(bodies, dt, G) {
  const n = bodies.length;
  const masses = bodies.map(b => b.mass);

  const s0 = new Float64Array(n * 6);
  for (let i = 0; i < n; i++) {
    s0[i*6]   = bodies[i].pos.x;
    s0[i*6+1] = bodies[i].pos.y;
    s0[i*6+2] = bodies[i].pos.z;
    s0[i*6+3] = bodies[i].vel.x;
    s0[i*6+4] = bodies[i].vel.y;
    s0[i*6+5] = bodies[i].vel.z;
  }

  const k1 = computeDerivatives(s0, masses, G);

  const s1 = new Float64Array(n * 6);
  for (let i = 0; i < n * 6; i++) s1[i] = s0[i] + 0.5 * dt * k1[i];
  const k2 = computeDerivatives(s1, masses, G);

  const s2 = new Float64Array(n * 6);
  for (let i = 0; i < n * 6; i++) s2[i] = s0[i] + 0.5 * dt * k2[i];
  const k3 = computeDerivatives(s2, masses, G);

  const s3 = new Float64Array(n * 6);
  for (let i = 0; i < n * 6; i++) s3[i] = s0[i] + dt * k3[i];
  const k4 = computeDerivatives(s3, masses, G);

  for (let i = 0; i < n; i++) {
    const s = i * 6;
    bodies[i].pos.x = s0[s]   + dt/6 * (k1[s]   + 2*k2[s]   + 2*k3[s]   + k4[s]);
    bodies[i].pos.y = s0[s+1] + dt/6 * (k1[s+1] + 2*k2[s+1] + 2*k3[s+1] + k4[s+1]);
    bodies[i].pos.z = s0[s+2] + dt/6 * (k1[s+2] + 2*k2[s+2] + 2*k3[s+2] + k4[s+2]);
    bodies[i].vel.x = s0[s+3] + dt/6 * (k1[s+3] + 2*k2[s+3] + 2*k3[s+3] + k4[s+3]);
    bodies[i].vel.y = s0[s+4] + dt/6 * (k1[s+4] + 2*k2[s+4] + 2*k3[s+4] + k4[s+4]);
    bodies[i].vel.z = s0[s+5] + dt/6 * (k1[s+5] + 2*k2[s+5] + 2*k3[s+5] + k4[s+5]);
  }
}

// choreographyIC: pure, no globals
function choreographyIC(rawPos, rawVel, s) {
  const vs = 1 / Math.sqrt(s);
  return {
    masses:     [1.0, 1.0, 1.0],
    positions:  rawPos.map(([x, z]) => new Vector3(x * s, 0, z * s)),
    velocities: rawVel.map(([vx, vz]) => new Vector3(vx * vs, 0, vz * vs)),
  };
}

// equilateralIC: receives choreoScale as an argument (in production it reads a global)
function equilateralIC(vt, vr = 0, choreoScale = 3) {
  const s  = choreoScale;
  const vs = 1 / Math.sqrt(s);
  const VT = V_CIRC * vt * vs;
  const VR = V_CIRC * Math.abs(vr) * Math.sign(vr) * vs;

  return {
    masses: [1, 1, 1],
    positions: [
      new Vector3(      s, 0,          0),
      new Vector3(-0.5*s,  0,  0.866*s),
      new Vector3(-0.5*s,  0, -0.866*s),
    ],
    velocities: [
      new Vector3(-VR,                    0,  VT                  ),
      new Vector3(-0.866*VT + 0.5*VR,     0, -0.5*VT - 0.866*VR ),
      new Vector3( 0.866*VT + 0.5*VR,     0, -0.5*VT + 0.866*VR ),
    ],
  };
}

// Trail class — verbatim copy (uses the THREE stubs defined above)
class Trail {
  constructor(color) {
    this.hist = new Float32Array(MAX_HISTORY * 3);
    this.head  = 0;
    this.total = 0;
    this.buf   = new Float32Array(MAX_HISTORY * 3);

    this.geo = new THREE.BufferGeometry();
    const attr = new THREE.BufferAttribute(this.buf, 3);
    attr.setUsage(THREE.DynamicDrawUsage);
    this.geo.setAttribute('position', attr);
    this.geo.setDrawRange(0, 0);

    this.mat  = new THREE.LineBasicMaterial();
    this.line = new THREE.Line(this.geo, this.mat);
  }

  addPoint(v) {
    this.hist[this.head * 3]     = v.x;
    this.hist[this.head * 3 + 1] = v.y;
    this.hist[this.head * 3 + 2] = v.z;
    this.head  = (this.head + 1) % MAX_HISTORY;
    this.total = Math.min(this.total + 1, MAX_HISTORY);
  }

  updateVisible(n) {
    const show = Math.min(n, this.total);
    if (show === 0) {
      this.geo.setDrawRange(0, 0);
      return;
    }
    const start = (this.head - show + MAX_HISTORY * 2) % MAX_HISTORY;
    const hist = this.hist;
    const buf  = this.buf;

    if (start + show <= MAX_HISTORY) {
      buf.set(hist.subarray(start * 3, (start + show) * 3), 0);
    } else {
      const firstPart = MAX_HISTORY - start;
      buf.set(hist.subarray(start * 3), 0);
      buf.set(hist.subarray(0, (show - firstPart) * 3), firstPart * 3);
    }
    this.geo.setDrawRange(0, show);
    this.geo.attributes.position.needsUpdate = true;
  }

  reset() {
    this.head  = 0;
    this.total = 0;
    this.geo.setDrawRange(0, 0);
  }
}

// ─── Helpers ──────────────────────────────────────────────────────────────────

/** Build a body object with plain-object pos/vel (same shape as rk4Step expects) */
function makeBody(x, y, z, vx, vy, vz, mass = 1.0) {
  return { pos: new Vector3(x, y, z), vel: new Vector3(vx, vy, vz), mass };
}

/**
 * Compute kinetic + potential energy of a set of bodies.
 * Uses the same softened potential that the integrator uses:
 *   U_ij = -G * m_i * m_j / sqrt(r^2 + eps^2)
 */
function totalEnergy(bodies, G = 1.0) {
  let KE = 0, PE = 0;
  for (const b of bodies) {
    KE += 0.5 * b.mass * (b.vel.x**2 + b.vel.y**2 + b.vel.z**2);
  }
  for (let i = 0; i < bodies.length; i++) {
    for (let j = i + 1; j < bodies.length; j++) {
      const dx = bodies[j].pos.x - bodies[i].pos.x;
      const dy = bodies[j].pos.y - bodies[i].pos.y;
      const dz = bodies[j].pos.z - bodies[i].pos.z;
      const r = Math.sqrt(dx*dx + dy*dy + dz*dz + SOFTENING*SOFTENING);
      PE -= G * bodies[i].mass * bodies[j].mass / r;
    }
  }
  return KE + PE;
}

/** Sum of m*v over all bodies (returns {x,y,z} object) */
function totalMomentum(bodies) {
  let px = 0, py = 0, pz = 0;
  for (const b of bodies) {
    px += b.mass * b.vel.x;
    py += b.mass * b.vel.y;
    pz += b.mass * b.vel.z;
  }
  return { x: px, y: py, z: pz };
}

// ─── Test-runner primitives ───────────────────────────────────────────────────
// Collected by test.html and rendered there — no console dependency.

const suite = [];
let _current = null;

function describe(name, fn) {
  _current = { name, tests: [] };
  suite.push(_current);
  fn();
  _current = null;
}

function it(name, fn) {
  const entry = { name, passed: null, error: null };
  _current.tests.push(entry);
  try {
    fn();
    entry.passed = true;
  } catch (e) {
    entry.passed = false;
    entry.error  = e.message || String(e);
  }
}

function expect(actual) {
  return {
    toBe(expected) {
      if (actual !== expected)
        throw new Error(`Expected ${expected}, got ${actual}`);
    },
    toBeCloseTo(expected, eps = 1e-10) {
      if (Math.abs(actual - expected) > eps)
        throw new Error(`Expected ~${expected} (eps=${eps}), got ${actual}`);
    },
    toBeLessThan(bound) {
      if (actual >= bound)
        throw new Error(`Expected < ${bound}, got ${actual}`);
    },
    toBeGreaterThan(bound) {
      if (actual <= bound)
        throw new Error(`Expected > ${bound}, got ${actual}`);
    },
    toBeTruthy() {
      if (!actual) throw new Error(`Expected truthy, got ${actual}`);
    },
    toBeFalsy() {
      if (actual) throw new Error(`Expected falsy, got ${actual}`);
    },
    toBeFinite() {
      if (!isFinite(actual))
        throw new Error(`Expected finite, got ${actual}`);
    },
  };
}

// ─── Tests ────────────────────────────────────────────────────────────────────

// ── computeDerivatives ───────────────────────────────────────────────────────

describe('computeDerivatives — velocity-to-position derivatives', () => {
  it('copies vx/vy/vz of body 0 into position derivative slots', () => {
    // State: one body at rest, one moving at (3, 4, 5)
    const state = new Float64Array([
      0, 0, 0,  3, 4, 5,   // body 0: pos=(0,0,0) vel=(3,4,5)
      10, 0, 0, 0, 0, 0,   // body 1: far away, vel=0
    ]);
    const deriv = computeDerivatives(state, [1, 1], 1.0);
    // Position derivative for body 0 = its velocity
    expect(deriv[0]).toBe(3);
    expect(deriv[1]).toBe(4);
    expect(deriv[2]).toBe(5);
  });

  it('copies vx/vy/vz of body 1 into its position derivative slots', () => {
    const state = new Float64Array([
      0, 0, 0, 0, 0, 0,
      5, 5, 5, -1, -2, -3,
    ]);
    const deriv = computeDerivatives(state, [1, 1], 1.0);
    expect(deriv[6]).toBe(-1);
    expect(deriv[7]).toBe(-2);
    expect(deriv[8]).toBe(-3);
  });
});

describe('computeDerivatives — Newton\'s 3rd law (equal & opposite forces)', () => {
  it('force on body 0 from body 1 is equal and opposite to force on body 1 from body 0', () => {
    // Place two bodies apart; zero velocity
    const state = new Float64Array([
      0, 0, 0,  0, 0, 0,
      3, 0, 0,  0, 0, 0,
    ]);
    const masses = [2.0, 3.0];
    const deriv  = computeDerivatives(state, masses, 1.0);

    // acc_0 = F_on_0 / m_0;  acc_1 = F_on_1 / m_1
    // For Newton 3rd: F_on_0 = -F_on_1  ⟹  m_0*a_0 + m_1*a_1 = 0
    const fx0 = masses[0] * deriv[3];   // force on body 0 in x
    const fx1 = masses[1] * deriv[9];   // force on body 1 in x
    expect(Math.abs(fx0 + fx1)).toBeCloseTo(0, 1e-12);

    const fy0 = masses[0] * deriv[4];
    const fy1 = masses[1] * deriv[10];
    expect(Math.abs(fy0 + fy1)).toBeCloseTo(0, 1e-12);
  });

  it('forces sum to zero for all three bodies (momentum conservation in derivatives)', () => {
    const state = new Float64Array([
      1, 2, 3,  0, 0, 0,
      -5, 1, 0, 0, 0, 0,
      2, -3, 1, 0, 0, 0,
    ]);
    const masses = [1.5, 2.0, 0.8];
    const deriv  = computeDerivatives(state, masses, 1.0);

    // Sum of m_i * a_i should be zero (no external forces)
    let sumFx = 0, sumFy = 0, sumFz = 0;
    for (let i = 0; i < 3; i++) {
      sumFx += masses[i] * deriv[i*6 + 3];
      sumFy += masses[i] * deriv[i*6 + 4];
      sumFz += masses[i] * deriv[i*6 + 5];
    }
    expect(Math.abs(sumFx)).toBeCloseTo(0, 1e-10);
    expect(Math.abs(sumFy)).toBeCloseTo(0, 1e-10);
    expect(Math.abs(sumFz)).toBeCloseTo(0, 1e-10);
  });
});

describe('computeDerivatives — gravity scaling', () => {
  it('doubling G doubles the acceleration magnitude', () => {
    const state = new Float64Array([
      0, 0, 0, 0, 0, 0,
      4, 0, 0, 0, 0, 0,
    ]);
    const masses = [1.0, 1.0];
    const d1 = computeDerivatives(state, masses, 1.0);
    const d2 = computeDerivatives(state, masses, 2.0);
    // ax of body 0 at index 3
    expect(Math.abs(d2[3] / d1[3])).toBeCloseTo(2.0, 1e-10);
  });

  it('G=0 produces zero accelerations', () => {
    const state = new Float64Array([
      0, 0, 0, 5, 0, 0,
      3, 0, 0, 0, 0, 0,
    ]);
    const deriv = computeDerivatives(state, [1, 1], 0.0);
    // Position derivatives should still be the velocities
    expect(deriv[0]).toBe(5);
    // Acceleration components should be zero
    expect(deriv[3]).toBe(0);
    expect(deriv[4]).toBe(0);
    expect(deriv[9]).toBe(0);
  });
});

describe('computeDerivatives — softening prevents division by zero', () => {
  it('returns finite accelerations when two bodies are at identical positions', () => {
    // Coincident bodies: r=0, but softening keeps r2 = eps^2 > 0
    const state = new Float64Array([
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
    ]);
    const deriv = computeDerivatives(state, [1, 1], 1.0);
    // All acceleration components must be finite (dx=dy=dz=0 → force direction is 0 vector)
    for (let i = 3; i < 12; i += (i === 5 ? 4 : 1)) {
      expect(isFinite(deriv[i])).toBeTruthy();
    }
    // Specifically: with dx=dy=dz=0 the force magnitude is G*m/eps^2 but direction is 0
    expect(deriv[3]).toBeCloseTo(0, 1e-10);
    expect(deriv[9]).toBeCloseTo(0, 1e-10);
  });

  it('force on body 0 grows as bodies approach but stays finite', () => {
    // Separation 0.001 (much less than SOFTENING=0.08)
    const state = new Float64Array([
      0, 0, 0, 0, 0, 0,
      0.001, 0, 0, 0, 0, 0,
    ]);
    const deriv = computeDerivatives(state, [1, 1], 1.0);
    expect(isFinite(deriv[3])).toBeTruthy();
    // Separation 1000 (much more than SOFTENING)
    state[6] = 1000;
    const derivFar = computeDerivatives(state, [1, 1], 1.0);
    // Close-body force is larger in x than far-body force
    expect(Math.abs(deriv[3])).toBeGreaterThan(Math.abs(derivFar[3]));
  });
});

describe('computeDerivatives — single-body edge case', () => {
  it('returns zero acceleration for a single isolated body', () => {
    const state = new Float64Array([1, 2, 3, 4, 5, 6]);
    const deriv = computeDerivatives(state, [1.0], 1.0);
    // Position derivative = velocity
    expect(deriv[0]).toBe(4);
    expect(deriv[1]).toBe(5);
    expect(deriv[2]).toBe(6);
    // Acceleration = zero (no other bodies)
    expect(deriv[3]).toBe(0);
    expect(deriv[4]).toBe(0);
    expect(deriv[5]).toBe(0);
  });
});

// ── rk4Step ───────────────────────────────────────────────────────────────────

describe('rk4Step — momentum conservation', () => {
  it('conserves total linear momentum after one step (zero-momentum initial config)', () => {
    // Zero net momentum by construction
    const bodies = [
      makeBody(-3, 0, 0,  0,  0.5, 0, 1.0),
      makeBody( 3, 0, 0,  0, -0.5, 0, 1.0),
    ];
    const p0 = totalMomentum(bodies);
    rk4Step(bodies, 0.005, 1.0);
    const p1 = totalMomentum(bodies);

    expect(Math.abs(p1.x - p0.x)).toBeCloseTo(0, 1e-10);
    expect(Math.abs(p1.y - p0.y)).toBeCloseTo(0, 1e-10);
    expect(Math.abs(p1.z - p0.z)).toBeCloseTo(0, 1e-10);
  });

  it('conserves momentum for three unequal-mass bodies', () => {
    const bodies = [
      makeBody(-4,  1,  0,  0.3, -0.1,  0.0, 1.5),
      makeBody( 4, -1,  0, -0.1,  0.2,  0.0, 2.0),
      makeBody( 0,  0,  3, -0.2, -0.1,  0.0, 0.8),
    ];
    // Zero the CM velocity first so momentum is truly zero
    const p = totalMomentum(bodies);
    const totalM = bodies.reduce((s, b) => s + b.mass, 0);
    for (const b of bodies) {
      b.vel.x -= p.x / totalM;
      b.vel.y -= p.y / totalM;
      b.vel.z -= p.z / totalM;
    }

    const p0 = totalMomentum(bodies);
    rk4Step(bodies, 0.005, 1.0);
    const p1 = totalMomentum(bodies);

    expect(Math.abs(p1.x - p0.x)).toBeCloseTo(0, 1e-10);
    expect(Math.abs(p1.y - p0.y)).toBeCloseTo(0, 1e-10);
    expect(Math.abs(p1.z - p0.z)).toBeCloseTo(0, 1e-10);
  });
});

describe('rk4Step — rough energy conservation', () => {
  it('changes energy by less than 0.1% over 100 small steps (dt=0.005)', () => {
    // Relatively wide initial separation to avoid softening artefacts
    const bodies = [
      makeBody(-5, 0, 0,  0,  0.4, 0, 1.0),
      makeBody( 5, 0, 0,  0, -0.4, 0, 1.0),
    ];
    const E0 = totalEnergy(bodies);
    for (let i = 0; i < 100; i++) rk4Step(bodies, 0.005, 1.0);
    const E1 = totalEnergy(bodies);
    const relErr = Math.abs((E1 - E0) / E0);
    // RK4 with dt=0.005 should keep relative energy error well below 0.1%
    expect(relErr).toBeLessThan(0.001);
  });

  it('does not produce NaN positions after 100 steps', () => {
    const bodies = [
      makeBody(-3, 0, 1,  0.2, 0.1, 0, 1.5),
      makeBody( 3, 0,-1, -0.1,-0.2, 0, 1.0),
      makeBody( 0, 4, 0,  0.0, 0.1, 0, 2.0),
    ];
    for (let i = 0; i < 100; i++) rk4Step(bodies, 0.005, 1.0);
    for (const b of bodies) {
      expect(isFinite(b.pos.x)).toBeTruthy();
      expect(isFinite(b.pos.y)).toBeTruthy();
      expect(isFinite(b.vel.x)).toBeTruthy();
    }
  });
});

describe('rk4Step — free-flight limit', () => {
  it('bodies with G=0 move in straight lines', () => {
    const bodies = [
      makeBody(0, 0, 0,  1, 0, 0, 1.0),
      makeBody(5, 0, 0,  0, 2, 0, 1.0),
    ];
    const dt = 0.5;
    rk4Step(bodies, dt, 0.0);  // zero gravity

    // Each body should have moved exactly vel*dt
    expect(bodies[0].pos.x).toBeCloseTo(0 + 1*dt, 1e-12);
    expect(bodies[0].pos.y).toBeCloseTo(0,         1e-12);
    expect(bodies[1].pos.x).toBeCloseTo(5 + 0*dt,  1e-12);
    expect(bodies[1].pos.y).toBeCloseTo(0 + 2*dt,  1e-12);
  });
});

// ── choreographyIC ────────────────────────────────────────────────────────────

describe('choreographyIC — output structure', () => {
  it('returns three equal unit masses', () => {
    const ic = choreographyIC(
      [[1, 0], [-1, 0], [0, 1]],
      [[0, 1], [0, -1], [1, 0]],
      3
    );
    expect(ic.masses.length).toBe(3);
    ic.masses.forEach(m => expect(m).toBe(1.0));
  });

  it('returns exactly three positions and three velocities', () => {
    const ic = choreographyIC(
      [[1, 0], [-1, 0], [0, 1]],
      [[0, 1], [0, -1], [1, 0]],
      3
    );
    expect(ic.positions.length).toBe(3);
    expect(ic.velocities.length).toBe(3);
  });

  it('sets y=0 for all positions and velocities (XZ-plane solution)', () => {
    const ic = choreographyIC(
      [[1, 2], [-3, 4], [0, -1]],
      [[0.5, -0.5], [0.1, 0.2], [-0.6, 0.3]],
      2
    );
    ic.positions.forEach(p  => expect(p.y).toBe(0));
    ic.velocities.forEach(v => expect(v.y).toBe(0));
  });
});

describe('choreographyIC — Kepler scaling', () => {
  const rawPos = [[1, 0], [-1, 0], [0, 1]];
  const rawVel = [[0, 1], [0, -1], [1, 0]];

  it('positions scale linearly with s', () => {
    const ic1 = choreographyIC(rawPos, rawVel, 1);
    const ic2 = choreographyIC(rawPos, rawVel, 4);
    // Each position should be multiplied by 4
    ic1.positions.forEach((p, i) => {
      expect(ic2.positions[i].x).toBeCloseTo(p.x * 4, 1e-10);
      expect(ic2.positions[i].z).toBeCloseTo(p.z * 4, 1e-10);
    });
  });

  it('velocities scale as 1/sqrt(s)', () => {
    const ic1 = choreographyIC(rawPos, rawVel, 1);
    const ic4 = choreographyIC(rawPos, rawVel, 4);
    // 1/sqrt(4) = 0.5, so ic4 velocities should be half ic1's
    ic1.velocities.forEach((v, i) => {
      expect(ic4.velocities[i].x).toBeCloseTo(v.x * 0.5, 1e-10);
      expect(ic4.velocities[i].z).toBeCloseTo(v.z * 0.5, 1e-10);
    });
  });

  it('s=1 is a pass-through: positions equal rawPos values exactly', () => {
    const ic = choreographyIC([[2, 3], [-2, -3], [1, -1]], [[0, 0], [0, 0], [0, 0]], 1);
    expect(ic.positions[0].x).toBeCloseTo(2,  1e-12);
    expect(ic.positions[0].z).toBeCloseTo(3,  1e-12);
    expect(ic.positions[1].x).toBeCloseTo(-2, 1e-12);
  });

  it('produces only finite values for a range of s values', () => {
    [0.1, 1, 3, 8, 100].forEach(s => {
      const ic = choreographyIC(rawPos, rawVel, s);
      ic.positions.forEach(p => {
        expect(isFinite(p.x)).toBeTruthy();
        expect(isFinite(p.z)).toBeTruthy();
      });
      ic.velocities.forEach(v => {
        expect(isFinite(v.x)).toBeTruthy();
        expect(isFinite(v.z)).toBeTruthy();
      });
    });
  });

  it('figure-8 initial conditions have the correct zero-velocity third body at origin', () => {
    // Standard figure-8: third body starts at origin with vel (-2*vx, -2*vz)
    const vx = 0.46620369, vz = 0.43236573;
    const ic = choreographyIC(
      [[-0.97000436, 0.24308753], [0.97000436, -0.24308753], [0, 0]],
      [[vx, vz], [vx, vz], [-2*vx, -2*vz]],
      1   // s=1 so we can check raw values
    );
    // Third body position should be at origin (scaled by 1)
    expect(ic.positions[2].x).toBeCloseTo(0, 1e-10);
    expect(ic.positions[2].z).toBeCloseTo(0, 1e-10);
  });

  it('figure-8 bodies 0 and 1 have equal velocities at s=1', () => {
    const vx = 0.46620369, vz = 0.43236573;
    const ic = choreographyIC(
      [[-0.97000436, 0.24308753], [0.97000436, -0.24308753], [0, 0]],
      [[vx, vz], [vx, vz], [-2*vx, -2*vz]],
      1
    );
    expect(ic.velocities[0].x).toBeCloseTo(ic.velocities[1].x, 1e-10);
    expect(ic.velocities[0].z).toBeCloseTo(ic.velocities[1].z, 1e-10);
  });
});

// ── equilateralIC ─────────────────────────────────────────────────────────────

describe('equilateralIC — output structure', () => {
  it('returns three equal unit masses', () => {
    const ic = equilateralIC(1.0, 0, 3);
    expect(ic.masses.length).toBe(3);
    ic.masses.forEach(m => expect(m).toBe(1));
  });

  it('sets y=0 for all positions and velocities', () => {
    const ic = equilateralIC(0.8, 0.3, 3);
    ic.positions.forEach(p  => expect(p.y).toBe(0));
    ic.velocities.forEach(v => expect(v.y).toBe(0));
  });
});

describe('equilateralIC — 3-fold positional symmetry', () => {
  it('body 0 is at (s, 0, 0) — vertex on positive X axis', () => {
    const s  = 3;
    const ic = equilateralIC(1.0, 0, s);
    expect(ic.positions[0].x).toBeCloseTo(s, 1e-10);
    expect(ic.positions[0].z).toBeCloseTo(0, 1e-10);
  });

  it('all three positions lie on a circle of radius s', () => {
    const s  = 5;
    const ic = equilateralIC(1.0, 0, s);
    ic.positions.forEach(p => {
      const r = Math.sqrt(p.x**2 + p.z**2);
      expect(r).toBeCloseTo(s, 1e-3);  // 0.866 ≈ sqrt(3)/2, slight rounding in the IC
    });
  });

  it('positions are 120° apart (dot-product test)', () => {
    const s  = 4;
    const ic = equilateralIC(1.0, 0, s);
    const [p0, p1, p2] = ic.positions;
    // cos(120°) = -0.5  ⟹  p0·p1 / (|p0||p1|) ≈ -0.5
    const dot01 = p0.x*p1.x + p0.z*p1.z;
    const r     = s * s;  // |p|^2 = s^2
    expect(dot01 / r).toBeCloseTo(-0.5, 1e-3);

    const dot02 = p0.x*p2.x + p0.z*p2.z;
    expect(dot02 / r).toBeCloseTo(-0.5, 1e-3);
  });
});

describe('equilateralIC — zero net momentum', () => {
  it('sum of m*v is zero in x for pure tangential config', () => {
    const ic = equilateralIC(1.0, 0, 3);
    const px = ic.velocities.reduce((s, v, i) => s + ic.masses[i] * v.x, 0);
    expect(Math.abs(px)).toBeCloseTo(0, 1e-10);
  });

  it('sum of m*v is zero in z for pure tangential config', () => {
    const ic = equilateralIC(1.0, 0, 3);
    const pz = ic.velocities.reduce((s, v, i) => s + ic.masses[i] * v.z, 0);
    expect(Math.abs(pz)).toBeCloseTo(0, 1e-10);
  });

  it('zero net momentum holds for mixed tangential+radial config', () => {
    const ic = equilateralIC(0.8, 0.35, 3);
    const px = ic.velocities.reduce((s, v, i) => s + ic.masses[i] * v.x, 0);
    const pz = ic.velocities.reduce((s, v, i) => s + ic.masses[i] * v.z, 0);
    expect(Math.abs(px)).toBeCloseTo(0, 1e-10);
    expect(Math.abs(pz)).toBeCloseTo(0, 1e-10);
  });

  it('zero net momentum for various choreoScale values', () => {
    [1, 2, 3, 5, 8].forEach(s => {
      const ic = equilateralIC(1.0, 0.2, s);
      const px = ic.velocities.reduce((sum, v, i) => sum + ic.masses[i] * v.x, 0);
      const pz = ic.velocities.reduce((sum, v, i) => sum + ic.masses[i] * v.z, 0);
      expect(Math.abs(px)).toBeCloseTo(0, 1e-10);
      expect(Math.abs(pz)).toBeCloseTo(0, 1e-10);
    });
  });

  it('vt=0, vr=0 produces all-zero velocities', () => {
    const ic = equilateralIC(0, 0, 3);
    ic.velocities.forEach(v => {
      expect(v.x).toBeCloseTo(0, 1e-12);
      expect(v.z).toBeCloseTo(0, 1e-12);
    });
  });
});

describe('equilateralIC — Kepler scaling', () => {
  it('scaling choreoScale multiplies positions by s', () => {
    const ic1 = equilateralIC(1.0, 0, 1);
    const ic4 = equilateralIC(1.0, 0, 4);
    ic1.positions.forEach((p, i) => {
      expect(ic4.positions[i].x).toBeCloseTo(p.x * 4, 1e-8);
      expect(ic4.positions[i].z).toBeCloseTo(p.z * 4, 1e-8);
    });
  });

  it('scaling choreoScale divides velocities by sqrt(s)', () => {
    const ic1 = equilateralIC(1.0, 0, 1);
    const ic4 = equilateralIC(1.0, 0, 4);
    // 1/sqrt(4) = 0.5
    ic1.velocities.forEach((v, i) => {
      expect(ic4.velocities[i].x).toBeCloseTo(v.x * 0.5, 1e-8);
      expect(ic4.velocities[i].z).toBeCloseTo(v.z * 0.5, 1e-8);
    });
  });
});

// ── Trail ─────────────────────────────────────────────────────────────────────

describe('Trail — construction', () => {
  it('starts with head=0 and total=0', () => {
    const t = new Trail(0xff0000);
    expect(t.head).toBe(0);
    expect(t.total).toBe(0);
  });

  it('draw range is 0 after construction', () => {
    const t = new Trail(0xff0000);
    expect(t.geo._drawCount).toBe(0);
  });

  it('hist and buf are Float32Arrays of length MAX_HISTORY * 3', () => {
    const t = new Trail(0xff0000);
    expect(t.hist.length).toBe(MAX_HISTORY * 3);
    expect(t.buf.length).toBe(MAX_HISTORY * 3);
  });
});

describe('Trail.addPoint — basic writes', () => {
  it('writes x/y/z into the first three hist slots', () => {
    const t = new Trail(0xff0000);
    t.addPoint({ x: 1, y: 2, z: 3 });
    expect(t.hist[0]).toBe(1);
    expect(t.hist[1]).toBe(2);
    expect(t.hist[2]).toBe(3);
  });

  it('advances head to 1 after first addPoint', () => {
    const t = new Trail(0);
    t.addPoint({ x: 0, y: 0, z: 0 });
    expect(t.head).toBe(1);
  });

  it('increments total up to MAX_HISTORY then caps it', () => {
    const t = new Trail(0);
    // Write MAX_HISTORY + 5 points
    for (let i = 0; i < MAX_HISTORY + 5; i++) {
      t.addPoint({ x: i, y: 0, z: 0 });
    }
    expect(t.total).toBe(MAX_HISTORY);
  });

  it('head wraps back to 0 after MAX_HISTORY points', () => {
    const t = new Trail(0);
    for (let i = 0; i < MAX_HISTORY; i++) t.addPoint({ x: i, y: 0, z: 0 });
    expect(t.head).toBe(0);  // wrapped around
  });

  it('overwrites the oldest slot on wrap-around', () => {
    const t = new Trail(0);
    // Fill entire buffer
    for (let i = 0; i < MAX_HISTORY; i++) t.addPoint({ x: i, y: 0, z: 0 });
    // Now write one more — overwrites slot 0
    t.addPoint({ x: 99999, y: 0, z: 0 });
    expect(t.hist[0]).toBe(99999);   // slot 0 now has the newest point
    expect(t.head).toBe(1);
    expect(t.total).toBe(MAX_HISTORY);  // still capped
  });

  it('successive points occupy consecutive slots', () => {
    const t = new Trail(0);
    t.addPoint({ x: 10, y: 11, z: 12 });
    t.addPoint({ x: 20, y: 21, z: 22 });
    expect(t.hist[3]).toBe(20);
    expect(t.hist[4]).toBe(21);
    expect(t.hist[5]).toBe(22);
  });
});

describe('Trail.updateVisible — basic behaviour', () => {
  it('updateVisible(0) sets draw count to 0', () => {
    const t = new Trail(0);
    t.addPoint({ x: 1, y: 0, z: 0 });
    t.updateVisible(0);
    expect(t.geo._drawCount).toBe(0);
  });

  it('updateVisible(n) with n > total shows only total points', () => {
    const t = new Trail(0);
    t.addPoint({ x: 1, y: 0, z: 0 });
    t.addPoint({ x: 2, y: 0, z: 0 });
    t.updateVisible(1000);  // asking for more than available
    expect(t.geo._drawCount).toBe(2);
  });

  it('updateVisible(1) shows the most recent point in buf[0..2]', () => {
    const t = new Trail(0);
    t.addPoint({ x: 5, y: 6, z: 7 });
    t.addPoint({ x: 8, y: 9, z: 10 });
    t.updateVisible(1);
    // Most recent point is {8,9,10}
    expect(t.buf[0]).toBe(8);
    expect(t.buf[1]).toBe(9);
    expect(t.buf[2]).toBe(10);
    expect(t.geo._drawCount).toBe(1);
  });

  it('sets needsUpdate on the position attribute', () => {
    const t = new Trail(0);
    t.addPoint({ x: 1, y: 0, z: 0 });
    t.geo.attributes.position.needsUpdate = false;
    t.updateVisible(1);
    expect(t.geo.attributes.position.needsUpdate).toBeTruthy();
  });

  it('buf contains points in chronological order (oldest first)', () => {
    const t = new Trail(0);
    for (let i = 1; i <= 5; i++) t.addPoint({ x: i, y: 0, z: 0 });
    t.updateVisible(5);
    // Oldest to newest: x = 1, 2, 3, 4, 5
    for (let i = 0; i < 5; i++) {
      expect(t.buf[i * 3]).toBe(i + 1);
    }
  });
});

describe('Trail.updateVisible — ring-buffer wrap-around', () => {
  it('correctly assembles buf when the window straddles the ring boundary', () => {
    const t = new Trail(0);
    // Fill the buffer completely with x=index values
    for (let i = 0; i < MAX_HISTORY; i++) t.addPoint({ x: i, y: 0, z: 0 });
    // head is back at 0, total = MAX_HISTORY
    // Now write 3 more points so head=3, and the ring wraps
    t.addPoint({ x: 100, y: 0, z: 0 });  // slot 0
    t.addPoint({ x: 101, y: 0, z: 0 });  // slot 1
    t.addPoint({ x: 102, y: 0, z: 0 });  // slot 2
    // head = 3, total = MAX_HISTORY
    // Asking for last 5 points: oldest→newest = [MAX_HISTORY-2, MAX_HISTORY-1, 100, 101, 102]
    t.updateVisible(5);
    expect(t.geo._drawCount).toBe(5);
    // buf[0..2]: x = MAX_HISTORY - 2
    expect(t.buf[0]).toBe(MAX_HISTORY - 2);
    // buf[3..5]: x = MAX_HISTORY - 1
    expect(t.buf[3]).toBe(MAX_HISTORY - 1);
    // buf[6..8]: x = 100
    expect(t.buf[6]).toBe(100);
    // buf[9..11]: x = 101
    expect(t.buf[9]).toBe(101);
    // buf[12..14]: x = 102
    expect(t.buf[12]).toBe(102);
  });

  it('requesting MAX_HISTORY points on a full buffer produces correct draw count', () => {
    const t = new Trail(0);
    for (let i = 0; i < MAX_HISTORY; i++) t.addPoint({ x: i, y: 0, z: 0 });
    t.updateVisible(MAX_HISTORY);
    expect(t.geo._drawCount).toBe(MAX_HISTORY);
  });

  it('window entirely in the wrapped (second) segment is assembled correctly', () => {
    const t = new Trail(0);
    // Fill buffer, then overwrite the first 10 slots
    for (let i = 0; i < MAX_HISTORY; i++) t.addPoint({ x: i * 10, y: 0, z: 0 });
    for (let i = 0; i < 10; i++) t.addPoint({ x: 9000 + i, y: 0, z: 0 });
    // head=10. Show last 3 points (which are in slots 7,8,9 — all < head, no straddling)
    t.updateVisible(3);
    expect(t.geo._drawCount).toBe(3);
    expect(t.buf[0]).toBe(9007);
    expect(t.buf[3]).toBe(9008);
    expect(t.buf[6]).toBe(9009);
  });
});

describe('Trail.reset', () => {
  it('sets head and total back to 0', () => {
    const t = new Trail(0);
    for (let i = 0; i < 50; i++) t.addPoint({ x: i, y: 0, z: 0 });
    t.reset();
    expect(t.head).toBe(0);
    expect(t.total).toBe(0);
  });

  it('sets draw range to 0 after reset', () => {
    const t = new Trail(0);
    t.addPoint({ x: 1, y: 0, z: 0 });
    t.updateVisible(1);
    t.reset();
    expect(t.geo._drawCount).toBe(0);
  });

  it('can accept new points normally after reset', () => {
    const t = new Trail(0);
    for (let i = 0; i < 10; i++) t.addPoint({ x: i, y: 0, z: 0 });
    t.reset();
    t.addPoint({ x: 42, y: 0, z: 0 });
    expect(t.hist[0]).toBe(42);
    expect(t.head).toBe(1);
    expect(t.total).toBe(1);
  });
});

// Export suite for consumption by test.html
export { suite };
