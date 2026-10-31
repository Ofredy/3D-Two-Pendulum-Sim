use bevy::math::Vec3;
use bevy::prelude::Resource;

pub const L1: f32 = 1.0;
pub const L2: f32 = 1.0;
pub const M1: f32 = 1.0;
pub const M2: f32 = 1.0;
pub const GRAVITY: Vec3 = Vec3::new(0.0, 0.0, -9.81);

/// 12-number state: positions and velocities of B and C, measured from the
/// fixed pivot A at the origin. This is the only thing that gets integrated;
/// lam1/lam2 are algebraic and recomputed on every derivative evaluation.
#[derive(Resource, Clone, Copy, Debug)]
pub struct PendulumState {
    pub r_b: Vec3,
    pub v_b: Vec3,
    pub r_c: Vec3,
    pub v_c: Vec3,
}

/// Same shape as `PendulumState`, used as the RK4 derivative/combination type.
#[derive(Clone, Copy)]
struct Deriv {
    r_b: Vec3,
    v_b: Vec3,
    r_c: Vec3,
    v_c: Vec3,
}

impl Deriv {
    fn scale(self, s: f32) -> Self {
        Deriv {
            r_b: self.r_b * s,
            v_b: self.v_b * s,
            r_c: self.r_c * s,
            v_c: self.v_c * s,
        }
    }

    fn add(self, other: Self) -> Self {
        Deriv {
            r_b: self.r_b + other.r_b,
            v_b: self.v_b + other.v_b,
            r_c: self.r_c + other.r_c,
            v_c: self.v_c + other.v_c,
        }
    }
}

/// Solves the 2x2 Lagrange-multiplier system from the EOM derivation and
/// returns (aB, aC, lam1, lam2).
fn accel(state: &PendulumState) -> (Vec3, Vec3, f32, f32) {
    let r_b = state.r_b;
    let v_b = state.v_b;
    let d = state.r_c - state.r_b;
    let v_d = state.v_c - state.v_b;

    let rb_dot_d = r_b.dot(d);

    // M * [lam1, lam2]^T = b
    let m00 = L1 * L1 / M1;
    let m01 = -rb_dot_d / M1;
    let m10 = m01;
    let m11 = L2 * L2 * (1.0 / M1 + 1.0 / M2);

    let b0 = -v_b.dot(v_b) - r_b.dot(GRAVITY);
    let b1 = -v_d.dot(v_d);

    let det = m00 * m11 - m01 * m10;
    let (lam1, lam2) = if det.abs() > 1e-9 {
        ((b0 * m11 - m01 * b1) / det, (m00 * b1 - b0 * m10) / det)
    } else {
        (0.0, 0.0)
    };

    let a_b = GRAVITY + (r_b * lam1 - d * lam2) / M1;
    let a_c = GRAVITY + (d * lam2) / M2;

    (a_b, a_c, lam1, lam2)
}

fn derivative(state: &PendulumState) -> Deriv {
    let (a_b, a_c, _, _) = accel(state);
    Deriv {
        r_b: state.v_b,
        v_b: a_b,
        r_c: state.v_c,
        v_c: a_c,
    }
}

fn apply_deriv(state: &PendulumState, d: &Deriv, dt: f32) -> PendulumState {
    PendulumState {
        r_b: state.r_b + d.r_b * dt,
        v_b: state.v_b + d.v_b * dt,
        r_c: state.r_c + d.r_c * dt,
        v_c: state.v_c + d.v_c * dt,
    }
}

impl PendulumState {
    /// Planar double-pendulum initial condition: both rods start in the
    /// x-z plane (y = 0) and released from rest, so the motion stays
    /// confined to that plane for all time (gravity has no y-component,
    /// and the EOM never introduces one). Angles are measured from
    /// straight-down (-Z), the classic double-pendulum convention.
    pub fn initial_planar(theta1: f32, theta2: f32) -> Self {
        let r_b = Vec3::new(theta1.sin(), 0.0, -theta1.cos()) * L1;
        let r_c = r_b + Vec3::new(theta2.sin(), 0.0, -theta2.cos()) * L2;

        PendulumState {
            r_b,
            v_b: Vec3::ZERO,
            r_c,
            v_c: Vec3::ZERO,
        }
    }

    /// Total mechanical energy (KE + PE), for checking integrator health.
    /// If this drifts noticeably over time, the integrator/drift-correction
    /// is adding real numerical error on top of the system's inherent chaos.
    pub fn energy(&self) -> f32 {
        let ke = 0.5 * M1 * self.v_b.dot(self.v_b) + 0.5 * M2 * self.v_c.dot(self.v_c);
        // U = -m * GRAVITY.dot(r), consistent with F = m*GRAVITY = -grad(U).
        let pe = -M1 * GRAVITY.dot(self.r_b) - M2 * GRAVITY.dot(self.r_c);
        ke + pe
    }

    /// Classic RK4 step. `accel` (and therefore lam1/lam2) is recomputed on
    /// each of the four stage evaluations, never integrated directly.
    pub fn rk4_step(&self, dt: f32) -> Self {
        let k1 = derivative(self);
        let s2 = apply_deriv(self, &k1, dt * 0.5);
        let k2 = derivative(&s2);
        let s3 = apply_deriv(self, &k2, dt * 0.5);
        let k3 = derivative(&s3);
        let s4 = apply_deriv(self, &k3, dt);
        let k4 = derivative(&s4);

        let combined = k1
            .add(k2.scale(2.0))
            .add(k3.scale(2.0))
            .add(k4)
            .scale(1.0 / 6.0);

        apply_deriv(self, &combined, dt)
    }

    /// Rescales both rods back to their nominal lengths and removes the
    /// velocity component along each rod, correcting numerical drift.
    pub fn apply_drift_correction(&mut self) {
        self.r_b = self.r_b.normalize() * L1;
        let u1 = self.r_b.normalize();
        self.v_b -= u1 * self.v_b.dot(u1);

        let mut d = self.r_c - self.r_b;
        d = d.normalize() * L2;
        self.r_c = self.r_b + d;

        let u2 = d.normalize();
        let mut v_d = self.v_c - self.v_b;
        v_d -= u2 * v_d.dot(u2);
        self.v_c = self.v_b + v_d;
    }

    pub fn step(&mut self, dt: f32) {
        *self = self.rk4_step(dt);
        self.apply_drift_correction();
    }
}
