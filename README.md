# 3D-Two-Pendulum-Sim

A double pendulum simulated and rendered in 3D with [Bevy](https://bevyengine.org/), where both rods are free to swing out of the plane instead of being confined to 2D. The project has two binaries: an interactive viewer and a headless-ish recorder that exports the simulation straight to an mp4.

![Heart traced by the pendulum's trail](media/heart_1727.png)

That heart shape isn't drawn or scripted — it's the trail traced by bob C's actual trajectory for a particular choice of initial angles (see [Why the trail draws a heart](#why-the-trail-draws-a-heart) below).

## The math

The system is two point masses, B and C, connected by rigid massless rods: a pivot A (fixed at the origin) to B (length `L1`), and B to C (length `L2`). Instead of the usual angle-based double-pendulum formulation, the state is tracked directly in Cartesian coordinates with the rod lengths enforced as constraints:

- **State** (`PendulumState` in `src/physics.rs`): position and velocity of B and C, `(r_b, v_b, r_c, v_c)` — 12 numbers total, all measured from the fixed pivot.
- **Constraints**: `|r_b| = L1` and `|r_c - r_b| = L2`, i.e. both rods stay rigid.

Rigid rods are enforced as constraint forces along each rod, scaled by Lagrange multipliers `lam1` and `lam2`. Differentiating both constraints twice (position → velocity → acceleration) turns them into a small linear system:

```
M * [lam1, lam2]^T = b
```

where `M` is a 2×2 matrix built from the current rod geometry and `b` comes from the current velocities and gravity. Solving that system at every evaluation gives the instantaneous constraint forces, and from there the accelerations:

```
a_b = gravity + (r_b * lam1 - d * lam2) / M1
a_c = gravity + (d * lam2) / M2       where d = r_c - r_b
```

This is solved fresh on every derivative evaluation — the multipliers are never integrated directly, only recomputed from the current state (see `accel()` in `src/physics.rs`).

**Integration**: classic 4th-order Runge-Kutta (`rk4_step`), which calls the derivative function four times per step and combines them with the usual `1/6 (k1 + 2k2 + 2k3 + k4)` weighting.

**Drift correction**: because of accumulated floating-point error, the RK4 state alone will slowly stretch the rods and pick up small velocity components along their length. After every step, `apply_drift_correction` renormalizes both rods back to their nominal lengths and projects out the velocity component along each rod, so the constraints stay (numerically) exactly satisfied.

**Energy diagnostic**: total mechanical energy (`energy()` = kinetic + potential) is logged once a second as a sanity check — in a correct integrator this should stay roughly flat; steady drift would mean the integrator/correction step is adding real error on top of the system's inherent chaos.

## Why the trail draws a heart

The double pendulum is chaotic — almost any initial condition quickly turns into an unpredictable tangle. But for one specific pair of starting angles (`theta1 = 2.4912903`, `theta2 = -2.743706`, both measured from straight-down), the early part of the trajectory happens to sweep bob C's path into a heart-like curve before the motion diverges into its usual chaos. Those angles are hardcoded as `THETA1`/`THETA2` in both `src/main.rs` and `src/bin/record.rs`. `media/heart_1727.png` is frame 1727 of a recording (≈28.8s in, at 60fps) — the point where the heart shape is most fully traced out.

## Binaries

- **`pendulum3d`** (`src/main.rs`) — interactive 3D viewer. Orbit the camera with the mouse (via `bevy_panorbit_camera`), watch the pendulum swing from the fixed initial angles, and see the swept trail and energy-drift log in the console.

  ```
  cargo run --release --bin pendulum3d
  ```

- **`record`** (`src/bin/record.rs`) — exports the same simulation to video instead of just displaying it. Physics is integrated at a fixed 240Hz (matching the interactive viewer) independent of video frame rate, so the output is deterministic regardless of how fast this machine can render and screenshot each frame. It captures one PNG per video frame (default 60fps) to `media/frames/`, then stitches them into `media/pendulum.mp4` with `ffmpeg` (must be on `PATH`).

  ```
  cargo run --release --bin record [seconds]   # default 45s
  ```

  `media/frames/` and `media/*.mp4` are gitignored — they're regenerated output, not source.

## Recording

🔗 _Google Drive link to a rendered recording: TODO_
