use std::collections::VecDeque;

use bevy::prelude::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use pendulum3d::physics::PendulumState;

#[derive(Component)]
struct MassB;
#[derive(Component)]
struct MassC;

const THETA1: f32 = 2.4912903;
const THETA2: f32 = -2.743706;
const TRAIL_MAX_LEN: usize = 750;

/// Breadcrumb trail of bob C's (mass 2's) position, used to draw the swept
/// path it traces out over time.
#[derive(Resource, Default)]
struct Trail(VecDeque<Vec3>);

/// Diagnostic: logs total energy periodically so drift from the integrator
/// can be distinguished from the system's inherent chaos.
#[derive(Resource)]
struct EnergyLog {
    timer: Timer,
    initial: Option<f32>,
}

impl Default for EnergyLog {
    fn default() -> Self {
        EnergyLog {
            timer: Timer::from_seconds(1.0, TimerMode::Repeating),
            initial: None,
        }
    }
}

fn main() {
    println!("Starting from theta1={THETA1:.6} theta2={THETA2:.6}");

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PanOrbitCameraPlugin)
        .insert_resource(ClearColor(Color::WHITE))
        .insert_resource(PendulumState::initial_planar(THETA1, THETA2))
        .insert_resource(Trail::default())
        .insert_resource(EnergyLog::default())
        .insert_resource(Time::<Fixed>::from_seconds(1.0 / 240.0))
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, (step_physics, record_trail))
        .add_systems(
            Update,
            (
                sync_transforms,
                draw_rods,
                draw_inertial_axes,
                draw_trail,
                log_energy,
            ),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // The pendulum swings in the physics x-z plane (y = 0), which `to_render`
    // maps to the render-space x-y plane. Start the camera on the render-Z
    // axis (the plane's normal) so the motion is viewed face-on, not edge-on.
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, -0.5, 8.0)
                .looking_at(Vec3::new(0.0, -0.5, 0.0), Vec3::Y),
            ..default()
        },
        PanOrbitCamera::default(),
    ));

    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 2_000_000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(3.0, 4.0, 3.0),
        ..default()
    });

    // pivot marker
    commands.spawn(PbrBundle {
        mesh: meshes.add(Sphere::new(0.03)),
        material: materials.add(Color::srgb(0.15, 0.15, 0.15)),
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..default()
    });

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Sphere::new(0.06)),
            material: materials.add(Color::srgb(0.9, 0.2, 0.2)),
            ..default()
        },
        MassB,
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Sphere::new(0.06)),
            material: materials.add(Color::srgb(0.2, 0.4, 0.9)),
            ..default()
        },
        MassC,
    ));
}

fn step_physics(mut state: ResMut<PendulumState>, time: Res<Time<Fixed>>) {
    let dt = time.delta_seconds();
    state.step(dt);
}

/// Physics uses gravity along -Z, but Bevy's "up" is +Y. Swap Y/Z so gravity
/// renders as pointing straight down on screen.
fn to_render(v: Vec3) -> Vec3 {
    Vec3::new(v.x, v.z, v.y)
}

fn sync_transforms(
    state: Res<PendulumState>,
    mut b_q: Query<&mut Transform, (With<MassB>, Without<MassC>)>,
    mut c_q: Query<&mut Transform, (With<MassC>, Without<MassB>)>,
) {
    if let Ok(mut t) = b_q.get_single_mut() {
        t.translation = to_render(state.r_b);
    }
    if let Ok(mut t) = c_q.get_single_mut() {
        t.translation = to_render(state.r_c);
    }
}

fn draw_rods(mut gizmos: Gizmos, state: Res<PendulumState>) {
    let rod_color = Color::srgb(0.15, 0.15, 0.15);
    gizmos.line(Vec3::ZERO, to_render(state.r_b), rod_color);
    gizmos.line(to_render(state.r_b), to_render(state.r_c), rod_color);
}

fn log_energy(state: Res<PendulumState>, time: Res<Time>, mut log: ResMut<EnergyLog>) {
    log.timer.tick(time.delta());
    if !log.timer.just_finished() {
        return;
    }
    let e = state.energy();
    let initial = *log.initial.get_or_insert(e);
    let drift_pct = (e - initial) / initial.abs() * 100.0;
    info!("energy = {e:.6}  drift = {drift_pct:+.4}% from start");
}

fn record_trail(state: Res<PendulumState>, mut trail: ResMut<Trail>) {
    trail.0.push_back(state.r_c);
    if trail.0.len() > TRAIL_MAX_LEN {
        trail.0.pop_front();
    }
}

fn draw_trail(mut gizmos: Gizmos, trail: Res<Trail>) {
    if trail.0.len() < 2 {
        return;
    }
    let points = trail.0.iter().map(|p| to_render(*p));
    gizmos.linestrip(points, Color::srgb(0.1, 0.7, 0.3));
}

/// Fixed-length arrows at the pivot marking the inertial (world) coordinate
/// axes. Length never changes with the simulation state.
fn draw_inertial_axes(mut gizmos: Gizmos) {
    const AXIS_LENGTH: f32 = 1.0;
    gizmos.arrow(Vec3::ZERO, Vec3::X * AXIS_LENGTH, Color::srgb(0.9, 0.1, 0.1));
    gizmos.arrow(Vec3::ZERO, Vec3::Y * AXIS_LENGTH, Color::srgb(0.1, 0.6, 0.1));
    gizmos.arrow(Vec3::ZERO, Vec3::Z * AXIS_LENGTH, Color::srgb(0.1, 0.1, 0.9));
}
