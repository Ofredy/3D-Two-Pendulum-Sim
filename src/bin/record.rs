//! Standalone recorder: runs the same 3D pendulum render loop as `pendulum3d`,
//! dumps one PNG per frame to `media/frames/`, then stitches them into
//! `media/pendulum.mp4` with ffmpeg. Decoupled from the interactive binary so
//! recording never affects `main.rs`.
//!
//! Usage: `cargo run --release --bin record [seconds]` (default 45s).
//! Requires `ffmpeg` on PATH.

use std::collections::VecDeque;
use std::env;
use std::process::Command;

use bevy::app::AppExit;
use bevy::prelude::*;
use bevy::render::view::screenshot::ScreenshotManager;
use bevy::window::PrimaryWindow;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use pendulum3d::physics::PendulumState;

#[derive(Component)]
struct MassB;
#[derive(Component)]
struct MassC;

const THETA1: f32 = 2.4912903;
const THETA2: f32 = -2.743706;
// Physics integrates at the same rate main.rs uses, independent of video fps.
const PHYSICS_HZ: u32 = 240;
const FPS: u32 = 60;
const STEPS_PER_FRAME: u32 = PHYSICS_HZ / FPS;
const FRAMES_DIR: &str = "media/frames";
const OUTPUT_VIDEO: &str = "media/pendulum.mp4";
const TRAIL_MAX_LEN: usize = 750;

#[derive(Resource)]
struct Recorder {
    next_frame: u32,
    total_frames: u32,
}

/// Breadcrumb trail of bob C's (mass 2's) position, used to draw the swept
/// path it traces out over time.
#[derive(Resource, Default)]
struct Trail(VecDeque<Vec3>);

fn main() {
    let seconds: f32 = env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(45.0);
    let total_frames = (seconds * FPS as f32).round() as u32;

    std::fs::remove_dir_all(FRAMES_DIR).ok();
    std::fs::create_dir_all(FRAMES_DIR).expect("create frames dir");

    println!(
        "Recording {total_frames} frames ({seconds:.1}s at {FPS}fps) from theta1={THETA1:.6} theta2={THETA2:.6}"
    );

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                resolution: (1920.0_f32, 1080.0_f32).into(),
                title: "pendulum3d recorder".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(PanOrbitCameraPlugin)
        .insert_resource(ClearColor(Color::WHITE))
        .insert_resource(PendulumState::initial_planar(THETA1, THETA2))
        .insert_resource(Recorder {
            next_frame: 0,
            total_frames,
        })
        .insert_resource(Trail::default())
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                step_physics,
                sync_transforms,
                draw_rods,
                draw_inertial_axes,
                draw_trail,
            )
                .chain(),
        )
        .add_systems(PostUpdate, capture_frame)
        .run();

    stitch_video();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
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

/// Advances the simulation by exactly one video frame's worth of physics
/// time (`STEPS_PER_FRAME` steps at `1/PHYSICS_HZ` each), independent of
/// real wall-clock speed, so output duration is deterministic regardless of
/// how fast this machine can render+screenshot each frame.
fn step_physics(mut state: ResMut<PendulumState>, mut trail: ResMut<Trail>) {
    let dt = 1.0 / PHYSICS_HZ as f32;
    for _ in 0..STEPS_PER_FRAME {
        state.step(dt);
        trail.0.push_back(state.r_c);
        if trail.0.len() > TRAIL_MAX_LEN {
            trail.0.pop_front();
        }
    }
}

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

fn draw_inertial_axes(mut gizmos: Gizmos) {
    const AXIS_LENGTH: f32 = 1.0;
    gizmos.arrow(Vec3::ZERO, Vec3::X * AXIS_LENGTH, Color::srgb(0.9, 0.1, 0.1));
    gizmos.arrow(Vec3::ZERO, Vec3::Y * AXIS_LENGTH, Color::srgb(0.1, 0.6, 0.1));
    gizmos.arrow(Vec3::ZERO, Vec3::Z * AXIS_LENGTH, Color::srgb(0.1, 0.1, 0.9));
}

fn draw_trail(mut gizmos: Gizmos, trail: Res<Trail>) {
    if trail.0.len() < 2 {
        return;
    }
    let points = trail.0.iter().map(|p| to_render(*p));
    gizmos.linestrip(points, Color::srgb(0.1, 0.7, 0.3));
}

/// Grabs one screenshot per rendered frame until `total_frames` is reached,
/// then exits the app so `stitch_video` can run.
fn capture_frame(
    mut recorder: ResMut<Recorder>,
    main_window: Query<Entity, With<PrimaryWindow>>,
    mut screenshot_manager: ResMut<ScreenshotManager>,
    mut exit: EventWriter<AppExit>,
) {
    if recorder.next_frame >= recorder.total_frames {
        exit.send(AppExit::Success);
        return;
    }
    let path = format!("{FRAMES_DIR}/frame_{:05}.png", recorder.next_frame);
    if screenshot_manager
        .save_screenshot_to_disk(main_window.single(), path)
        .is_ok()
    {
        recorder.next_frame += 1;
    }
}

fn stitch_video() {
    println!("Encoding {FRAMES_DIR}/*.png -> {OUTPUT_VIDEO} with ffmpeg...");
    let status = Command::new("ffmpeg")
        .args([
            "-y",
            "-framerate",
            &FPS.to_string(),
            "-i",
            &format!("{FRAMES_DIR}/frame_%05d.png"),
            "-c:v",
            "libx264",
            "-preset",
            "slow",
            "-crf",
            "16",
            "-pix_fmt",
            "yuv420p",
            OUTPUT_VIDEO,
        ])
        .status();

    match status {
        Ok(s) if s.success() => println!("Wrote {OUTPUT_VIDEO}"),
        Ok(s) => eprintln!("ffmpeg exited with status {s}"),
        Err(e) => eprintln!("failed to run ffmpeg (is it installed and on PATH?): {e}"),
    }
}
