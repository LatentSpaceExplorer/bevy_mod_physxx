mod common;

use bevy::prelude::*;
use bevy_mod_physx::prelude::{self as bpx, *};
use physx::prelude::*;
use physx_sys::PxSolverType;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins.set(
            PhysicsCore {
                scene: bpx::SceneDescriptor {
                    solver_type: PxSolverType::Tgs,
                    ..default()
                },
                ..default()
            }.with_pvd()
        ))
        .add_plugins(common::DemoUtils) // optional
        .add_systems(Startup, (
            spawn_long_chain,
            spawn_obstacle,
            spawn_plane,
            spawn_camera_and_light,
        ))
        .run();
}

fn spawn_long_chain(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut px_geometries: ResMut<Assets<bpx::Geometry>>,
) {
    const SCALE: f32 = 0.25;
    const RADIUS: f32 = 0.5 * SCALE;
    const HALF_HEIGHT: f32 = 1. * SCALE;
    const SEGMENTS: usize = 40;
    const SEGMENT_MASS: f32 = 1.;

    // change this for another kind of rope
    const OVERLAPPING_LINKS: bool = true;

    let mut position = Vec3::new(0., 24., 0.);
    let mesh = meshes.add(Capsule3d { radius: RADIUS, half_length: HALF_HEIGHT + RADIUS * 2., ..default() });
    let material = materials.add(Color::rgb(1., 0.7, 0.));

    let px_geometry = px_geometries.add(bpx::Geometry::capsule(HALF_HEIGHT, RADIUS));
    let mut parent_link = None;

    for _ in 0..SEGMENTS {
        let mut builder = commands.spawn_empty();
        builder
            .insert(bpx::RigidBody::ArticulationLink)
            .insert(SpatialBundle::from_transform(Transform::from_translation(position)))
            .insert(Damping {
                linear: 0.1,
                angular: 0.1,
            })
            .insert(MaxVelocity {
                linear: 100.,
                angular: 30.,
            })
            .insert(bpx::Shape {
                geometry: px_geometry.clone(),
                ..default()
            })
            .insert(MassProperties::mass(SEGMENT_MASS))
            .with_children(|builder| {
                builder.spawn(PbrBundle {
                    mesh: mesh.clone(),
                    material: material.clone(),
                    transform: Transform::from_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
                    ..default()
                });
            });

        if let Some(parent_link) = parent_link {
            let parent_pose;
            let child_pose;

            if OVERLAPPING_LINKS {
                parent_pose = Transform::from_xyz(HALF_HEIGHT, 0., 0.);
                child_pose = Transform::from_xyz(-HALF_HEIGHT, 0., 0.);
            } else {
                parent_pose = Transform::from_xyz(RADIUS + HALF_HEIGHT, 0., 0.);
                child_pose = Transform::from_xyz(-RADIUS - HALF_HEIGHT, 0., 0.);
            }

            // need to configure inbound joint
            builder.insert(ArticulationJoint {
                parent: parent_link,
                parent_pose,
                child_pose,
                joint_type: ArticulationJointType::Spherical,
                motion_swing1: ArticulationJointMotion::Free,
                motion_swing2: ArticulationJointMotion::Free,
                motion_twist: ArticulationJointMotion::Free,
                friction_coefficient: 1.,
                ..default()
            });
        } else {
            // articulation root
            builder.insert(ArticulationRoot {
                fix_base: true,
                ..default()
            });
        }

        let id = builder.id();

        if OVERLAPPING_LINKS {
            position.x += RADIUS + HALF_HEIGHT * 2.;
        } else {
            position.x += (RADIUS + HALF_HEIGHT) * 2.;
        }

        parent_link = Some(id);
    }

    // attach large heavy box at the end of the rope
    const BOX_SIZE: f32 = 1.;
    const BOX_MASS: f32 = 50.;

    position.x -= (RADIUS + HALF_HEIGHT) * 2.;
    position.x += (RADIUS + HALF_HEIGHT) + BOX_SIZE;

    let box_mesh = meshes.add(Cuboid { half_size: Vec3::splat(BOX_SIZE) });
    let box_material = materials.add(Color::rgb(0.8, 0.7, 0.6));
    let box_geometry = px_geometries.add(bpx::Geometry::cuboid(BOX_SIZE, BOX_SIZE, BOX_SIZE));

    commands.spawn_empty()
        .insert(bpx::RigidBody::ArticulationLink)
        .insert(PbrBundle {
            mesh: box_mesh,
            material: box_material,
            transform: Transform::from_translation(position),
            ..default()
        })
        .insert(Damping {
            linear: 0.1,
            angular: 0.1,
        })
        .insert(MaxVelocity {
            linear: 100.,
            angular: 30.,
        })
        .insert(bpx::Shape {
            geometry: box_geometry,
            ..default()
        })
        .insert(MassProperties::mass(BOX_MASS))
        .insert(ArticulationJoint {
            parent: parent_link.unwrap(),
            parent_pose: Transform::from_xyz(RADIUS + HALF_HEIGHT, 0., 0.),
            child_pose: Transform::from_xyz(-BOX_SIZE, 0., 0.),
            joint_type: ArticulationJointType::Spherical,
            motion_swing1: ArticulationJointMotion::Free,
            motion_swing2: ArticulationJointMotion::Free,
            motion_twist: ArticulationJointMotion::Free,
            friction_coefficient: 1.,
            ..default()
        });
}

fn spawn_obstacle(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut physics: ResMut<bpx::Physics>,
    mut px_geometries: ResMut<Assets<bpx::Geometry>>,
    mut px_materials: ResMut<Assets<bpx::Material>>,
) {
    const HALF_X: f32 = 1.;
    const HALF_Y: f32 = 0.1;
    const HALF_Z: f32 = 2.;

    let mesh = meshes.add(
        Cuboid { half_size : Vec3::new(HALF_X, HALF_Y, HALF_Z) }
    );
    let material = materials.add(Color::rgb(0.8, 0.7, 0.6));

    let px_geometry = px_geometries.add(bpx::Geometry::cuboid(HALF_X, HALF_Y, HALF_Z));
    let px_material = px_materials.add(bpx::Material::new(&mut physics, 0.5, 0.5, 0.6));
    let transform = Transform::from_xyz(10., 21., 0.);

    commands.spawn_empty()
        .insert(PbrBundle {
            mesh,
            material,
            transform,
            ..default()
        })
        .insert(bpx::RigidBody::Static)
        .insert(bpx::Shape {
            material: px_material,
            geometry: px_geometry,
            ..default()
        })
        .insert(Name::new("Obstacle"));
}

fn spawn_plane(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut physics: ResMut<bpx::Physics>,
    mut px_geometries: ResMut<Assets<bpx::Geometry>>,
    mut px_materials: ResMut<Assets<bpx::Material>>,
) {
    let mesh = meshes.add(Plane3d::default().mesh().size( 500.0, 500.0 ));
    let material = materials.add(Color::rgb(0.3, 0.5, 0.3));
    let px_geometry = px_geometries.add(bpx::Geometry::halfspace(Vec3::Y));
    let px_material = px_materials.add(bpx::Material::new(&mut physics, 0.5, 0.5, 0.6));

    commands.spawn_empty()
        .insert(PbrBundle {
            mesh,
            material,
            ..default()
        })
        .insert(bpx::RigidBody::Static)
        .insert(bpx::Shape {
            geometry: px_geometry,
            material: px_material,
            ..default()
        })
        .insert(Name::new("Plane"));
}

fn spawn_camera_and_light(mut commands: Commands) {
    commands
        .spawn(SpatialBundle::from_transform(Transform::from_xyz(10., 17., 0.)))
        .with_children(|builder| {
            builder.spawn(Camera3dBundle {
                transform: Transform::from_xyz(24.5, 17.3, 26.4).looking_at(Vec3::ZERO, Vec3::Y),
                ..default()
            });
        })
        .insert(Name::new("Camera"));

    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.2, -0.2, 0.)),
        ..default()
    }).insert(Name::new("Light"));
}
