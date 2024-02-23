use bevy::prelude::*;
use bevy_mod_physx::prelude::*;
use bevy_mod_physx::prelude::Material; // bevy prelude conflicts

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins.set(
            PhysicsCore::new().with_pvd()
        ))
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Update, release_fixed_joint)
        .insert_resource(DebugRenderSettings::enable())
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-3.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}

fn setup_physics(
    mut commands: Commands,
    mut physics: ResMut<Physics>,
    mut geometries: ResMut<Assets<Geometry>>,
    mut materials: ResMut<Assets<Material>>,
) {

    // Create the ground.
    commands.spawn((
        RigidBody::Static,
        Shape {
            geometry: geometries.add(Geometry::halfspace(Vec3::Y)),
            material: materials.add(Material::new(&mut physics, 0.5, 0.5, 0.6)),
            ..default()
        },
        SpatialBundle::from_transform(Transform::from_xyz(0.0, -2.0, 0.0)),
    ));

    // Create the bouncing ball.
    let ball_0 = commands.spawn((
        RigidBody::Dynamic,
        Shape {
            geometry: geometries.add(Geometry::ball(0.5)),
            material: materials.add(Material::new(&mut physics, 0.5, 0.5, 0.6)),
            ..default()
        },
        SpatialBundle::from_transform(Transform::from_xyz(1.0, 4.0, 0.0)),
    )).id();

    // Create the bouncing ball.
    let ball_1 = commands.spawn((
        RigidBody::Dynamic,
        Shape {
            geometry: geometries.add(Geometry::ball(0.5)),
            material: materials.add(Material::new(&mut physics, 0.5, 0.5, 0.6)),
            ..default()
        },
        SpatialBundle::from_transform(Transform::from_xyz(0.0, 4.0, 0.0)),
    )).id();

    //create a fixed joint between the two balls
    commands.spawn(FixedJoint::new(ball_0, ball_1).with_local_frame0(Transform::from_xyz(0.0, 2.0, 0.0)));
}



fn release_fixed_joint(
    mut commands: Commands,
    input: Res<ButtonInput<KeyCode>>,
    mut fixed_joints: Query<Entity, With<FixedJoint>>,

) {

    if input.just_pressed(KeyCode::Space) {

        for fixed_joint in fixed_joints.iter_mut() {
            commands.entity(fixed_joint).despawn_recursive();
        }
    }

}