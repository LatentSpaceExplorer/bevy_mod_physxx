//! Systems performing actor creation, simulation and transform sync.
use bevy::prelude::*;
use physx::scene::Scene;

mod create_actors;
pub use create_actors::*;

mod sync_transforms;
pub use sync_transforms::*;

pub fn scene_simulate(
    mut scene: ResMut<crate::prelude::Scene>,
    time: Res<crate::prelude::PhysicsTime>,
) {
    if time.delta_seconds() <= 0.0 { return; } //fixed bug with time.delta_seconds = 0.0 in debug mode
    let mut scene = scene.get_mut();
    scene.simulate(time.delta_seconds(), None, None);
    scene.fetch_results(true).unwrap();
}
