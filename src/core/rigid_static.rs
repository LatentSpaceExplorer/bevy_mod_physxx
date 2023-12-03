//! Static rigid simulation object in the Physics SDK.
use bevy::prelude::*;
use derive_more::{Deref, DerefMut};
use physx::prelude::*;

use crate::core::scene::SceneRwLock;
use crate::types::*;

#[derive(Component, Deref, DerefMut)]
pub struct RigidStaticHandle {
    #[deref]
    #[deref_mut]
    pub handle: SceneRwLock<Owner<PxRigidStatic>>,
    // used for change detection
    pub predicted_gxform: GlobalTransform,
}

impl RigidStaticHandle {
    pub fn new(px_rigid_static: Owner<PxRigidStatic>, predicted_gxform: GlobalTransform) -> Self {
        Self { handle: SceneRwLock::new(px_rigid_static), predicted_gxform }
    }
}
