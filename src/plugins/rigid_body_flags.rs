use bevy::prelude::*;
use physx::traits::Class;
use physx_sys::PxRigidBodyFlags;
use crate::prelude::{Scene, *};


#[derive(Component, Debug, PartialEq, Clone, Copy, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
/// Set lock flags for a dynamic rigid body.
pub struct RigidBodyFlags {
    pub flags: PxRigidBodyFlags
}

impl RigidBodyFlags {
    pub fn new() -> Self {
        Self {
            flags: PxRigidBodyFlags::empty()
        }
    }

    pub fn from_flags(flags: PxRigidBodyFlags) -> Self {
        Self {
            flags
        }
    }

    pub fn set_flag(mut self, flag: PxRigidBodyFlags, value: bool) -> Self {
        self.flags.set(flag, value);
        self
    }

    pub fn set_kinematic(mut self, value: bool) -> Self {
        self.flags.set(PxRigidBodyFlags::Kinematic, value);
        self
    }

    pub fn set_enable_ccd(mut self, value: bool) -> Self {
        self.flags.set(PxRigidBodyFlags::EnableCcd, value);
        self
    }

}

pub struct RigidBodyFlagsPlugin;

impl Plugin for RigidBodyFlagsPlugin {
    fn build(&self, app: &mut App) {
        // app.register_type::<RigidBodyFlags>();
        app.add_systems(PhysicsSchedule, rigid_body_flags_sync.in_set(PhysicsSet::Sync));
    }
}

 
pub fn rigid_body_flags_sync(
    mut scene: ResMut<Scene>,
    mut actors: Query<
        (
            Option<&mut RigidDynamicHandle>,
            Option<&mut ArticulationLinkHandle>,
            Ref<RigidBodyFlags>,
        ),
        Or<(
            Added<RigidDynamicHandle>,
            Added<ArticulationLinkHandle>,
            Changed<RigidBodyFlags>,
        )>,
    >,
    ) {
    // this function only applies user defined properties,
    // there's nothing to get back from physx engine
    for (dynamic, articulation, rb_flags) in actors.iter_mut() {
        let actor_handle = if let Some(mut actor) = dynamic {
            actor.get_mut(&mut scene).as_mut_ptr()
        } else if let Some(mut actor) = articulation {
            actor.get_mut(&mut scene).as_mut_ptr()
        } else {
            if !rb_flags.is_added() {
                bevy::log::warn!("MassProperties component exists, but it's neither a rigid dynamic nor articulation link");
            }
            continue;
        };
        unsafe {
            physx_sys::PxRigidBody_setRigidBodyFlags_mut(actor_handle, rb_flags.flags);
        }

    } 
}