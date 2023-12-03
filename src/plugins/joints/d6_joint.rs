use bevy::prelude::*;
use physx::traits::Class;



//todo add broken bool

//"core"
use derive_more::{Deref, DerefMut};
use physx_sys::PxD6Axis;
use crate::{core::scene::SceneRwLock, bpx::{RigidDynamicHandle, ArticulationLinkHandle, RigidStaticHandle, IntoPxTransform}};


#[derive(Component, Deref, DerefMut)]
pub struct D6JointHandle { //todo implent 6d joint in physx-rs
    #[deref]
    #[deref_mut]
    handle: SceneRwLock<*mut physx_sys::PxD6Joint>, //Owner<>
}
unsafe impl Send for D6JointHandle {}
unsafe impl Sync for D6JointHandle {}

//todo add owner to handle
impl D6JointHandle {
    pub fn new(px_d6_joint: *mut physx_sys::PxD6Joint) -> Self {
        Self { handle: SceneRwLock::new(px_d6_joint) }
    }
}


impl Drop for D6JointHandle {
    fn drop(&mut self) {
        unsafe { physx_sys::PxJoint_release_mut(self.handle.get_mut_unsafe().to_owned() as *mut physx_sys::PxJoint) };
        unsafe { std::ptr::drop_in_place(self.handle.get_mut_unsafe()) }
    }
}



// #[derive(Clone, Copy, Reflect)]
// pub enum PxD6Axis {
//     X,
//     Y,
//     Z,
//     Twist,
//     Swing1,
//     Swing2,
// }


#[derive(Default, Clone, Copy, Reflect)]
pub enum D6JointMotion {
    #[default]
    Locked,
    Free,
    Limited { min: f32, max: f32 },
}

impl D6JointMotion {
    pub fn into_physx_sys(self) -> physx_sys::PxD6Motion {
        match self {
            Self::Locked => physx_sys::PxD6Motion::Locked,
            Self::Free => physx_sys::PxD6Motion::Free,
            Self::Limited { min: _, max: _ } => physx_sys::PxD6Motion::Limited,
        }
    }
}



//"plugin"
#[derive(Component, Clone, Copy, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct D6Joint {
    pub actor0: Entity,
    pub local_frame0: Transform,
    pub actor1: Entity,
    pub local_frame1: Transform,

    pub motion_twist: D6JointMotion,
    pub motion_swing1: D6JointMotion,
    pub motion_swing2: D6JointMotion,
    pub motion_x: D6JointMotion,
    pub motion_y: D6JointMotion,
    pub motion_z: D6JointMotion,
}

impl Default for D6Joint {
    fn default() -> Self {
        Self {
            actor0: Entity::PLACEHOLDER,
            actor1: Entity::PLACEHOLDER,
            ..default()
        }
    }
}


impl D6Joint {
    pub fn new(actor0: Entity, actor1: Entity) -> Self {
        Self { actor0, actor1, ..default() }
    }

    pub fn with_local_frame0(mut self, local_frame0: Transform) -> Self {
        self.local_frame0 = local_frame0;
        self
    }

    pub fn with_local_frame1(mut self, local_frame1: Transform) -> Self {
        self.local_frame1 = local_frame1;
        self
    }

    pub fn with_axis_motion(mut self, axis: PxD6Axis, motion: D6JointMotion) -> Self {
        match axis {
            PxD6Axis::X => self.motion_x = motion,
            PxD6Axis::Y => self.motion_y = motion,
            PxD6Axis::Z => self.motion_z = motion,
            PxD6Axis::Twist => self.motion_twist = motion,
            PxD6Axis::Swing1 => self.motion_swing1 = motion,
            PxD6Axis::Swing2 => self.motion_swing2 = motion,
            PxD6Axis::Count => panic!("Invalid axis"),
        }
        self
    }
}




pub struct D6JointPlugin;

impl Plugin for D6JointPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<D6JointMotion>();
        app.register_type::<D6Joint>();



        // app.add_systems(PhysicsSchedule, (

        // ).in_set(PhysicsSet::Sync));
    }
}



pub fn create_d6_joint(
    mut commands: Commands,
    mut scene: ResMut<crate::prelude::Scene>,
    mut physics: ResMut<crate::prelude::Physics>,
    query: Query<(Entity, &D6Joint), Without<D6JointHandle>>,
    mut actor_handle_q: Query<(Option<&mut RigidStaticHandle>, Option<&mut RigidDynamicHandle>, Option<&mut ArticulationLinkHandle>)>,
) {

    for (entity, fixed_joint) in query.iter() {

        unsafe {

            //extract handle for actor0, if more than one handle is found, panic, if no handle is found, panic
            let actor0_handle = match actor_handle_q.get_mut(fixed_joint.actor0) {
                Ok((Some(mut rigid_static_handle), None, None)) => rigid_static_handle.handle.get_mut(&mut scene).as_mut_ptr() as *mut physx_sys::PxRigidActor,
                Ok((None, Some(mut rigid_dynamic_handle), None)) => rigid_dynamic_handle.handle.get_mut(&mut scene).as_mut_ptr() as *mut physx_sys::PxRigidActor,
                Ok((None, None, Some(mut articulation_link_handle))) => articulation_link_handle.handle.as_mut().unwrap().get_mut(&mut scene).as_mut_ptr() as *mut physx_sys::PxRigidActor,
                _ => panic!("Actor0 has no handle"),
            }; 

            //extract handle for actor1, if more than one handle is found, panic, if no handle is found, panic
            let actor1_handle = match actor_handle_q.get_mut(fixed_joint.actor1) {
                Ok((Some(mut rigid_static_handle), None, None)) => rigid_static_handle.handle.get_mut(&mut scene).as_mut_ptr() as *mut physx_sys::PxRigidActor,
                Ok((None, Some(mut rigid_dynamic_handle), None)) => rigid_dynamic_handle.handle.get_mut(&mut scene).as_mut_ptr() as *mut physx_sys::PxRigidActor,
                Ok((None, None, Some(mut articulation_link_handle))) => articulation_link_handle.handle.as_mut().unwrap().get_mut(&mut scene).as_mut_ptr() as *mut physx_sys::PxRigidActor,
                _ => panic!("Actor1 has no handle"),
            };

            let px_d6_joint = physx_sys::phys_PxD6JointCreate(
                physics.physics_mut().as_mut_ptr(), 
                actor0_handle, fixed_joint.local_frame0.to_physx_sys().as_mut_ptr(), 
                actor1_handle, fixed_joint.local_frame1.to_physx_sys().as_mut_ptr()
            );

            commands.entity(entity).insert(D6JointHandle::new(px_d6_joint));

        }


    }

}



pub fn sync_6d_joint(
    mut scene: ResMut<crate::prelude::Scene>,
    mut query: Query<(&D6Joint, &mut D6JointHandle), Changed<D6Joint>>,
) {

    for (d6_joint, mut d6_joint_handle) in query.iter_mut() {

        unsafe {

            //sync transform
            physx_sys::PxJoint_setLocalPose_mut(d6_joint_handle.handle.get_mut(&mut scene).to_owned() as *mut physx_sys::PxJoint, 
                physx_sys::PxJointActorIndex::Actor0, d6_joint.local_frame0.to_physx_sys().as_mut_ptr());

            physx_sys::PxJoint_setLocalPose_mut(d6_joint_handle.handle.get_mut(&mut scene).to_owned() as *mut physx_sys::PxJoint, 
                physx_sys::PxJointActorIndex::Actor1, d6_joint.local_frame1.to_physx_sys().as_mut_ptr());


            //sync motion
            physx_sys::PxD6Joint_setMotion_mut(d6_joint_handle.handle.get_mut(&mut scene).to_owned(),
                PxD6Axis::X, d6_joint.motion_x.into_physx_sys());
        
            physx_sys::PxD6Joint_setMotion_mut(d6_joint_handle.handle.get_mut(&mut scene).to_owned(),
                PxD6Axis::Y, d6_joint.motion_y.into_physx_sys());

            physx_sys::PxD6Joint_setMotion_mut(d6_joint_handle.handle.get_mut(&mut scene).to_owned(),
                PxD6Axis::Z, d6_joint.motion_z.into_physx_sys());

        }

    }

}