use bevy::prelude::*;
use physx::traits::Class;



//todo add broken bool

//"core"
use derive_more::{Deref, DerefMut};
use crate::{core::scene::SceneRwLock, bpx::{RigidDynamicHandle, ArticulationLinkHandle, RigidStaticHandle, IntoPxTransform}};


#[derive(Component, Deref, DerefMut)]
pub struct FixedJointHandle { //todo implent fixed joint in physx-rs
    #[deref]
    #[deref_mut]
    handle: SceneRwLock<*mut physx_sys::PxFixedJoint>, //Owner<>
}
unsafe impl Send for FixedJointHandle {}
unsafe impl Sync for FixedJointHandle {}

//todo add owner to handle
impl FixedJointHandle {
    pub fn new(px_fixed_joint: *mut physx_sys::PxFixedJoint) -> Self {
        Self { handle: SceneRwLock::new(px_fixed_joint) }
    }
}


impl Drop for FixedJointHandle {
    fn drop(&mut self) {

        // unsafe { //wake up actors to be able to delete them
            //create actor0 and actor1 null ptrs
            // let mut actor0 = std::ptr::null_mut();
            // let mut actor1 = std::ptr::null_mut();

            // physx_sys::PxJoint_getActors(self.handle.get_mut_unsafe().to_owned() as *mut physx_sys::PxJoint, &mut actor0, &mut actor1);

            //wake up actors
            // physx_sys::PxRigidDynamic_wakeUp_mut(actor0 as *mut physx_sys::PxRigidDynamic);
            // physx_sys::PxRigidDynamic_wakeUp_mut(actor1 as *mut physx_sys::PxRigidDynamic);

        // }

        unsafe { physx_sys::PxJoint_release_mut( self.handle.get_mut_unsafe().to_owned() as *mut physx_sys::PxJoint) };
        unsafe { std::ptr::drop_in_place(self.handle.get_mut_unsafe()) }
    }
}


//"plugin"
#[derive(Component, Debug, PartialEq, Clone, Copy, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct FixedJoint {
    pub actor0: Entity,
    pub local_frame0: Transform,
    pub actor1: Entity,
    pub local_frame1: Transform,

    pub break_force: f32,
    pub break_torque: f32,
}

impl Default for FixedJoint {
    fn default() -> Self {
        Self { 
            actor0: Entity::PLACEHOLDER, 
            local_frame0: Transform::default(), 
            actor1: Entity::PLACEHOLDER, 
            local_frame1: Transform::default(), 
            break_force: core::f32::MAX, 
            break_torque: core::f32::MAX 
        }
    }
}

impl FixedJoint {
    pub fn new(actor0: Entity, actor1: Entity) -> Self {
        Self { actor0, local_frame0: Transform::default(), actor1, local_frame1: Transform::default(), break_force: core::f32::MAX, break_torque: core::f32::MAX }
    }

    pub fn with_local_frame0(mut self, local_frame0: Transform) -> Self {
        self.local_frame0 = local_frame0;
        self
    }

    pub fn with_local_frame1(mut self, local_frame1: Transform) -> Self {
        self.local_frame1 = local_frame1;
        self
    }

    pub fn with_break_force(mut self, break_force: f32) -> Self {
        self.break_force = break_force;
        self
    }

    pub fn with_break_torque(mut self, break_torque: f32) -> Self {
        self.break_torque = break_torque;
        self
    }
}


pub struct FixedJointPlugin;

impl Plugin for FixedJointPlugin {
    fn build(&self, app: &mut App) {
        // app.register_type::<FixedJoint>();

        app.add_systems(crate::PhysicsSchedule, (
            create_fixed_joint.after(crate::core::systems::create_rigid_actors).in_set(crate::PhysicsSet::Last), //need to be after actor handle creation, .after(create_rigid_actors) does not work for some reason
            sync_fixed_joint.in_set(crate::PhysicsSet::Sync)
        )); 
    }
}



pub fn create_fixed_joint(
    mut commands: Commands,
    mut scene: ResMut<crate::prelude::Scene>,
    mut physics: ResMut<crate::prelude::Physics>,
    query: Query<(Entity, &FixedJoint), Without<FixedJointHandle>>,
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

            let px_fixed_joint = physx_sys::phys_PxFixedJointCreate(
                physics.physics_mut().as_mut_ptr(), 
                actor0_handle, fixed_joint.local_frame0.to_physx_sys().as_mut_ptr(), 
                actor1_handle, fixed_joint.local_frame1.to_physx_sys().as_mut_ptr()
            );

            physx_sys::PxJoint_setBreakForce_mut(px_fixed_joint as *mut physx_sys::PxJoint, fixed_joint.break_force, fixed_joint.break_torque);


            commands.entity(entity).insert(FixedJointHandle::new(px_fixed_joint));

        }


    }

}



pub fn sync_fixed_joint(
    mut scene: ResMut<crate::prelude::Scene>,
    mut query: Query<(&FixedJoint, &mut FixedJointHandle), Changed<FixedJoint>>,
) {

    for (fixed_joint, mut fixed_joint_handle) in query.iter_mut() {

        unsafe {

            physx_sys::PxJoint_setLocalPose_mut(fixed_joint_handle.handle.get_mut(&mut scene).to_owned() as *mut physx_sys::PxJoint, 
                physx_sys::PxJointActorIndex::Actor0, fixed_joint.local_frame0.to_physx_sys().as_mut_ptr());

            physx_sys::PxJoint_setLocalPose_mut(fixed_joint_handle.handle.get_mut(&mut scene).to_owned() as *mut physx_sys::PxJoint, 
                physx_sys::PxJointActorIndex::Actor1, fixed_joint.local_frame1.to_physx_sys().as_mut_ptr());


            physx_sys::PxJoint_setBreakForce_mut(fixed_joint_handle.handle.get_mut(&mut scene).to_owned() as *mut physx_sys::PxJoint, 
                fixed_joint.break_force, fixed_joint.break_torque);

        }

    }

}



