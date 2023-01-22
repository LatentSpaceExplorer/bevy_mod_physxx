use bevy::prelude::*;
use physx::cooking::{PxCooking, PxCookingParams};
use physx::prelude::*;
use physx::traits::Class;
use physx_sys::{phys_PxInitVehicleSDK, phys_PxVehicleSetBasisVectors, phys_PxVehicleSetUpdateMode, PxVehicleUpdateMode, phys_PxCloseVehicleSDK, PxRaycastHit, PxBatchQueryDesc_new, PxRaycastQueryResult, PxFilterData, PxQueryHitType, PxHitFlags, PxScene_createBatchQuery_mut, PxBatchQuery};
use std::ffi::c_void;
use std::ops::{Deref, DerefMut};
use std::ptr::{null_mut, drop_in_place};
use crate::assets::BPxMaterial;

use super::prelude::*;
use super::{PxShape, PxScene};

#[derive(Resource)]
pub struct BPxPhysics {
    physics: PhysicsFoundation<physx::foundation::DefaultAllocator, PxShape>,
    vsdk: bool,
}

impl BPxPhysics {
    pub fn new(enable_debugger: bool, enable_vsdk: bool) -> Self {
        let mut physics;

        let mut builder = physx::physics::PhysicsFoundationBuilder::default();
        builder.enable_visual_debugger(enable_debugger);
        builder.with_extensions(true);
        physics = builder.build();

        if physics.is_none() && enable_debugger {
            // failed to connect, try without debugger
            let mut builder = physx::physics::PhysicsFoundationBuilder::default();
            builder.with_extensions(true);
            physics = builder.build();
        }

        let mut physics = physics.expect("building PhysX foundation failed");

        if enable_vsdk {
            unsafe {
                phys_PxInitVehicleSDK(physics.as_mut_ptr(), null_mut());
                phys_PxVehicleSetBasisVectors(PxVec3::new(0.,1.,0.).as_ptr(), PxVec3::new(0.,0.,1.).as_ptr());
                phys_PxVehicleSetUpdateMode(PxVehicleUpdateMode::eVELOCITY_CHANGE);
            }
        }

        Self { physics, vsdk: enable_vsdk }
    }
}

impl Deref for BPxPhysics {
    type Target = PhysicsFoundation<physx::foundation::DefaultAllocator, PxShape>;
    fn deref(&self) -> &Self::Target {
        &self.physics
    }
}

impl DerefMut for BPxPhysics {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.physics
    }
}

impl Drop for BPxPhysics {
    fn drop(&mut self) {
        if self.vsdk {
            unsafe {
                phys_PxCloseVehicleSDK(null_mut());
            }
            return;
        }
    }
}

#[derive(Resource, Deref, DerefMut)]
pub struct BPxScene(Owner<PxScene>);

impl BPxScene {
    pub fn new(physics: &mut BPxPhysics, gravity: Vec3) -> Self {
        let scene = physics
            .create(SceneDescriptor {
                gravity: gravity.to_physx(),
                ..SceneDescriptor::new(())
            })
            .unwrap();

        Self(scene)
    }
}

#[derive(Resource, Deref, DerefMut)]
pub struct BPxCooking(Owner<PxCooking>);

impl BPxCooking {
    pub fn new(physics: &mut BPxPhysics) -> Self {
        let params = &PxCookingParams::new(&**physics).expect("failed to create cooking params");
        let cooking = PxCooking::new(physics.foundation_mut(), &params).expect("failed to create cooking");
        Self(cooking)
    }
}

#[derive(Resource, Default)]
pub struct BPxTimeSync {
    timestep: f32,
    speed_factor: f32,
    bevy_physx_delta: f32,
}

impl BPxTimeSync {
    pub fn new(timestep: f32) -> Self {
        Self { timestep, speed_factor: 1., ..default() }
    }

    pub fn get_delta(&self) -> f32 {
        self.bevy_physx_delta
    }

    pub fn advance_bevy_time(&mut self, time: &Time) {
        self.bevy_physx_delta += time.delta_seconds() * self.speed_factor;
    }

    pub fn check_advance_physx_time(&mut self) -> Option<f32> {
        if self.bevy_physx_delta >= self.timestep {
            self.bevy_physx_delta -= self.timestep;
            Some(self.timestep)
        } else {
            None
        }
    }
}

#[derive(Resource, Deref, DerefMut, Default)]
pub struct BPxDefaultMaterial(Option<Handle<BPxMaterial>>);

#[derive(Resource)]
pub struct BPxVehicleRaycastBuffer {
    current_size: usize,
    sq_results: Vec<u8>,
    sq_hit_buffer: Vec<u8>,
    batch_query: *mut PxBatchQuery,
}

unsafe impl Send for BPxVehicleRaycastBuffer {}
unsafe impl Sync for BPxVehicleRaycastBuffer {}

impl BPxVehicleRaycastBuffer {
    pub fn alloc(&mut self, scene: &mut BPxScene, wheel_count: usize) {
        extern "C" fn pre_filter_shader(_data0: PxFilterData, _data1: PxFilterData, _cblock: c_void, _cblocksize: u32, _flags: PxHitFlags) -> u32 {
            PxQueryHitType::eNONE
        }

        // buffers already allocated
        if wheel_count <= self.current_size { return; }

        self.current_size = wheel_count.next_power_of_two();
        // PxRaycastQueryResult, rust port generates wrong struct size, also 80 bytes aren't enough?
        self.sq_results = vec![0u8; 100 * self.current_size];
        self.sq_hit_buffer = vec![0u8; std::mem::size_of::<PxRaycastHit>() * self.current_size];
        let mut sq_desc = unsafe { PxBatchQueryDesc_new(self.current_size as u32, 0, 0) };

        if !self.batch_query.is_null() {
            unsafe { drop_in_place(self.batch_query); }
        }

        self.batch_query = unsafe {
            sq_desc.queryMemory.userRaycastResultBuffer = self.sq_results.as_mut_ptr() as *mut PxRaycastQueryResult;
            sq_desc.queryMemory.userRaycastTouchBuffer = self.sq_hit_buffer.as_mut_ptr() as *mut PxRaycastHit;
            sq_desc.queryMemory.raycastTouchBufferSize = self.current_size as u32;
            sq_desc.preFilterShader = pre_filter_shader as *mut c_void;
            PxScene_createBatchQuery_mut(scene.as_mut_ptr(), &sq_desc as *const _)
        };
    }

    pub fn get_batch_query(&mut self) -> *mut PxBatchQuery {
        self.batch_query
    }

    pub fn get_query_results(&mut self) -> *mut PxRaycastQueryResult {
        self.sq_results.as_mut_ptr() as *mut PxRaycastQueryResult
    }
}

impl Default for BPxVehicleRaycastBuffer {
    fn default() -> Self {
        Self {
            current_size: 0,
            sq_results: vec![],
            sq_hit_buffer: vec![],
            batch_query: null_mut(),
        }
    }
}

impl Drop for BPxVehicleRaycastBuffer {
    fn drop(&mut self) {
        if !self.batch_query.is_null() {
            unsafe { drop_in_place(self.batch_query); }
        }
    }
}
