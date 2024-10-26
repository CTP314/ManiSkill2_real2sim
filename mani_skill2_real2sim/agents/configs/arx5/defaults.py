from copy import deepcopy
import numpy as np

from mani_skill2_real2sim.agents.controllers import *
from mani_skill2_real2sim.sensors.camera import CameraConfig

class ARX5DefaultConfig:
    def __init__(self, mobile_base=False, finger_friction=2.0, base_arm_drive_mode="force") -> None:
        self.urdf_path = "{PACKAGE_ASSET_DIR}/descriptions/arx5_description/urdf/arx5_description_isaac.urdf"

        finger_min_patch_radius = 0.01
        self.urdf_config = dict(
            _materials=dict(
                gripper=dict(static_friction=2.0, dynamic_friction=2.0, restitution=0.0)
            ),
            link=dict(
                left_finger_link=dict(
                    material="gripper",
                    patch_radius=finger_min_patch_radius,
                    min_patch_radius=finger_min_patch_radius,
                ),
                right_finger_link=dict(
                    material="gripper",
                    patch_radius=finger_min_patch_radius,
                    min_patch_radius=finger_min_patch_radius,
                ),
            ),
        )
        self.arm_joint_names = [
            
        ]
        
    @property
    def controllers(self):
        # -------------------------------------------------------------------------- #
        # Arm
        # -------------------------------------------------------------------------- #
        arm_pd_joint_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            None,
            None,
            self.arm_stiffness,
            self.arm_damping,
            self.arm_force_limit,
            normalize_action=False,
        )
        arm_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            -0.1,
            0.1,
            self.arm_stiffness,
            self.arm_damping,
            self.arm_force_limit,
            normalize_action=False,
        )
        arm_pd_joint_target_delta_pos = deepcopy(arm_pd_joint_delta_pos)
        arm_pd_joint_target_delta_pos.use_target_delta = True
        
        # PD ee position
        arm_pd_ee_delta_pos = PDEEPosControllerConfig(
            self.arm_joint_names,
            -0.1,
            0.1,
            self.arm_stiffness,
            self.arm_damping,
            self.arm_force_limit,
            ee_link=self.ee_link_name,
        )
        arm_pd_ee_delta_pose = PDEEPoseControllerConfig(
            self.arm_joint_names,
            -0.1,
            0.1,
            0.1,
            self.arm_stiffness,
            self.arm_damping,
            self.arm_force_limit,
            ee_link=self.ee_link_name,
        )

        arm_pd_ee_target_delta_pos = deepcopy(arm_pd_ee_delta_pos)
        arm_pd_ee_target_delta_pos.use_target = True
        arm_pd_ee_target_delta_pose = deepcopy(arm_pd_ee_delta_pose)
        arm_pd_ee_target_delta_pose.use_target = True
        
        # PD joint velocity
        arm_pd_joint_vel = PDJointVelControllerConfig(
            self.arm_joint_names,
            -1.0,
            1.0,
            self.arm_damping,  # this might need to be tuned separately
            self.arm_force_limit,
        )
        
        # -------------------------------------------------------------------------- #
        # Gripper
        # -------------------------------------------------------------------------- #
        # NOTE(jigu): IssacGym uses large P and D but with force limit
        # However, tune a good force limit to have a good mimic behavior
        gripper_pd_joint_pos = PDJointPosMimicControllerConfig(
            self.gripper_joint_names,
            -0.01,  # a trick to have force when the object is thin
            0.04,
            self.gripper_stiffness,
            self.gripper_damping,
            self.gripper_force_limit,
        )

        controller_configs = dict(
            pd_joint_delta_pos=dict(
                arm=arm_pd_joint_delta_pos, gripper=gripper_pd_joint_pos
            ),
            pd_joint_pos=dict(arm=arm_pd_joint_pos, gripper=gripper_pd_joint_pos),
            pd_ee_delta_pos=dict(arm=arm_pd_ee_delta_pos, gripper=gripper_pd_joint_pos),
            pd_ee_delta_pose=dict(
                arm=arm_pd_ee_delta_pose, gripper=gripper_pd_joint_pos
            ),
            pd_joint_target_delta_pos=dict(
                arm=arm_pd_joint_target_delta_pos, gripper=gripper_pd_joint_pos
            ),
            pd_ee_target_delta_pos=dict(
                arm=arm_pd_ee_target_delta_pos, gripper=gripper_pd_joint_pos
            ),
            pd_ee_target_delta_pose=dict(
                arm=arm_pd_ee_target_delta_pose, gripper=gripper_pd_joint_pos
            ),
            # Caution to use the following controllers
            pd_joint_vel=dict(arm=arm_pd_joint_vel, gripper=gripper_pd_joint_pos),
        )

        # Make a deepcopy in case users modify any config
        return deepcopy_dict(controller_configs)
    
    @property
    def cameras(self):
        return [
            CameraConfig(
                uid="front_camera",
                p=[0, 0, 0],
                q=[0, 0, 0, 1],
                width=384,
                height=384,
                fov=0.9599310885968813,
                near=0.01,
                far=10,
                actor_uid="camera_link2",
            ),
            CameraConfig(
                uid="left_camera",
                p=[0, 0, 0],
                q=[0, 0, 0, 1],
                width=384,
                height=384,
                fov=0.9599310885968813,
                near=0.01,
                far=10,
                actor_uid="left_camera",
            ),
            CameraConfig(
                uid="right_camera",
                p=[0, 0, 0],
                q=[0, 0, 0, 1],
                width=384,
                height=384,
                fov=0.9599310885968813,
                near=0.01,
                far=10,
                actor_uid="right_camera",
            )
        ]