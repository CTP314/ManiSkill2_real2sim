from copy import deepcopy
import numpy as np

from mani_skill2_real2sim.agents.controllers import *
from mani_skill2_real2sim.sensors.camera import CameraConfig

class ARX5DefaultConfig:
    def __init__(self, mobile_base=False, finger_friction=2.0, base_arm_drive_mode="force") -> None:
        self.urdf_path = "{PACKAGE_ASSET_DIR}/descriptions/arx5_description/arx5_description_isaac.urdf"

        finger_min_patch_radius = 0.01
        self.urdf_config = dict(
            _materials=dict(
                gripper=dict(static_friction=2.0, dynamic_friction=2.0, restitution=0.0)
            ),
            link=dict(
                fl_link7=dict(
                    material="gripper",
                    patch_radius=finger_min_patch_radius,
                    min_patch_radius=finger_min_patch_radius,
                ),
                fl_link8=dict(
                    material="gripper",
                    patch_radius=finger_min_patch_radius,
                    min_patch_radius=finger_min_patch_radius,
                ),
                fr_link7=dict(
                    material="gripper",
                    patch_radius=finger_min_patch_radius,
                    min_patch_radius=finger_min_patch_radius,
                ),
                fr_link8=dict(
                    material="gripper",
                    patch_radius=finger_min_patch_radius,
                    min_patch_radius=finger_min_patch_radius,
                ),
            ),
        )
        self.ee_link_names = []
        self.arm_joints_dict = {}
        self.ee_links_dict = {}
        self.gripper_joints_dict = {}
        self.arm_stiffness = 1e3
        self.arm_damping = 1e2
        self.arm_force_limit = 100
        self.gripper_stiffness = 1e3
        self.gripper_damping = 1e2
        self.gripper_force_limit = 100

        for prefix in ["fl", "fr"]:
            arm_joints = []
            for i in range(1, 7):
                arm_joints.append(f"{prefix}_joint{i}")
            self.arm_joints_dict[prefix] = arm_joints
            self.ee_links_dict[prefix] = f"{prefix}_link6"
            self.ee_link_names.append(f"{prefix}_link6")

            gripper_joints = []
            for i in range(7, 9):
                gripper_joints.append(f"{prefix}_joint{i}")
            self.gripper_joints_dict[prefix] = gripper_joints
            

    @property
    def controllers(self):

        # Make a deepcopy in case users modify any config
        arms_pd_joint_pos = []
        arms_pd_ee_target_delta_pose = []
        grippers_pd_jonit_pos = []
        for prefix in ["fl", "fr"]:
            arms_pd_joint_pos.append(
                PDJointPosControllerConfig(
                    self.arm_joints_dict[prefix],
                    None,
                    None,
                    self.arm_stiffness,
                    self.arm_damping,
                    self.arm_force_limit,
                    normalize_action=False,
                )
            )
            arms_pd_ee_target_delta_pose.append(
                PDEEPoseControllerConfig(
                    self.arm_joints_dict[prefix],
                    -0.1,
                    0.1,
                    self.arm_stiffness,
                    self.arm_damping,
                    self.arm_force_limit,
                    ee_link=self.ee_links_dict[prefix],
                )
            )
            grippers_pd_jonit_pos.append(
                PDJointPosMimicControllerConfig(
                    self.gripper_joints_dict[prefix],
                    -0.01,
                    0.04,
                    self.gripper_stiffness,
                    self.gripper_damping,
                    self.gripper_force_limit,
                )
            )
        controller_configs = dict(
            pd_joint_pos=dict(
                left_arm=arms_pd_joint_pos[0],
                left_gripper=grippers_pd_jonit_pos[0],
                right_arm=arms_pd_joint_pos[1],
                right_gripper=grippers_pd_jonit_pos[1],
            ),
            pd_ee_target_delta_pose=dict(
                left_arm=arms_pd_ee_target_delta_pose[0],
                left_gripper=grippers_pd_jonit_pos[0],
                right_arm=arms_pd_ee_target_delta_pose[1],
                right_gripper=grippers_pd_jonit_pos[1],
            )
        )
        return deepcopy_dict(controller_configs)
    
    @property
    def cameras(self):
        return [
            CameraConfig(
                uid="front_camera",
                p=[0, 0, 0],
                q=[1, 0, 0, 0],
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
                q=[1, 0, 0, 0],
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
                q=[1, 0, 0, 0],
                width=384,
                height=384,
                fov=0.9599310885968813,
                near=0.01,
                far=10,
                actor_uid="right_camera",
            )
        ]