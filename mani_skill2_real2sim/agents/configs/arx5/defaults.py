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

