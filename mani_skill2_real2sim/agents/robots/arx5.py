import numpy as np
import sapien.core as sapien
from sapien.core import Pose
from pathlib import Path

from mani_skill2_real2sim.agents.base_agent import BaseAgent
from mani_skill2_real2sim.agents.configs.arx5 import defaults
from mani_skill2_real2sim.utils.common import compute_angle_between
from mani_skill2_real2sim.utils.sapien_utils import (
    get_entity_by_name,
    get_pairwise_contact_impulse,
)
from mani_skill2_real2sim import format_path
from mani_skill2_real2sim.utils.sapien_utils import check_urdf_config, parse_urdf_config


class ARX5(BaseAgent):
    def _after_init(self):
        return super()._after_init()

    def get_gripper_closedness(self):
        raise NotImplementedError
    
    def get_fingers_info(self):
        raise NotImplementedError
    
    def check_grasp(self):
        raise NotImplementedError
    
    def check_contact_fingers(self, actor: sapien.ActorBase, min_impulse=1e-6):
        raise NotImplementedError
    
    @staticmethod
    def build_grasp_pose(approaching, closing, center):
        raise NotImplementedError
    
    def get_proprioception(self):
        state_dict = super().get_proprioception()
        return state_dict
    
    @property
    def base_pose(self):
        raise NotImplementedError
    
    def set_base_pose(self, xy):
        raise NotImplementedError
    
    def _load_articulation(self):
        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = self.fix_root_link
        # loader.load_multiple_collisions_from_file = True

        urdf_path = format_path(str(self.urdf_path))

        urdf_config = parse_urdf_config(self.urdf_config, self.scene)
        check_urdf_config(urdf_config)

        # TODO(jigu): support loading multiple convex collision shapes
        self.robot = loader.load(urdf_path, urdf_config)
        assert self.robot is not None, f"Fail to load URDF from {urdf_path}"
        self.robot.set_name(Path(urdf_path).stem)

        # Cache robot link ids
        self.robot_link_ids = [link.get_id() for link in self.robot.get_links()]
    
class ARX5StaticBase(ARX5):
    _config: defaults.ARX5DefaultConfig
    
    @classmethod
    def get_default_config(cls):
        return defaults.ARX5DefaultConfig()
    
    def __init__(
        self, scene, control_freq, control_mode=None, fix_root_link=True, config=None,
    ):
        if control_mode is None:
            raise ValueError("control_mode must be specified")
        super().__init__(
            scene,
            control_freq,
            control_mode=control_mode,
            fix_root_link=fix_root_link,
            config=config,
        )
        