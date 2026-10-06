from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple,Any, Dict, Protocol

from lerobot.cameras import CameraConfig

from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("lerobot_robot_piper")
@dataclass
class PiperConfig(RobotConfig):
    # Port to connect to the arm
    can_name: str

    # cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

