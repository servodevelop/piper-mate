import logging
import time
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from piper_sdk import *

from .config_piper import PiperConfig
from functools import cached_property

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
# from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.robots.robot import Robot




logger = logging.getLogger(__name__)


class Piper(Robot):
    config_class = PiperConfig
    name = "starai_piper"

    def __init__(self, config: PiperConfig):
        super().__init__(config)
        self.config = config
        self.piper = C_PiperInterface_V2("can0")
        self._is_connected:bool = False

        self.cameras = make_cameras_from_configs(config.cameras)

        self.motors_pos_type = {"Joint_1.pos": float,
                        "Joint_2.pos": float,
                        "Joint_3.pos": float,
                        "Joint_4.pos": float,
                        "Joint_5.pos": float,
                        "Joint_6.pos": float,
                        "Gripper.pos": float}

        

    @property
    def _motors_ft(self) -> dict[str, type]:
        return self.motors_pos_type

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self._is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        self.piper.ConnectPort()
        for cam in self.cameras.values():
            cam.connect()
        while( not self.piper.EnablePiper()):
            time.sleep(0.01)
        self._is_connected = True

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass


    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()
        ArmJointMsgs = self.piper.GetArmJointMsgs()
        ArmGripperMsgs = self.piper.GetArmGripperMsgs()

        obs_dict = {}
        obs_dict["Joint_1.pos"] = ArmJointMsgs.joint_state.joint_1*0.001
        obs_dict["Joint_2.pos"] = ArmJointMsgs.joint_state.joint_2*0.001
        obs_dict["Joint_3.pos"] = ArmJointMsgs.joint_state.joint_3*0.001
        obs_dict["Joint_4.pos"] = ArmJointMsgs.joint_state.joint_4*0.001
        obs_dict["Joint_5.pos"] = ArmJointMsgs.joint_state.joint_5*0.001
        obs_dict["Joint_6.pos"] = ArmJointMsgs.joint_state.joint_6*0.001
        obs_dict["Gripper.pos"]= ArmGripperMsgs.gripper_state.grippers_angle*0.001

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def clamp(self,value, min_val, max_val):
        """
        将 value 限制在 [min_val, max_val] 范围内。

        Args:
            value (float or int): 待限幅的值。
            min_val (float or int): 最小允许值。
            max_val (float or int): 最大允许值。

        Returns:
            float or int: 限幅后的值。
        """
        return max(min_val, min(value, max_val))

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:

        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
        # self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)

        # Convert tensor values to Python floats before rounding
        joint_1 = round(self.clamp(float(action["Joint_1.pos"]),-150,150)*1000)
        joint_2 = round(self.clamp(float(action["Joint_2.pos"]),0,180)*1000)
        joint_3 = round(self.clamp(float(action["Joint_3.pos"]),-170,0)*1000)
        joint_4 = round(self.clamp(float(action["Joint_4.pos"]),-100,100)*1000)
        joint_5 = round(self.clamp(float(action["Joint_5.pos"]),-70,70)*1000)
        joint_6 = round(self.clamp(float(action["Joint_6.pos"]),-120,120)*1000)
        gripper = round(self.clamp(float(action["Gripper.pos"]),0,100)*1000)

        self.piper.JointCtrl(joint_1, joint_2, joint_3, joint_4, joint_5,joint_6)

        self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)

    def disconnect(self):
        self._is_connected = False
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")

    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()

        ArmJointMsgs = self.piper.GetArmJointMsgs()
        ArmGripperMsgs = self.piper.GetArmGripperMsgs()

        action = {}
        action["Joint_1.pos"] = ArmJointMsgs.joint_state.joint_1*0.001
        action["Joint_2.pos"] = ArmJointMsgs.joint_state.joint_2*0.001
        action["Joint_3.pos"] = ArmJointMsgs.joint_state.joint_3*0.001
        action["Joint_4.pos"] = ArmJointMsgs.joint_state.joint_4*0.001
        action["Joint_5.pos"] = ArmJointMsgs.joint_state.joint_5*0.001
        action["Joint_6.pos"] = ArmJointMsgs.joint_state.joint_6*0.001
        action["Gripper.pos"]= ArmGripperMsgs.gripper_state.grippers_angle*0.001



        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    
