import logging
import time
from typing import Any

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from fashionstar_uart_sdk.uart_pocket_handler import (
    PortHandler ,
    Monitor_data
)
from lerobot.teleoperators.teleoperator import Teleoperator
from .config_starai_firefly import FireflyConfig

logger = logging.getLogger(__name__)


class Firefly(Teleoperator):
    config_class = FireflyConfig
    name = "starai_firefly"

    def __init__(self, config: FireflyConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self.porthandler:PortHandler = PortHandler(self.config.port,self.config.baudrate)
        self.motors = {"Joint_1": 0,
                        "Joint_2": 1,
                        "Joint_3": 2,
                        "Joint_4": 3,
                        "Joint_5": 4,
                        "Joint_6": 5,
                        "Gripper": 6}
        self.joint_dir={}
        self.joint_dir["Joint_1"] = self.config.joint_dir[0]
        self.joint_dir["Joint_2"] = self.config.joint_dir[1]
        self.joint_dir["Joint_3"] = self.config.joint_dir[2]
        self.joint_dir["Joint_4"] = self.config.joint_dir[3]
        self.joint_dir["Joint_5"] = self.config.joint_dir[4]
        self.joint_dir["Joint_6"] = self.config.joint_dir[5]
        self.joint_dir["Gripper"] = 2

        self.angle_min ={}
        self.angle_min["Joint_1"] = self.config.angle_min[0]
        self.angle_min["Joint_2"] = self.config.angle_min[1]
        self.angle_min["Joint_3"] = self.config.angle_min[2]
        self.angle_min["Joint_4"] = self.config.angle_min[3]
        self.angle_min["Joint_5"] = self.config.angle_min[4]
        self.angle_min["Joint_6"] = self.config.angle_min[5]
        self.angle_min["Gripper"] = self.config.angle_min[6]

        self.angle_max ={}
        self.angle_max["Joint_1"] = self.config.angle_max[0]
        self.angle_max["Joint_2"] = self.config.angle_max[1]
        self.angle_max["Joint_3"] = self.config.angle_max[2]
        self.angle_max["Joint_4"] = self.config.angle_max[3]
        self.angle_max["Joint_5"] = self.config.angle_max[4]
        self.angle_max["Joint_6"] = self.config.angle_max[5]
        self.angle_max["Gripper"] = self.config.angle_max[6]


    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.porthandler.openPort()
        for motor,value in self.motors.items():
            if not self.porthandler.ping(value):
                raise Exception(f"motor not found id:{value}")

        for motor,value in self.motors.items():
            self.porthandler.write["Stop_On_Control_Mode"](value, "unlocked", 900)
            time.sleep(0.01)
        self.porthandler.ResetLoop(0xFF)
        self._is_connected = True 
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

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


    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()

        monitor_data: dict[str, Monitor_data]= self.porthandler.sync_read["Monitor"](self.motors)
        
        action = {f"{motor}.pos": self.clamp(val.current_position * self.joint_dir[motor],self.angle_min[motor],self.angle_max[motor]) for motor, val in monitor_data.items()}

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            DeviceNotConnectedError(f"{self} is not connected.")
        self.porthandler.closePort()
        self._is_connected = False

        logger.info(f"{self} disconnected.")

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        raise NotImplementedError
    