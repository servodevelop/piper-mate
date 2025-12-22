#!/usr/bin/env python3
"""
FashionStar与Piper机械臂集成控制
通过USB口连接FashionStar机械臂，读取其关节数据并控制Piper机械臂
"""

import math
import time
import serial
import argparse
from typing import List, Optional

# 导入FashionStar SDK
from fashionstar_uart_sdk.uart_pocket_handler import (
    PortHandler as starai_PortHandler,
    SyncPositionControlOptions,
)

# 导入Piper SDK
from piper_sdk import C_PiperInterface_V2


class FashionStarAgilex:
    """
    FashionStar与Piper机械臂集成控制类
    """
    
    # Piper手臂关节角度限制（角度和弧度）
    JOINT_ANGLE_LIMITS = {
        'joint1': [-150.0, 150.0],      # 角度限制
        'joint2': [0.0, 180.0],         
        'joint3': [-170.0, 0.0],        
        'joint4': [-100.0, 100.0],      
        'joint5': [-70.0, 70.0],        
        'joint6': [-120.0, 120.0],      
    }
    
    JOINT_RADIAN_LIMITS = {
        'joint1': [-2.6179, 2.6179],    # 弧度限制
        'joint2': [0, 3.14],            
        'joint3': [-2.967, 0],          
        'joint4': [-1.745, 1.745],      
        'joint5': [-1.22, 1.22],        
        'joint6': [-2.09439, 2.09439],  
    }
    
    def __init__(self, 
                 fashionstar_port: str = "/dev/ttyUSB0", 
                 piper_can_name: str = "can0",
                 gripper_exist: bool = True):
        """
        初始化FashionStar和Piper机械臂
        
        Args:
            fashionstar_port: FashionStar机械臂USB端口
            piper_can_name: Piper机械臂CAN端口名称
            gripper_exist: 是否包含夹爪
        """
        self.gripper_exist = gripper_exist
        
        # 初始化FashionStar机械臂
        print(f"初始化FashionStar机械臂，端口: {fashionstar_port}")
        try:
            self.fashionstar_handler = starai_PortHandler(fashionstar_port, 1000000)
            self.fashionstar_handler.openPort()
            print("FashionStar机械臂连接成功")
        except Exception as e:
            print(f"FashionStar机械臂连接失败: {e}")
            raise
        
        # 舵机配置
        self.servo_ids = [0, 1, 2, 3, 4, 5]  # 6个关节舵机
        if gripper_exist:
            self.servo_ids.append(6)  # 夹爪舵机
        
        # 清除圈数
        for servo_id in self.servo_ids:
            self.fashionstar_handler.write["Stop_On_Control_Mode"](servo_id, "unlocked", 900)
        self.fashionstar_handler.reset_multi_turn_angle(0xff)

        # 关节名称
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        if gripper_exist:
            self.joint_names.append('gripper')
        
        # 初始化Piper机械臂
        print(f"初始化Piper机械臂，CAN端口: {piper_can_name}")
        try:
            self.piper_interface = C_PiperInterface_V2.get_instance(
                can_name=piper_can_name,
                judge_flag=True,
                can_auto_init=True
            )
            print("Piper机械臂接口初始化成功")
        except Exception as e:
            print(f"Piper机械臂接口初始化失败: {e}")
            raise
        
        # 连接Piper机械臂
        try:
            self.piper_interface.ConnectPort()
            print("Piper机械臂连接成功")
        except Exception as e:
            print(f"Piper机械臂连接失败: {e}")
            raise
        
        # 设置Piper机械臂控制模式为关节控制模式
        try:
            # 设置控制模式：关节控制模式，速度控制模式
            self.piper_interface.ModeCtrl(
                ctrl_mode=0x01,      # 关节控制模式
                move_mode=0x01,      # 速度控制模式
                move_spd_rate_ctrl=100  # 速度控制比例
                # is_mit_mode=0xAD      # MIT模式
            )
            print("Piper机械臂控制模式设置成功")
        except Exception as e:
            print(f"Piper机械臂控制模式设置失败: {e}")

        print("FashionStar与Piper机械臂集成控制初始化完成")
    
    def degrees_to_radians(self, degrees: float) -> float:
        """将角度转换为弧度"""
        return degrees * (math.pi / 180)
    
    def radians_to_degrees(self, radians: float) -> float:
        """将弧度转换为角度"""
        return radians * (180 / math.pi)
    
    def degrees_to_meters(self, degrees: float) -> float:
        """将夹爪角度转换为米（0-0.08m）"""
        return (degrees / 45.0) * 0.08
    
    def servoangle2jointstate(self, servo_id: int, servo_angle: float) -> float:
        """
        将FashionStar舵机角度转换为Piper关节位置
        
        Args:
            servo_id: 舵机ID
            servo_angle: 舵机角度
            
        Returns:
            转换后的关节位置（弧度或米）
        """
        if servo_id in range(6):  # 手臂关节
            # 先对第1、4、6关节方向取反
            if servo_id in [0, 3, 5]:  # joint1, joint4, joint6
                servo_angle = -servo_angle
            
            # 根据关节名称获取角度限制
            joint_name = f'joint{servo_id + 1}'
            angle_limits = self.JOINT_ANGLE_LIMITS.get(joint_name, [-180.0, 180.0])
            
            # 对取反后的角度进行限制
            limited_angle = max(angle_limits[0], min(servo_angle, angle_limits[1]))
            
            # 转换为弧度
            return self.degrees_to_radians(limited_angle)
        elif servo_id == 6:  # 夹爪
            # 夹爪：将角度转换为米
            return self.degrees_to_meters(servo_angle)
        else:
            return 0.0
    
    def get_fashionstar_joint_states(self) -> dict:
        """
        读取FashionStar机械臂的关节状态
        
        Returns:
            包含关节名称和对应位置的字典
        """
        try:
            # 同步读取所有舵机状态
            servos_id = {name: servo_id for name, servo_id in zip(self.joint_names, self.servo_ids)}
            monitor_data = self.fashionstar_handler.sync_read["Monitor"](servos_id)
            
            # 转换关节状态
            joint_states = {}
            for joint_name in self.joint_names:
                servo_id = servos_id[joint_name]
                servo_angle = monitor_data[joint_name].current_position
                joint_state = self.servoangle2jointstate(servo_id, servo_angle)
                joint_states[joint_name] = joint_state
            
            return joint_states
            
        except Exception as e:
            print(f"读取FashionStar关节状态失败: {e}")
            return {}
    
    def control_piper_joints(self, joint_states: dict):
        """
        根据FashionStar关节状态控制Piper机械臂
        
        Args:
            joint_states: 包含关节名称和对应位置的字典
        """
        try:
            # 提取关节角度（转换为Piper需要的0.001度单位）
            joint_angles = []
            for i in range(1, 7):  # joint1到joint6
                joint_name = f'joint{i}'
                if joint_name in joint_states:
                    # 将弧度转换为0.001度
                    radians = joint_states[joint_name]
                    degrees = self.radians_to_degrees(radians)
                    micro_degrees = int(degrees * 1000)  # 转换为0.001度
                    joint_angles.append(micro_degrees)
                else:
                    joint_angles.append(0)
            
            # 控制Piper机械臂关节
            if len(joint_angles) == 6:
                self.piper_interface.JointCtrl(
                    joint_angles[0],  # joint1
                    joint_angles[1],  # joint2
                    joint_angles[2],  # joint3
                    joint_angles[3],  # joint4
                    joint_angles[4],  # joint5
                    joint_angles[5]   # joint6
                )
            
            # 控制Piper夹爪（如果存在）
            if self.gripper_exist and 'gripper' in joint_states:
                # 将夹爪位置（米）转换为Piper需要的单位（微米，0.001mm）
                gripper_meters = joint_states['gripper']
                gripper_micrometers = int(gripper_meters * 1000 * 1000)  # 米 -> 毫米 -> 微米
                # 控制Piper夹爪
                # 参数：夹爪距离(微米), 夹爪力矩(0.001N/m), 控制码(0x01启用), 设置零点(0不设置)
                self.piper_interface.GripperCtrl(gripper_micrometers, 1000, 0x01, 0)
                
        except Exception as e:
            print(f"控制Piper机械臂失败: {e}")
    
    def enable_torque(self):
        """使能FashionStar力矩"""
        try:
            # 使用锁定模式使能力矩
            for servo_id in self.servo_ids:
                self.fashionstar_handler.write["Stop_On_Control_Mode"](servo_id, "locked", 0)
            print("FashionStar机械臂力矩已使能")
        except Exception as e:
            print(f"使能力矩失败: {e}")
    
    def disable_torque(self):
        """禁用FashionStar力矩"""
        try:
            # 使用解锁模式禁用力矩
            for servo_id in self.servo_ids:
                self.fashionstar_handler.write["Stop_On_Control_Mode"](servo_id, "unlocked", 900)
            print("FashionStar机械臂力矩已禁用")
        except Exception as e:
            print(f"禁用力矩失败: {e}")
    
    def get_piper_joint_states(self):
        """获取Piper机械臂的关节状态"""
        try:
            return self.piper_interface.GetArmJoint()
        except Exception as e:
            print(f"获取Piper关节状态失败: {e}")
            return None
    
    def close(self):
        """关闭连接"""
        print("关闭机械臂连接...")
        
        # 禁用FashionStar力矩
        self.disable_torque()
        
        # 关闭Piper连接
        try:
            self.piper_interface.DisconnectPort()
            print("Piper机械臂连接已关闭")
        except Exception as e:
            print(f"关闭Piper连接失败: {e}")
        
        # 关闭FashionStar连接
        try:
            self.fashionstar_handler.closePort()
            print("FashionStar机械臂连接已关闭")
        except Exception as e:
            print(f"关闭FashionStar连接失败: {e}")
        
        print("所有连接已关闭")


def main():
    """主函数 - 预设参数，持续遥操作"""
    
    # 预设参数 - 根据实际情况修改这些值
    FASHIONSTAR_PORT = "/dev/ttyUSB0"    # FashionStar USB端口
    PIPER_CAN_NAME = "can0"              # Piper CAN端口
    GRIPPER_EXIST = True                 # 是否包含夹爪
    UPDATE_RATE = 100.0                   # 控制频率（Hz）- 可调节

    try:
        # 创建机械臂控制器
        robot_controller = FashionStarAgilex(
            fashionstar_port=FASHIONSTAR_PORT,
            piper_can_name=PIPER_CAN_NAME,
            gripper_exist=GRIPPER_EXIST
        )
        # 开始持续遥操作
        print(f"\n开始FashionStar控制Piper,控制频率{UPDATE_RATE}Hz...")
        print("按 Ctrl+C 停止遥操作")
        print("=" * 120)
        
        # 持续遥操作循环
        update_interval = 1.0 / UPDATE_RATE
        frame_count = 0
        
        while True:
            try:
                # 读取FashionStar关节状态
                joint_states = robot_controller.get_fashionstar_joint_states()
                
                if joint_states:
                    # 控制Piper机械臂
                    robot_controller.control_piper_joints(joint_states)
                    
                    # 实时显示关节状态（每10帧显示一次，避免刷屏）
                    frame_count += 1
                    if frame_count % 10 == 0:
                        print("\rpiper当前关节状态:", end="")
                        for joint, state in joint_states.items():
                            print(f" {joint}:{state:.4f}", end="")
                        print("   ", end="", flush=True)
                
                # 等待下一个更新周期
                time.sleep(update_interval)
                
            except KeyboardInterrupt:
                print("\n\n用户停止遥操作")
                break
            except Exception as e:
                print(f"\n遥操作过程中出错: {e}")
                import traceback
                traceback.print_exc()
                # 继续运行，不退出
                time.sleep(1.0)  # 出错后等待1秒再继续
        
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    except Exception as e:
        print(f"\n程序运行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 确保连接被正确关闭
        if 'robot_controller' in locals():
            robot_controller.close()


if __name__ == "__main__":
    main()