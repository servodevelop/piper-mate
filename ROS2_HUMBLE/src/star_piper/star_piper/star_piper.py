#!/usr/bin/env python3
"""
FashionStar机械臂驱动节点
读取手臂角度信息并发布关节状态给Piper手臂
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import serial

# 从fashionstar_uart_sdk导入必要的组件
from fashionstar_uart_sdk.uart_pocket_handler import (
    PortHandler as starai_PortHandler,
    SyncPositionControlOptions,
)

STAR_PIPER_NODE = "star_piper_node"  # 驱动节点名称 / driver node name

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

class FashionStarDriver(Node):
    def __init__(self):
        super().__init__(STAR_PIPER_NODE)
        
        # 参数声明
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('gripper_exist', True)
        
        # 获取参数
        port = self.get_parameter('port').get_parameter_value().string_value
        auto_enable = self.get_parameter('auto_enable').get_parameter_value().bool_value
        gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        
        # 初始化机械臂
        self.get_logger().info(f"初始化FashionStar机械臂，端口: {port}")
        
        # 舵机配置
        self.servo_ids = [0, 1, 2, 3, 4, 5]  # 6个关节舵机
        if gripper_exist:
            self.servo_ids.append(6)  # 夹爪舵机
        
        # 关节名称
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        if gripper_exist:
            self.joint_names.append('gripper')
        
        # 初始化舵机端口处理器
        try:
            self.port_handler = starai_PortHandler(port, 1000000)
            self.port_handler.openPort()
            self.get_logger().info("机械臂连接成功")
        except Exception as e:
            self.get_logger().error(f"机械臂连接失败: {e}")
            raise
        # 清除圈数
        for servo_id in self.servo_ids:
            self.port_handler.write["Stop_On_Control_Mode"](servo_id, "unlocked", 900)
        self.port_handler.reset_multi_turn_angle(0xff)
        
        # 创建发布器 - 关节状态
        self.joint_ctrl_pub = self.create_publisher(JointState, 'joint_ctrl_single', 10)
        
        # 定时器 - 读取并发布关节状态
        self.timer = self.create_timer(0.001, self.publish_joint_states)
        
        # 使能控制
        if auto_enable:
            self.enable_torque()
        else:
            self.disable_torque()
        
        self.get_logger().info("FashionStar驱动节点初始化完成")
    
    def degrees_to_radians(self, degrees):
        """将角度转换为弧度"""
        return degrees * (math.pi / 180)
    
    def degrees_to_meters(self, degrees):
        """将夹爪角度转换为米（0-0.08m）"""
        return (degrees / 90.0) * 0.08
    
    def servoangle2jointstate(self, servo_id, servo_angle):
        """将舵机角度转换为关节位置，先取反再进行角度限制"""
        if servo_id in range(6):  # 手臂关节
            # 先对第1、4、6关节方向取反
            if servo_id in [0, 3, 5]:  # joint1, joint4, joint6
                servo_angle = -servo_angle
            
            # 根据关节名称获取角度限制
            joint_name = f'joint{servo_id + 1}'
            angle_limits = JOINT_ANGLE_LIMITS.get(joint_name, [-180.0, 180.0])
            
            # 对取反后的角度进行限制
            limited_angle = max(angle_limits[0], min(servo_angle, angle_limits[1]))
            
            # 转换为弧度
            return self.degrees_to_radians(limited_angle)
        elif servo_id == 6:  # 夹爪
            # 夹爪：将角度转换为米
            return self.degrees_to_meters(servo_angle)
    
    def publish_joint_states(self):
        """读取关节状态并发布给Piper手臂"""
        try:
            # 同步读取所有舵机状态
            servos_id = {name: servo_id for name, servo_id in zip(self.joint_names, self.servo_ids)}
            monitor_data = self.port_handler.sync_read["Monitor"](servos_id)
            
            # 创建JointState消息
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            
            # 获取当前角度并转换
            positions = []
            velocities = []
            for joint_name in self.joint_names:
                servo_id = servos_id[joint_name]
                servo_angle = monitor_data[joint_name].current_position
                joint_angle = self.servoangle2jointstate(servo_id, servo_angle)
                positions.append(joint_angle)
                velocities.append(100.0)
            
            joint_state_msg.position = positions
            joint_state_msg.velocity = velocities
            
            # 发布给Piper手臂
            self.joint_ctrl_pub.publish(joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(f"读取关节状态失败: {e}")
    
    def enable_torque(self):
        """使能力矩"""
        try:
            # 使用锁定模式使能力矩
            for servo_id in self.servo_ids:
                self.port_handler.write["Stop_On_Control_Mode"](servo_id, "locked", 0)
            self.get_logger().info("机械臂力矩已使能")
        except Exception as e:
            self.get_logger().error(f"使能力矩失败: {e}")
    
    def disable_torque(self):
        """禁用力矩"""
        try:
            # 使用解锁模式禁用力矩
            for servo_id in self.servo_ids:
                self.port_handler.write["Stop_On_Control_Mode"](servo_id, "unlocked", 900)
            self.get_logger().info("机械臂力矩已禁用")
        except Exception as e:
            self.get_logger().error(f"禁用力矩失败: {e}")
    
    def destroy_node(self):
        """节点销毁时清理"""
        self.disable_torque()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        driver_node = FashionStarDriver()
        rclpy.spin(driver_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点启动失败: {e}")
    finally:
        if 'driver_node' in locals():
            driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()