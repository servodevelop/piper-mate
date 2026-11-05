
import argparse
from fashionstar_uart_sdk.uart_pocket_handler import PortHandler

# ========== 参数配置 ==========
PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000

# 六个关节 + 夹爪
SERVO_IDS = [0, 1, 2, 3, 4, 5, 6]

# ========== 命令行参数 ==========
parser = argparse.ArgumentParser()
parser.add_argument(
    "mode",
    choices=["lock", "unlock"],
    help="lock=锁定, unlock=释放"
)
args = parser.parse_args()

# ========== 初始化 ==========
handler = PortHandler(PORT, BAUDRATE)
handler.openPort()

try:
    if args.mode == "lock":

        # 读取锁定前的角度
        names = {f"servo{i}": i for i in SERVO_IDS}
        data = handler.sync_read["Monitor"](names)

        print("锁定前的角度:")
        for name in names:
            print(f"{name}: {data[name].current_position:.2f}°")

        # 锁定全部舵机
        for servo_id in SERVO_IDS:
            handler.write["Stop_On_Control_Mode"](
                servo_id, "locked", 0
            )
            print(f"舵机 {servo_id} 已发送锁定命令")

        print("锁定命令发送完成")

    elif args.mode == "unlock":

        # 释放全部舵机
        for servo_id in SERVO_IDS:
            handler.write["Stop_On_Control_Mode"](
                servo_id, "unlocked", 900
            )
            print(f"舵机 {servo_id} 已发送解锁命令")

        print("解锁命令发送完成")

finally:
    handler.closePort()
