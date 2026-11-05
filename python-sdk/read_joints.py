
import time
from fashionstar_uart_sdk.uart_pocket_handler import PortHandler

# ========== 参数配置 ==========
PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000

# 六个关节 + 一个夹爪
SERVO_IDS = {
    "joint1": 0,
    "joint2": 1,
    "joint3": 2,
    "joint4": 3,
    "joint5": 4,
    "joint6": 5,
    "gripper": 6
}

# ========== 初始化 ==========
handler = PortHandler(PORT, BAUDRATE)
handler.openPort()

print("PiPER Mate 连接成功")

try:
    while True:
        # 同步读取全部舵机
        data = handler.sync_read["Monitor"](SERVO_IDS)

        # 输出每个关节的角度
        for name in SERVO_IDS:
            angle = data[name].current_position
            print(f"{name}: {angle:.2f}°", end="  ")

        print()
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\n停止读取")

finally:
    handler.closePort()
