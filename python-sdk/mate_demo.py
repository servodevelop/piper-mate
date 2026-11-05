import argparse
import serial
import fashionstar_uart_sdk as uservo

# ==================== 参数配置 ====================

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000       # 你的 PiPER Mate 原程序使用 1000000

#6个关节+夹抓
SERVO_ID = [0,1,2,3,4,5,6]             # 0~5 六个关节，6 为夹爪


TARGET_ANGLE = 30.0      # 目标角度（度）
VELOCITY = 30.0          # 运动速度（度/秒）
POWER = 500              # 功率参数（mW）
DAMPING_POWER=1000

# =================== 命令行解析 ===================
parser = argparse.ArgumentParser()
parser.add_argument(
    "mode",
    choices=["ping","read","move","hold","damping","lock", "unlock","status",
    "monitor","read_all"],
    help="ping,read,move,hold,damping,lock,unlock,status,monitor,read_all"
)
parser.add_argument(
    "--id",
    type=int,
    nargs="+",
    help="指定舵机ID，可输入多个，例如 --id 0 1 6"
)
parser.add_argument(
    "--all",
    action="store_true",
    help="对全部7个舵机操作"
)
parser.add_argument(
    "--angle",
    type=float,
    default=TARGET_ANGLE,
    help="目标角度，默认30度"
)
parser.add_argument(
    "--velocity",
    type=float,
    default=VELOCITY,
    help="运动速度，默认30度/秒"
)
args = parser.parse_args()


# 可选：
# ping      检查通信
# read      读取当前角度
# move      移动到指定角度
# hold      锁定当前位置
# damping   阻尼模式
# unlock    释放力矩
# status    查询电压、电流等状态
# monitor   读取舵机综合监控数据
# read_all  读取全部六关节及夹爪

# ==================命令行要求======================

if args.all and args.id is not None:
    parser.error("--all 和 --id 不能同时使用")

if args.id is not None:
    ids = args.id
elif args.all or args.mode in ("ping", "read_all"):
    ids = SERVO_ID
else:
    ids = [0]  # 默认只控制0号舵机

if len(ids) != len(set(ids)):
    parser.error("舵机 ID 不能重复")

if any(i not in SERVO_ID for i in ids):
    parser.error("舵机ID必须在0~6之间")

if args.velocity <= 0:
    parser.error("速度必须大于0")


# ==================== 初始化 SDK ====================

uart = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    timeout=0
)
servo = uservo.UartServoManager(uart)

try:
    # 1. 通信检测
    if args.mode == "ping":
        for i in ids:
             print(f"舵机{i}:", servo.ping(i))

    
    # 2. 读取角度
    elif args.mode in ("read", "readall"):
        for i in ids:
            angle = servo.query_servo_angle(i)
            print(f"舵机 {i}: {angle:.2f}°")

    # 3. 设置目标角度
    elif args.mode == "move":

        # 先确认所有目标舵机都能通信
        for i in ids:
            if not servo.ping(i):
                raise RuntimeError(f"舵机{i}无响应")

        # 先检查所有目标，再下发指令
        for i in ids:
            current = servo.query_servo_angle(i)

            if abs(args.angle - current) > 30:
                raise ValueError(
                    f"舵机{i}当前{current:.2f}°, "
                    f"目标{args.angle:.2f}°, "
                    "变化超过30度"
                )
        # 逐个发送位置命令
        for i in ids:
            servo.set_servo_angle(
                i,
                angle=args.angle,
                velocity=args.velocity
            )
            print(f"舵机{i}目标角度: {args.angle}°")

        servo.wait(timeout=5000)


     # 4. 保持当前位置
    elif args.mode == "hold":

        # 先读取所有当前角度
        angles = {}

        for i in ids:
            if not servo.ping(i):
                raise RuntimeError(f"舵机{i}无响应")

            angles[i] = servo.query_servo_angle(i)

        # 位置控制，目标设为当前位置
        for i in ids:
            servo.set_servo_angle(
                i,
                angle=angles[i],
                velocity=args.velocity
            )
            print(f"舵机{i}保持角度: {angles[i]:.2f}°")


    # 5. 阻尼模式
    elif args.mode == "damping":
        for i in ids:
            servo.set_damping(i, power=DAMPING_POWER)
            print(f"舵机{i}已发送阻尼模式命令")

    
    # 6. 锁定力矩
    elif args.mode == "lock":
        for i in ids:
            servo.stop_on_control_mode(
                i,
                method=0x11,
                power=POWER
            )
            print(f"舵机{i}已发送锁力指令")

   # 7. 释放力矩
    elif args.mode == "unlock":
        for i in ids:
            servo.stop_on_control_mode(
                i,
                method=0x10,
                power=POWER
            )
            print(f"舵机{i}已发送卸力指令")

    # 8. 查询状态
    elif args.mode == "status":

        for i in ids:
            print(f"\n===== 舵机{i} =====")
            print("角度:",
                  servo.query_servo_angle(i))
            print("电压:",
                  servo.query_voltage(i), "V")
            print("电流:",
                  servo.query_current(i), "A")
            print("功率:",
                  servo.query_power(i), "W")
            print("温度ADC:",
                  servo.query_temperature(i))
            print("状态:",
                  servo.query_status(i))

    # 9. 综合监控
    elif args.mode == "monitor":

        for i in ids:
            data = servo.query_servo_monitor(i)
            print(f"舵机{i}: {data}")

finally:
    uart.close()
