import argparse
import serial
import fashionstar_uart_sdk as uservo


import time
import math


# ==================== 参数配置 ====================

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000       # 你的 PiPER Mate 原程序使用 1000000

#6个关节+夹爪
SERVO_ID = [0,1,2,3,4,5,6]             # 0~5 六个关节，6 为夹爪

TARGET_ANGLE = 30.0      # 目标角度（度）
VELOCITY = 30.0          # 运动速度（度/秒）
POWER = 500              # 功率参数（mW）
DAMPING_POWER=1000

#关节限制
# 临时角度限制：来自实测观测范围
# 注意：并非厂家认证的安全限位
LIMIT_ANGLE = {
    0: (-123.7, 120.0),
    1: (0.9, 140),
    2: (-115.0, 180),
    3: (-90.4, 88.9),
    4: (-84.5, 83.0),
    5: (-115.2, 106.5),
    6: (-1.0, 90.0),
}
MAX_CHANGE_DEG=100

#默认目标角度
TARGET_ANGLES=[0.0,30.0,-30.0,0.0,0.0,0.0,30.0]

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
    "--velocity",
    type=float,
    default=VELOCITY,
    help="运动速度，默认30度/秒"
)
parser.add_argument(
    "--angle",
    type=float,
    nargs="+",
    default=None,
    help="目标角度列表，按 --id 指定的顺序对应；不填则使用默认目标数组"
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

        import math

        # ==================== 1. 确定目标角度 ====================

        if args.angle is None:
            # 没有输入 --angle，使用预设目标角度
            targets = {
                i: TARGET_ANGLES[i]
                for i in ids
            }

        else:
            # 命令行输入角度数量必须和舵机数量一致
            if len(args.angle) != len(ids):
                parser.error(
                    f"选择了 {len(ids)} 个舵机，"
                    f"但输入了 {len(args.angle)} 个角度"
                )

            # 将每个 ID 和角度一一对应
            targets = dict(zip(ids, args.angle))

        # ==================== 2. 检查所有目标 ====================

        for i in ids:

            # 检查通信
            if not servo.ping(i):
                raise RuntimeError(f"舵机{i}无响应")

            # 读取当前位置和对应的目标角度
            current = servo.query_servo_angle(i)
            target = targets[i]

            # 检查当前角度
            if current is None or not math.isfinite(current):
                raise RuntimeError(f"舵机{i}当前角度读取异常")

            # 检查目标角度
            if not math.isfinite(target):
                raise ValueError(f"舵机{i}目标角度无效")

            # 检查关节角度限制
            min_angle, max_angle = LIMIT_ANGLE[i]

            if not min_angle <= target <= max_angle:
                raise ValueError(
                    f"舵机{i}目标角度{target:.2f}°超出限制"
                    f"[{min_angle}, {max_angle}]"
                )

            # 检查单次角度变化
            delta = abs(target - current)

            if delta > MAX_CHANGE_DEG:
                raise ValueError(
                    f"舵机{i}当前{current:.2f}°，"
                    f"目标{target:.2f}°，"
                    f"变化{delta:.2f}°超过"
                    f"{MAX_CHANGE_DEG}°"
                )

            print(
                f"舵机{i}: "
                f"当前={current:.2f}°，"
                f"目标={target:.2f}°"
            )

        
        # ==================== 3. 发送运动指令 ====================

        for i in ids:
            servo.set_servo_angle(
                i,
                angle=targets[i],
                velocity=args.velocity
            )
            print(f"舵机{i}发送目标: {targets[i]:.2f}°")

        # ==================== 4. 等待并检查反馈 ====================

        TIMEOUT = 12.0       # 最长等待12秒
        TOLERANCE = 2.0      # 允许角度误差2度
        CHECK_INTERVAL = 0.2 # 每0.2秒查询一次

        start_time = time.monotonic()
        reached = {i: False for i in ids}

        while time.monotonic() - start_time < TIMEOUT:

            all_reached = True

            for i in ids:
                try:
                    actual = servo.query_servo_angle(i)

                    if actual is None or not math.isfinite(actual):
                        raise RuntimeError("无效角度反馈")

                except Exception as e:
                    print(f"舵机{i}读取失败: {e}")
                    all_reached = False
                    reached[i] = False
                    continue

                target = targets[i]
                error = abs(target - actual)

                reached[i] = error <= TOLERANCE

                if not reached[i]:
                    all_reached = False

                print(
                    f"ID={i} "
                    f"目标={target:.1f}° "
                    f"实际={actual:.1f}° "
                    f"误差={error:.1f}°"
                )

            print("-----------------------")

            if all_reached:
                print("所有舵机达到角度容差范围")
                break

            time.sleep(CHECK_INTERVAL)

        else:
            print("等待超时，以下舵机尚未达到目标：")

            for i in ids:
                if not reached[i]:
                    print(f"舵机{i}: 目标={targets[i]:.1f}°")





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
            print("角度:",servo.query_servo_angle(i))
            print("电压:",servo.query_voltage(i), "V")
            print("电流:",servo.query_current(i), "A")
            print("功率:",servo.query_power(i), "W")
            print("温度ADC:",servo.query_temperature(i))
            print("状态:",servo.query_status(i))

    # 9. 综合监控
    elif args.mode == "monitor":

        for i in ids:
            data = servo.query_servo_monitor(i)
            print(f"舵机{i}: {data}")

finally:
    uart.close()
