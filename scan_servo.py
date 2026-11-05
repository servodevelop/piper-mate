
import serial
import fashionstar_uart_sdk as uservo

# 串口配置
PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000



# 初始化串口
uart = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    timeout=0
)

try:
    servo = uservo.UartServoManager(uart)

    # 扫描所有舵机
    print("开始扫描舵机...")
    servo.scan_servo()


    # 打印扫描结果
    servo_ids = list(servo.servos.keys())
    print("找到舵机 ID:", servo_ids)

finally:
    uart.close()
