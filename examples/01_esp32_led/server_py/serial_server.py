#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FProtocol 串口服务端
用于控制ESP32的LED
"""

import serial
import time
from robot_proto import RobotProto
from fprotocol import *
import fprotocol

# 串口配置
SERIAL_PORT = 'COM14'  # Windows串口号，Linux/Mac使用 '/dev/ttyUSB0' 或类似
BAUD_RATE = 115200
TIMEOUT = 0.1  # 改为100ms，确保数据接收完整

# 初始化串口
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    print(f"串口 {SERIAL_PORT} 连接成功，波特率: {BAUD_RATE}")
except serial.SerialException as e:
    print(f"串口连接失败: {e}")
    print("请检查串口号是否正确，并确保ESP32已连接")
    exit(1)

handler = None
last_led_write_time = time.time()
led_write_interval = 0.25
led_state = False

def read_callback():
    global ser
    """串口读回调函数"""
    try:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            return data
        return None
    except serial.SerialException as e:
        print(f"串口读取错误: {e}")
        return None

def write_callback(data):
    """串口写回调函数"""
    global ser
    try:
        ser.write(data)
        # ser.write('hello world~\n'.encode('utf-8'))
        ser.flush()  # 确保数据发送完成
        print(f"串口写入数据: {data.hex(' ')}")
    except serial.SerialException as e:
        print(f"串口写入错误: {e}")

# 初始化
handler = FProtocol(read_callback, write_callback)
robot_proto = RobotProto()
robot_proto.led = 0  # 初始LED状态为关闭

# 添加ESP32节点
handler.add_other_node(0x0001, robot_proto)

print(f"串口服务端启动，端口: {SERIAL_PORT}")
print("等待ESP32连接...")
print("服务端将每0.3秒发送一次LED状态（闪烁）")
print("-" * 50)

try:
    while True:
        # 使用FProtocol的tick方法处理数据
        handler.tick()
        
        # 每0.3秒发送一次LED状态
        if time.time() - last_led_write_time >= led_write_interval:
            led_state = not led_state  # 切换LED状态
            robot_proto.led = 1 if led_state else 0
            robot_proto.write_led(handler, FProtocolType.TRANSPORT_DATA, 0x0001)
            last_led_write_time = time.time()
            print(f"[LED Write] 发送LED状态: {'开启' if led_state else '关闭'}")
        
        time.sleep(0.001)
        
except KeyboardInterrupt:
    print("服务端关闭...")
finally:
    if ser.is_open:
        ser.close()
        print("串口已关闭")
