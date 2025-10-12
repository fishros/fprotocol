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

# 串口配置
SERIAL_PORT = '/dev/ttyACM0'  # Windows串口号，Linux/Mac使用 '/dev/ttyUSB0' 或类似
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

# odom数据接收相关
last_odom_print_time = time.time()
odom_print_interval = 0.1  # 每100ms打印一次odom数据
odom_packet_count = 0  # 接收到的odom数据包计数器

def read_callback():
    global ser
    """串口读回调函数"""
    try:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            # print(f"串口读取数据: {data.hex(' ')}")
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
        ser.flush()  # 确保数据发送完成
        # print(f"串口写入数据: {data.hex(' ')}")
    except serial.SerialException as e:
        print(f"串口写入错误: {e}")

def odom_callback(type, from_node, error_code):
    """odom数据接收回调函数"""
    global robot_proto, last_odom_print_time, odom_print_interval, odom_packet_count
    current_time = time.time()
    
    # 增加计数器
    odom_packet_count += 1
    
    # 打印所有接收到的数据，不限制频率
    print(f"[ODOM #{odom_packet_count}] 接收到odom数据: x={robot_proto.odom.x:.3f}, y={robot_proto.odom.y:.3f}, z={robot_proto.odom.z:.3f}")
    return 0

# 初始化
handler = FProtocol(read_callback, write_callback)
robot_proto = RobotProto()
robot_proto.led = 0  # 初始LED状态为关闭

# 设置odom数据的回调函数
robot_proto.odom.callback = odom_callback
print(f"[DEBUG] 设置odom回调函数: {robot_proto.odom.callback}")

# 添加ESP32节点
handler.add_other_node(0x0001, robot_proto)

print(f"串口服务端启动，端口: {SERIAL_PORT}")
print("等待ESP32连接...")
print("服务端将每0.3秒发送一次LED状态（闪烁）")
print("服务端将接收并打印ESP32发送的odom数据（100Hz）")
print("-" * 50)

try:
    while True:
        # 使用FProtocol的tick方法处理数据
        handler.tick()
        
        # 监控缓冲区状态
        if odom_packet_count > 0 and odom_packet_count % 100 == 0:
            print(f"[STATUS] 已接收 {odom_packet_count} 个odom数据包，缓冲区状态: {ser.in_waiting} 字节待读，FProtocol缓冲区: {handler.rxbuff.size()} 字节")
        
        # # 每0.3秒发送一次LED状态
        # if time.time() - last_led_write_time >= led_write_interval:
        #     led_state = not led_state  # 切换LED状态
        #     robot_proto.led = 1 if led_state else 0
        #     robot_proto.write_led(handler, FProtocolType.TRANSPORT_DATA, 0x0001)
        #     last_led_write_time = time.time()
        #     print(f"[LED Write] 发送LED状态: {'开启' if led_state else '关闭'}")
        
        # 完全移除sleep，让数据处理达到最高速度
        # if ser.in_waiting == 0:
        time.sleep(0.0001)  # 100微秒
except KeyboardInterrupt:
    print("服务端关闭...")
finally:
    if ser.is_open:
        ser.close()
        print("串口已关闭")
