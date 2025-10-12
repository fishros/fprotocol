#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
最简单的FProtocol UDP服务端
用于控制ESP32的LED
"""

import socket
import time
from robot_proto import RobotProto
from fprotocol import *
import fprotocol
print(fprotocol.__file__)

# UDP配置
host = '0.0.0.0'
port = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((host, port))
sock.settimeout(0.01)  # 设置超时，使接收非阻塞
handler = None
addr = None
last_heartbeat_time = time.time()
last_led_write_time = time.time()
led_write_interval = 0.3 
led_state = False

def read_callback():
    global addr
    try:
        data, addr = sock.recvfrom(128)
        # print(f"收到来自 {addr} 的数据: {list(data)}")
        return data
    except socket.timeout:
        return None

def write_callback(data):
    global addr
    # print(f"发送数据到 {addr} 长度={len(data)} -> ", list(data))
    if addr:
        sock.sendto(data, addr)


# 初始化
handler = FProtocol(read_callback, write_callback)
robot_proto = RobotProto()
robot_proto.led = 0  # 初始LED状态为关闭

# 添加ESP32节点
handler.add_other_node(0x0001, robot_proto)

print(f"UDP服务端启动，监听 {host}:{port}")
print("等待ESP32连接...")
print("服务端将每0.3秒发送一次LED状态（闪烁）")
print("-" * 50)

heartbeat_count = 0

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
            # print(f"[LED Write] 发送LED状态: {'开启' if led_state else '关闭'}")
        time.sleep(0.001)
        
except KeyboardInterrupt:
    print("服务端关闭...")
finally:
    sock.close()
