#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试RobotProto数据发送
模拟给config_wifi_t赋值并查看最终发送的字节
"""

import struct
from robot_proto import RobotProto
from fprotocol import *

def test_robot_proto():
    """测试RobotProto的数据发送功能"""
    print("=== RobotProto 测试 ===")
    
    # 创建RobotProto实例
    robot_proto = RobotProto()
    
    # 测试LED数据
    print("\n1. 测试LED数据:")
    robot_proto.led = 1
    print(f"LED状态: {robot_proto.led}")
    
    # 模拟FProtocol的write_callback来捕获发送的数据
    sent_data = []
    
    def mock_write_callback(data):
        sent_data.append(data)
        print(f"发送数据: {data.hex(' ')} (长度: {len(data)} 字节)")
    
    def mock_read_callback():
        return None
    
    # 创建FProtocol实例
    handler = FProtocol(mock_read_callback, mock_write_callback)
    
    # 添加节点
    handler.add_other_node(0x0001, robot_proto)
    
    # 发送LED数据
    print("\n2. 发送LED数据:")
    robot_proto.write_led(handler, FProtocolType.TRANSPORT_DATA, 0x0001)
    
    # 测试WiFi配置数据
    print("\n3. 测试WiFi配置数据:")
    
    # 设置WiFi配置
    robot_proto.config_wifi.name = 'fishrosfishrosfishrosfishrosfishrosfishrosfishrosfishrosfishrosfishrosfishrosfishrosfishrosfishros'
    robot_proto.config_wifi.pswd = '88888888'
    
    print(f"WiFi名称: {robot_proto.config_wifi.name}")
    print(f"WiFi密码: {robot_proto.config_wifi.pswd}")
    
    # 查看config_wifi的内部结构
    print(f"\nconfig_wifi结构:")
    print(f"  _name_size: {robot_proto.config_wifi._name_size}")
    print(f"  name: {robot_proto.config_wifi.name}")
    print(f"  _pswd_size: {robot_proto.config_wifi._pswd_size}")
    print(f"  pswd: {robot_proto.config_wifi.pswd}")
    
    # 发送WiFi配置数据
    print("\n4. 发送WiFi配置数据:")
    robot_proto.write_config_wifi(handler, FProtocolType.TRANSPORT_DATA, 0x0001)
    
    # 显示所有发送的数据
    print("\n5. 所有发送的数据汇总:")
    for i, data in enumerate(sent_data, 1):
        print(f"数据包 {i}: {data.hex(' ')}")
        print(f"  长度: {len(data)} 字节")
        print(f"  解析: {data}")
        print()

if __name__ == "__main__":
    test_robot_proto()