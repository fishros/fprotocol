#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FProtocol 串口服务端 + ROS2 集成
自动生成的Robot协议ROS2串口集成节点
"""

import serial
import time
from robot_proto import RobotProto
from fprotocol import *
import rclpy
from rclpy.node import Node
# from std_msgs.msg import String  # 示例消息类型，请根据需要修改

class SerialRobotNode(Node):
    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=115200, timeout=0.1):
        super().__init__('robot_serial_node')
        
        # 初始化串口
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=timeout)
            self.get_logger().info(f"串口 {serial_port} 连接成功，波特率: {baud_rate}")
        except serial.SerialException as e:
            self.get_logger().error(f"串口连接失败: {e}")
            exit(1)
        
        # 初始化协议处理器
        self.handler = FProtocol(self.read_callback, self.write_callback)
        self.robot_proto = RobotProto()
        
        # 设置协议节点
        self.handler.add_other_node(0x0001, self.robot_proto)

        # 示例：创建基本的发布者和订阅者（请根据需要修改）
        # self.example_pub = self.create_publisher(String, 'example_topic', 10)
        # self.example_sub = self.create_subscription(String, 'example_input', self.example_callback, 10)
        
        # 设置协议回调函数示例
        # self.robot_proto.led.callback = self.led_callback
        # self.robot_proto.config_wifi.callback = self.config_wifi_callback

        # 示例：可以在这里添加其他控制逻辑
        pass

        # 用定时器驱动协议处理
        self.timer = self.create_timer(0.001, self.timer_cb)  # 1ms tick一次
        
        self.get_logger().info("等待设备连接...")
        self.get_logger().info("-" * 50)

    # 示例回调方法（请根据实际需求修改）
    # def example_callback(self, msg):
    #     """处理ROS2订阅消息"""
    #     self.get_logger().info(f"收到消息: {msg.data}")
    #     pass

    def led_callback(self, type, from_node, error_code):
        """处理led协议数据回调"""
        self.get_logger().info(f"收到led数据回调，类型: {type}, 来源: {from_node}")
        # TODO: 在这里添加具体的数据处理逻辑
        # 协议数据可通过 self.robot_proto.led 访问
        return 0

    def config_wifi_callback(self, type, from_node, error_code):
        """处理config_wifi协议数据回调"""
        self.get_logger().info(f"收到config_wifi数据回调，类型: {type}, 来源: {from_node}")
        # TODO: 在这里添加具体的数据处理逻辑
        # 协议数据可通过 self.robot_proto.config_wifi 访问
        return 0

    def read_callback(self):
        """串口读取回调"""
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                return data
            return None
        except serial.SerialException as e:
            self.get_logger().error(f"串口读取错误: {e}")
            return None
    
    def write_callback(self, data):
        """串口写入回调"""
        try:
            self.ser.write(data)
            self.ser.flush()
        except serial.SerialException as e:
            self.get_logger().error(f"串口写入错误: {e}")
    
    def timer_cb(self):
        """定时器回调，处理协议tick"""
        # FProtocol处理
        self.handler.tick()
    
    def destroy_node(self):
        """节点销毁时关闭串口"""
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
        super().destroy_node()

def main():
    rclpy.init()
    
    # 可以通过参数自定义串口配置
    # node = SerialRobotNode(serial_port='/dev/ttyUSB0', baud_rate=9600, timeout=0.5)
    node = SerialRobotNode()  # 使用默认参数
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点关闭...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
