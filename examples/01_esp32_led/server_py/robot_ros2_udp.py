#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FProtocol UDP服务端 + ROS2 集成
自动生成的Robot协议ROS2 UDP集成节点
"""

import socket
import time
from robot_proto import RobotProto
from fprotocol import *
import rclpy
from rclpy.node import Node
# from std_msgs.msg import String  # 示例消息类型，请根据需要修改

class UdpRobotNode(Node):
    def __init__(self, udp_host='0.0.0.0', udp_port=8888):
        super().__init__('robot_udp_node')
        
        # 初始化UDP套接字
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((udp_host, udp_port))
            self.sock.setblocking(False)  # 设置为非阻塞模式
            self.get_logger().info(f"UDP服务器启动成功，监听地址: {udp_host}:{udp_port}")
        except Exception as e:
            self.get_logger().error(f"UDP服务器启动失败: {e}")
            exit(1)
        
        # 存储客户端地址
        self.client_address = None
        
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
        
        self.get_logger().info("等待UDP客户端连接...")
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
        """UDP读取回调"""
        try:
            data, addr = self.sock.recvfrom(1024)
            # 记录客户端地址，用于后续发送数据
            if self.client_address != addr:
                self.client_address = addr
                self.get_logger().info(f"新的UDP客户端连接: {addr}")
            return data
        except socket.error:
            # 非阻塞模式下，没有数据时会抛出异常
            return None
        except Exception as e:
            self.get_logger().error(f"UDP读取错误: {e}")
            return None
    
    def write_callback(self, data):
        """UDP写入回调"""
        try:
            if self.client_address:
                self.sock.sendto(data, self.client_address)
            else:
                self.get_logger().warn("没有可用的UDP客户端地址，无法发送数据")
        except Exception as e:
            self.get_logger().error(f"UDP写入错误: {e}")
    
    def timer_cb(self):
        """定时器回调，处理协议tick"""
        # FProtocol处理
        self.handler.tick()
    
    def destroy_node(self):
        """节点销毁时关闭UDP套接字"""
        if hasattr(self, "sock"):
            self.sock.close()
            self.get_logger().info("UDP套接字已关闭")
        super().destroy_node()

def main():
    rclpy.init()
    
    # 可以通过参数自定义UDP配置
    # node = UdpRobotNode(udp_host='192.168.1.100', udp_port=9999)
    node = UdpRobotNode()  # 使用默认参数
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点关闭...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
