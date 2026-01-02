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
from rclpy.node import Node
import rclpy

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
        self.sub_ctrl_vel_sub = self.create_subscription(Twist, 'sub_ctrl_vel', self.sub_ctrl_vel_callback, 10)
        self.pub_sensor_odom_pub = self.create_publisher(Odometry, 'pub_sensor_odom', 10)
        self.pub_sensor_imu_sub = self.create_subscription(Imu, 'pub_sensor_imu', self.pub_sensor_imu_callback, 10)

        # 设置协议回调函数（自动连接到ROS2发布者）
        self.robot_proto.sensor_odom.callback = self.sensor_odom_callback

        # 用定时器驱动协议处理
        self.timer = self.create_timer(0.001, self.timer_cb)  # 1ms tick一次
        
        self.get_logger().info("等待UDP客户端连接...")
        self.get_logger().info("-" * 50)

    def sensor_odom_callback(self, type, from_node, error_code):
        """处理sensor_odom协议数据并自动发布到ROS2话题 pub_sensor_odom"""
        # 协议数据已更新，现在转换并发布到ROS2
        msg = Odometry()
        now = self.get_clock().now().to_msg()
        
        # TODO: 将协议数据转换为ROS2消息
        # 协议数据可通过 self.robot_proto.sensor_odom 访问
        # 示例：
        # if hasattr(self.robot_proto.sensor_odom, 'x'):
        #     msg.pose.pose.position.x = float(self.robot_proto.sensor_odom.x)
        # msg.header.stamp = now
        # msg.header.frame_id = "base_link"
        
        self.pub_sensor_odom_pub.publish(msg)
        self.get_logger().info(f"发布pub_sensor_odom数据")
        return 0

    def sub_ctrl_vel_callback(self, msg):
        """处理ROS2 sub_ctrl_vel话题订阅并发送到设备"""
        # TODO: 将ROS2消息转换为协议数据并发送
        # 示例：
        # if hasattr(msg, 'linear') and hasattr(self.robot_proto, 'ctrl_vel'):
        #     self.robot_proto.ctrl_vel.x = msg.linear.x
        #     self.robot_proto.ctrl_vel.y = msg.linear.y
        #     self.robot_proto.ctrl_vel.z = msg.angular.z
        #     self.robot_proto.write_ctrl_vel(self.handler, FProtocolType.TRANSPORT_DATA, 0x0001)
        
        self.get_logger().info(f"收到sub_ctrl_vel消息，已发送到设备")

    def pub_sensor_imu_callback(self, msg):
        """处理ROS2 pub_sensor_imu话题订阅并发送到设备"""
        # TODO: 将ROS2消息转换为协议数据并发送
        # 示例：
        # if hasattr(msg, 'linear') and hasattr(self.robot_proto, 'sensor_imu'):
        #     self.robot_proto.sensor_imu.x = msg.linear.x
        #     self.robot_proto.sensor_imu.y = msg.linear.y
        #     self.robot_proto.sensor_imu.z = msg.angular.z
        #     self.robot_proto.write_sensor_imu(self.handler, FProtocolType.TRANSPORT_DATA, 0x0001)
        
        self.get_logger().info(f"收到pub_sensor_imu消息，已发送到设备")

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
