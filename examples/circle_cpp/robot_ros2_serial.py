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
from rclpy.node import Node
import rclpy

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
        self.sub_ctrl_vel_sub = self.create_subscription(Twist, 'sub_ctrl_vel', self.sub_ctrl_vel_callback, 10)
        self.pub_sensor_odom_pub = self.create_publisher(Odometry, 'pub_sensor_odom', 10)
        self.pub_sensor_imu_sub = self.create_subscription(Imu, 'pub_sensor_imu', self.pub_sensor_imu_callback, 10)

        # 设置协议回调函数（自动连接到ROS2发布者）
        self.robot_proto.sensor_odom.callback = self.sensor_odom_callback

        # 用定时器驱动协议处理
        self.timer = self.create_timer(0.001, self.timer_cb)  # 1ms tick一次
        
        self.get_logger().info("等待设备连接...")
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
