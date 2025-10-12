#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FProtocol 串口服务端 + ROS2 Odom 发布
用于控制ESP32的LED，并通过ros2 odom话题发布里程计数据
FProtocol和串口都集成在ros2 node，用定时器(timer)去tick
"""

import serial
import time
from robot_proto import RobotProto
from fprotocol import *

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
TIMEOUT = 0.1

class SerialFProtocolOdomNode(Node):
    def __init__(self):
        super().__init__('esp32_odom_serial_node')

        # 初始化串口
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
            self.get_logger().info(f"串口 {SERIAL_PORT} 连接成功，波特率: {BAUD_RATE}")
        except serial.SerialException as e:
            self.get_logger().error(f"串口连接失败: {e}")
            exit(1)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.last_led_write_time = self.get_clock().now().nanoseconds / 1e9
        self.led_write_interval = 0.25
        self.led_state = False

        self.handler = FProtocol(self.read_callback, self.write_callback)
        self.robot_proto = RobotProto()
        self.robot_proto.led = 0

        # Odom 回调: 收到串口里程计数据后通过ROS2 odom话题发布

        self.robot_proto.odom.callback = self.odom_callback
        self.handler.add_other_node(0x0001, self.robot_proto)

        self.get_logger().info("等待ESP32连接...")
        self.get_logger().info("服务端将每0.3秒发送一次LED状态（闪烁）")
        self.get_logger().info("收到ESP32里程计数据后会通过 ROS2 /odom 话题发布")
        self.get_logger().info("-" * 50)

        # 用定时器驱动协议tick和LED状态发送
        self.timer = self.create_timer(0.001, self.timer_cb)  # 1ms tick一次

    def odom_callback(self, type, from_node, error_code):
        self.publish_odom(self.robot_proto.odom.x, self.robot_proto.odom.y, self.robot_proto.odom.z)
        # self.get_logger().info(f"[ODOM] ROS2发布: x={self.robot_proto.odom.x:.3f}, y={self.robot_proto.odom.y:.3f}, z={self.robot_proto.odom.z:.3f}")
        return 0

    def read_callback(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                # self.get_logger().debug(f"串口读取数据: {data.hex(' ')}")
                return data
            return None
        except serial.SerialException as e:
            self.get_logger().error(f"串口读取错误: {e}")
            return None

    def write_callback(self, data):
        try:
            self.ser.write(data)
            self.ser.flush()
            # self.get_logger().debug(f"串口写入数据: {data.hex(' ')}")
        except serial.SerialException as e:
            self.get_logger().error(f"串口写入错误: {e}")

    def publish_odom(self, x, y, z):
        msg = Odometry()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        msg.pose.pose.orientation = Quaternion()
        self.odom_pub.publish(msg)

    def timer_cb(self):
        # FProtocol处理
        self.handler.tick()

        # 控制LED的闪烁发送
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_led_write_time >= self.led_write_interval:
            self.led_state = not self.led_state
            self.robot_proto.led = 1 if self.led_state else 0
            self.robot_proto.write_led(self.handler, FProtocolType.TRANSPORT_DATA, 0x0001)
            self.last_led_write_time = now
            self.get_logger().info(f"[LED Write] 发送LED状态: {'开启' if self.led_state else '关闭'}")

    def destroy_node(self):
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
        super().destroy_node()

def main():
    rclpy.init()
    node = SerialFProtocolOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("服务端关闭...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
