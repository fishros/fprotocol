#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FProtocol 串口服务端 + ROS2 集成
自动生成的Robot协议ROS2串口集成节点
"""

import serial
import time
import argparse
import json
from robot_proto import RobotProto
from fprotocol import *
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy


class SerialRobotNode(Node):
    def __init__(self, serial_port="/dev/ttyACM0", baud_rate=115200, timeout=0.1):
        super().__init__("robot_serial_node")

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
        self.handler.set_self_node(0x0002, self.robot_proto)
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # 设置协议回调函数（自动连接到ROS2发布者）
        # 用定时器驱动协议处理
        self.timer = self.create_timer(0.001, self.timer_cb)  # 1ms tick一次

        self.get_logger().info("等待设备连接...")
        self.get_logger().info("-" * 50)

    def cmd_vel_callback(self, msg):
        """处理ROS2 cmd_vel话题订阅并发送到设备"""
        if not hasattr(self.robot_proto, "cmd_vel"):
            self.get_logger().warning("协议对象不存在字段 cmd_vel")
            return

        pdata = self.robot_proto.cmd_vel
        if hasattr(msg, "linear") and hasattr(msg, "angular"):
            if hasattr(pdata, "x"):
                pdata.x = float(msg.linear.x)
            if hasattr(pdata, "y"):
                pdata.y = float(msg.linear.y)
            if hasattr(pdata, "z"):
                pdata.z = float(msg.angular.z)

        self.robot_proto.write_cmd_vel(self.handler, FProtocolType.TRANSPORT_DATA, 0xFF)
        self.get_logger().info(f"收到cmd_vel消息，已广播发送到设备")

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

    def nav_target_callback(self, msg_type, fdata, error_code):
        """处理下位机上报的 nav_opt_t(JSON字符串)"""
        try:
            payload = fdata.data
            if isinstance(payload, tuple):
                payload = payload[0] if payload else b""
            elif isinstance(payload, str):
                payload = payload.encode("utf-8", errors="ignore")

            data_size = int(getattr(fdata, "_data_size", 0))
            raw = bytes(payload[:data_size]) if data_size > 0 else bytes(payload)
            text = raw.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")

            if text:
                obj = json.loads(text)
                self.get_logger().info(f"收到 nav_target: {obj}")
            else:
                self.get_logger().info("收到 nav_target: 空数据")
        except Exception as e:
            self.get_logger().warning(f"nav_target 解析失败: {e}")
        return 0

    def destroy_node(self):
        """节点销毁时关闭串口"""
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("serial_port", nargs="?", default="/dev/ttyUSB0")
    parser.add_argument("baud_rate", nargs="?", type=int, default=115200)
    args = parser.parse_args()

    rclpy.init()
    node = SerialRobotNode(serial_port=args.serial_port, baud_rate=args.baud_rate)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点关闭...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
