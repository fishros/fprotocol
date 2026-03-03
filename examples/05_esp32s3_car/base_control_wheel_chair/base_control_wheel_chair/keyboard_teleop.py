#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.linear_step = 0.1
        self.angular_step = 0.3
        self.max_linear = 1.0
        self.max_angular = 2.0
        self.linear = 0.0
        self.angular = 0.0
        self.stop_requested = False
        self.timer = self.create_timer(0.05, self._on_timer)
        self._print_help()

    def _print_help(self):
        self.get_logger().info(
            "Keyboard control: w/s linear, a/d angular, space stop, q quit"
        )

    def _read_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def _on_timer(self):
        key = self._read_key()
        if key:
            if key == "w":
                self.linear = min(self.linear + self.linear_step, self.max_linear)
            elif key == "s":
                self.linear = max(self.linear - self.linear_step, -self.max_linear)
            elif key == "a":
                self.angular = min(self.angular + self.angular_step, self.max_angular)
            elif key == "d":
                self.angular = max(self.angular - self.angular_step, -self.max_angular)
            elif key == " ":
                self.linear = 0.0
                self.angular = 0.0
            elif key == "q":
                self.stop_requested = True

        msg = Twist()
        msg.linear.x = float(self.linear)
        msg.angular.z = float(self.angular)
        self.pub.publish(msg)

        if key in ("w", "a", "s", "d", " "):
            self.get_logger().info(
                f"cmd_vel => linear.x={self.linear:.2f}, angular.z={self.angular:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    node = KeyboardTeleop()
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        stop_msg = Twist()
        node.pub.publish(stop_msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
