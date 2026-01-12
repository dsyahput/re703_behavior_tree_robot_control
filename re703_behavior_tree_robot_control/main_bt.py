#!/usr/bin/env python3

import rclpy
from re703_behavior_tree_robot_control.robot.robot_node import RobotBT

def main():
    rclpy.init()
    node = RobotBT()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()