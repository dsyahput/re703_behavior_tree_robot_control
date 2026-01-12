import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

from re703_behavior_tree_robot_control.bt_nodes.composite_nodes import Selector, Sequence
from re703_behavior_tree_robot_control.bt_nodes.condition_nodes import BatteryLow
from re703_behavior_tree_robot_control.bt_nodes.action_nodes import (
    CancelMove, Navigate, BatteryCharging, CheckingSurrounding
)

class RobotBT(Node):
    def __init__(self):
        super().__init__("robot_bt")

        self.battery = 100.0
        self.low_battery_threshold = 50.0
        self.charging_rate = 0.5
        self.battery_drain_rate = 1
        self.rotation_drain_rate = 0.03
        self.charging = False

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_active = False
        self.goal_handle = None
        self.last_goal_is_docking = False

        self.last_pos = None
        self.last_time = self.get_clock().now()

        self.room1 = (-7.3, -0.09)
        self.room2 = (-5.2, -3.3)
        self.room3 = (0.1, 3.0)
        self.room4 = (5.8, -2.4)
        self.room5 = (8.3, 1.5)
        self.docking = (0.0, 0.0)

        self.tree = Selector([
            Sequence([
                BatteryLow(self),
                CancelMove(self),
                Navigate(self, self.docking),
                BatteryCharging(self)
            ]),
            
            Sequence([
                Navigate(self, self.room1),
                CheckingSurrounding(self),
                Navigate(self, self.room2),
                CheckingSurrounding(self),
                Navigate(self, self.room3),
                CheckingSurrounding(self),
                Navigate(self, self.room4),
                CheckingSurrounding(self),
                Navigate(self, self.room5),
                CheckingSurrounding(self),
            ])
        ])

        self.nav_client.wait_for_server()

        self.create_subscription(Odometry, "/bumperbot_controller/odom", self.odom_callback, 10)
        self.timer = self.create_timer(0.25, self.update)

    def odom_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        if self.last_pos and self.goal_active:
            dx = px - self.last_pos[0]
            dy = py - self.last_pos[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist > 1e-6:
                self.battery -= dist * self.battery_drain_rate
                self.battery = max(self.battery, 0)

        self.last_pos = (px, py)

    def send_goal(self, x, y):
        if self.goal_active:
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.goal_active = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.goal_active = False
            return

        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.goal_result)

    def goal_result(self, future):
        self.goal_active = False
        self.goal_handle = None

        if self.battery <= self.low_battery_threshold:
            self.charging = True

    def update(self):
        status = self.tree.tick()
        activity = "Idle"

        if self.goal_active:
            activity = f"Navigating ({'Docking' if self.last_goal_is_docking else 'Waypoint'})"
        elif self.charging:
            activity = "Charging"
        elif self.inspecting:
            activity = "Inspecting Surroundings"

        self.get_logger().info(f"[BT] {activity} | Battery: {self.battery:.1f}%")