import math
from geometry_msgs.msg import Twist
from .base_nodes import BTNode

class CancelMove(BTNode):
    def __init__(self, robot):
        self.robot = robot
        self.done = False

    def tick(self):
        if not self.robot.goal_active:
            self.done = False
            return "SUCCESS"

        if not self.done:
            cancel_future = self.robot.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._after_cancel)
            self.done = True
            return "RUNNING"

        return "RUNNING"

    def _after_cancel(self, future):
        self.robot.goal_active = False
        self.robot.goal_handle = None
        self.done = False

class Navigate(BTNode):
    def __init__(self, robot, pos):
        self.robot = robot
        self.pos = pos
        self.sent = False

    def tick(self):
        if self.robot.goal_active:
            return "RUNNING"
        
        if self.sent and not self.robot.goal_active:
            self.sent = False
            return "SUCCESS"

        self.robot.last_goal_is_docking = (self.pos == self.robot.docking)

        x, y = self.pos
        self.robot.send_goal(x, y)
        self.sent = True
        return "RUNNING"

class BatteryCharging(BTNode):
    def __init__(self, robot):
        self.robot = robot

    def tick(self):
        if not self.robot.charging:
            return "FAILURE"

        self.robot.battery += self.robot.charging_rate
        if self.robot.battery >= 100:
            self.robot.battery = 100
            self.robot.charging = False
            return "SUCCESS"

        return "RUNNING"

class CheckingSurrounding(BTNode):
    def __init__(self, robot, duration_per_rotation=10.0, rotations=2):
        self.robot = robot
        self.duration_per_rotation = duration_per_rotation
        self.rotations = rotations

        self.started = False
        self.start_time = None
        self.twist_pub = robot.create_publisher(Twist, "/cmd_vel", 10)

    def stop_rotation(self):
        twist = Twist()
        twist.angular.z = 0.0
        self.twist_pub.publish(twist)

    def tick(self):
        if self.robot.last_goal_is_docking:
            self.robot.inspecting = False
            if self.started:
                self.stop_rotation()
            self.started = False
            return "SUCCESS"

        if not self.started:
            self.started = True
            self.robot.inspecting = True
            self.start_time = self.robot.get_clock().now()

        now = self.robot.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        total_required = self.duration_per_rotation * self.rotations

        if elapsed < total_required:
            twist = Twist()
            twist.angular.z = 2 * math.pi / self.duration_per_rotation
            self.twist_pub.publish(twist)

            self.robot.battery -= self.robot.rotation_drain_rate  
            if self.robot.battery < 0:
                self.robot.battery = 0

            return "RUNNING"

        self.stop_rotation()
        self.robot.inspecting = False
        self.started = False
        
        return "SUCCESS"