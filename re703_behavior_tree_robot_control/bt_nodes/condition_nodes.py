from .base_nodes import BTNode

class BatteryLow(BTNode):
    def __init__(self, robot):
        self.robot = robot

    def tick(self):
        return "SUCCESS" if self.robot.battery <= self.robot.low_battery_threshold else "FAILURE"