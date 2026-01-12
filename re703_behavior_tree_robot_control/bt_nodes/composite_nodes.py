from .base_nodes import BTNode

class Sequence(BTNode):
    def __init__(self, children):
        self.children = children
        self.current = 0

    def tick(self):
        while self.current < len(self.children):
            status = self.children[self.current].tick()

            if status == "RUNNING":
                return "RUNNING"

            if status == "FAILURE":
                self.current = 0
                return "FAILURE"

            if status == "SUCCESS":
                self.current += 1

        self.current = 0
        return "SUCCESS"

class Selector(BTNode):
    def __init__(self, children):
        self.children = children

    def tick(self):
        for child in self.children:
            status = child.tick()
            if status in ("RUNNING", "SUCCESS"):
                return status
        return "FAILURE"