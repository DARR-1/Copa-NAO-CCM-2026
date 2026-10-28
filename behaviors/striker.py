from enum import Enum

class State(Enum):
    SEARCH = 1
    APPROACH = 2
    READ = 3
    ALIGN = 4
    SHOOT = 5
    DONE = 6

class Striker:
    def __init__(self, robot):
        self.robot = robot
        self.ball_detector = self.robot.ball_detector
        self.kick = self.robot.kick
        self.state = State.SEARCH

    def run(self):
        ...

    def is_done(self):
        if self.state == State.DONE:
            return True

    def _search(self):
        ...

    def _approach(self):
        ...

    def _read(self):
        ...

    def _align(self):
        ...

    def _shoot(self):
        ...
