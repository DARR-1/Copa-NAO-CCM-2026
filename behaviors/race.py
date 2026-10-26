class Race:
    def __init__(self, robot):
        self.robot = robot
        self.state = "start"

    def run(self):
        if self.state == "start":
            self.robot.move_to(5.0, 0.0, 0.0)  # Move forward 5 meters
            self.robot.rest()
            self.state = "finished"