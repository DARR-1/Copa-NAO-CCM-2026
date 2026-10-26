from numpy import deg2rad

class Goalie:
    def __init__(self, robot):
        self.robot = robot
        self.ball_detector = self.robot.ball_detector

        self.kp_x = 0.001  # Proportional gain for x-axis movement
        self.kp_y = deg2rad(47.64) / 960 # Proportional gain for y-axis movement

    def run(self):
        detection, _ = self.ball_detector.get_actual_detection(show_video=True)

        if detection:
            x, y, r = detection
            print(f"Ball detected at (x={x}, y={y}), radius={r}", end="\r")
            
            self.robot.look(deg2rad(y * self.kp_y), 0)

            self.robot.move(0, self.kp_x * -x, 0)  # Move sideways based on ball's x position

        else:
            print("No ball detected.", end="\r")
            self.robot.move(0, 0, 0)  # Stop moving if no ball is detected