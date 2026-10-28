from numpy import deg2rad

class Goalie:
    def __init__(self, robot):
        self.robot = robot
        self.ball_detector = self.robot.ball_detector

        self.kp_x = 0.01  # Proportional gain for x-axis movement
        self.kp_y = deg2rad(47.64) / 960 # Proportional gain for y-axis movement

    def run(self):

        detection, _ = self.ball_detector.get_actual_detection(show_video=True)

        if detection:
            x, y, r = detection
            print(f"Ball detected at (x={x}, y={y}), radius={r}", end="\r")

            
            self.robot.look(min(float(deg2rad(29.5)), max(float(y * self.kp_y) + self.robot.get_head_pitch_angle(), -float(deg2rad(38.5)))), 0)

            vel = self.kp_x * -x
            if abs(vel) < 0.2:
                vel = 0.0

            self.robot.move(0, max(-1, min(1, vel)), 0, True)  # Move sideways based on ball's x position

        else:
            self.robot.look(float(deg2rad(-10)), 0)
            print("No ball detected.", end="\r")
            #self.robot.move(0, 0, 0, True)  # Stop moving if no ball is detected