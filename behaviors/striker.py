from enum import Enum
import random
import time

SEARCH_TIMEOUT = 2.0 #seconds
SETTLE_TIME = 0.4
HITS_NEEDED = 3

HEAD_PITCH = 0.35  # rad, positivo = hacia abajo (máx ~0.51)
HEAD_SETTLE_TIME = 0.8  # seconds for the head to finish moving

STOP_Y = 100
STOP_HITS = 2
LOST_LIMIT = 5

FWD_SPEED = 0.5
TURN_GAIN = 0.8

TURN_ANGLE = 0.25
KICK_X = {"left": -60, "right": 60}
KICK_Y = 90
TOL_X = 20
TOL_Y = 20
MAX_ADJUSTS = 8
M_PER_PX = 0.00075  # m per pixel near the kick spot (rough estimate, calibrate)

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
        self.goalkeeper_detector = self.robot.goalkeeper_detector
        self.kick = self.robot.kick

        # Starting state
        self.state = State.SEARCH
        self.state_start = time.monotonic()

        # Bottom camera, head tilted down
        self.ball_detector.camera.set_camera(1)
        self.robot.look(HEAD_PITCH, 0)
        self.hits = 0

        # Stopping hits for approach
        self.stop_hits = 0
        self.lost = 0

        self.turned = False
        self.adjusts = 0
        self.target = None
        self.foot = None

        self.last_detection = None

    def run(self):
        if self.state == State.SEARCH:
            self._search()
        elif self.state == State.APPROACH:
            self._approach()
        elif self.state == State.READ:
            self._read()
        elif self.state == State.ALIGN:
            self._align()
        elif self.state == State.SHOOT:
            self._shoot()

    def is_done(self):
        if self.state == State.DONE:
            return True
        else:
            return False

    def _search(self):
        elapsed = time.monotonic() - self.state_start

        if elapsed < SETTLE_TIME:
            return

        detection, _ = self.ball_detector.get_actual_detection()

        if detection is not None:
            self.hits += 1
            if self.hits >= HITS_NEEDED:
                self.last_detection = detection

                #for approach to reset, subject to change
                self.stop_hits = 0
                self.lost = 0

                self.state = State.APPROACH
                self.state_start = time.monotonic()
            return

        self.hits = 0

        if elapsed > SEARCH_TIMEOUT:
            self.robot.stop_move()
            self.robot.say("No puedo encontrar la pelota")
            self.state = State.DONE


    def _approach(self):
        detection, _ = self.ball_detector.get_actual_detection()

        if detection is None:
            self.lost += 1
            if self.lost >= LOST_LIMIT:
                self.robot.stop_move()
                if self.last_detection[1] > STOP_Y - 30:
                    self.state = State.READ
                else:
                    self.hits = 0
                    self.state = State.SEARCH
                self.state_start = time.monotonic()
            return

        self.lost = 0
        self.last_detection = detection
        x, y, _ = detection

        if y >= STOP_Y:
            self.stop_hits += 1
            if self.stop_hits >= STOP_HITS:
                self.robot.stop_move()
                self.state = State.READ
                self.state_start = time.monotonic()
            return

        self.stop_hits = 0

        w = self.ball_detector.frame_size[0]
        v_theta = -TURN_GAIN * x / (w / 2.0)
        self.robot.move(FWD_SPEED, 0, v_theta, True)

    def _read(self):
        # Head straight forward so the top camera sees the goalie; wait so head motion isn't read as goalie motion
        self.robot.look(0, 0)
        time.sleep(HEAD_SETTLE_TIME)

        # Blocking (~1 s): reads the goalie with the top camera, then switches back to the bottom one
        movement = self.goalkeeper_detector.get_movement()

        # Head back down for the bottom camera in ALIGN
        self.robot.look(HEAD_PITCH, 0)
        time.sleep(HEAD_SETTLE_TIME)

        # Shoot to the side opposite to where the goalie is moving
        if movement == "LEFT":
            self.target = "right"
        elif movement == "RIGHT":
            self.target = "left"
        else:
            self.target = random.choice(["left", "right"])  # goalie STATIC: pick a random side
        self.foot = self.target

        self.turned = False
        self.adjusts = 0
        self.lost = 0
        self.state = State.ALIGN
        self.state_start = time.monotonic()
        

    def _align(self):
        if not self.turned:
            angle = TURN_ANGLE if self.target == "left" else -TURN_ANGLE
            self.robot.move_to(0, 0, angle)
            self.turned = True
            self.state_start = time.monotonic()
            return
        
        if time.monotonic() - self.state_start < SETTLE_TIME:
            return

        detection, _ = self.ball_detector.get_actual_detection()

        if detection is None:
            self.lost += 1
            if self.lost >= LOST_LIMIT:
                self.state = State.SHOOT
            return
        
        self.lost = 0
        x, y, _ = detection
        dx = x - KICK_X[self.foot]

        if self.adjusts >= MAX_ADJUSTS or (abs(dx) <= TOL_X and y >= KICK_Y - TOL_Y):
                self.state = State.SHOOT
                return

        # 4. correct one axis per tick: sideways first, then forward
        if abs(dx) > TOL_X:
            # ball to the right of the target (dx > 0) -> move right (negative y)
            self.kick.adjust_side(-dx * M_PER_PX)
        else:
            self.kick.adjust_front()

        self.adjusts += 1
        self.state_start = time.monotonic()

    def _shoot(self):
        if self.foot == "left":
            self.kick.kick_left()
        else:
            self.kick.kick_right()
        self.state = State.DONE
