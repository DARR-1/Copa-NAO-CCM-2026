import time

class PID:
    def __init__(self, kp, ki, kd, out_limit, i_limit=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit = out_limit
        self.i_limit = i_limit
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        self.integral += error * dt
        # anti-windup
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))

        derivative = 0.0
        if self.prev_error is not None and dt > 0:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.out_limit, min(self.out_limit, out))