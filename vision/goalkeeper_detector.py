import cv2
import time
import numpy as np

class GoalkeeperDetector:
    def __init__(self, camera, fps=15):
        self.camera = camera
        self.fps = fps

        self.previous_frame = None

        self.min_flow = 0.4
        self.direction_threshold = 0.2
        self.min_confidence = 0.6

    def prepare_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        gray = cv2.GaussianBlur(gray, (7, 7), 0)

        return gray

    def detect_movement(self, frame):
        current_frame = self.prepare_frame(frame)

        if self.previous_frame is None:
            self.previous_frame = current_frame
            return None, 0.0

        flow = cv2.calcOpticalFlowFarneback(
            self.previous_frame,
            current_frame,
            None,
            0.5,
            3,
            15,
            3,
            5,
            1.1,
            0
        )

        self.previous_frame = current_frame

        flow_x = flow[..., 0]
        flow_y = flow[..., 1]

        magnitude = np.sqrt(
            flow_x ** 2 + flow_y ** 2
        )

        moving_pixels = magnitude > self.min_flow

        if not np.any(moving_pixels):
            return "STATIC", 0.0

        horizontal_motion = flow_x[moving_pixels]

        mean_dx = np.mean(horizontal_motion)

        if abs(mean_dx) < self.direction_threshold:
            return "STATIC", 0.0

        if mean_dx < 0:
            direction = "LEFT"
            confidence = np.mean(
                horizontal_motion < -self.direction_threshold
            )

        else:
            direction = "RIGHT"
            confidence = np.mean(
                horizontal_motion > self.direction_threshold
            )

        if confidence < self.min_confidence:
            return "STATIC", confidence

        return direction, confidence

    def get_actual_detection(self, show_video=False):
        frame = self.camera.get_frame()

        if frame is None:
            return None, None

        direction, confidence = self.detect_movement(frame)

        if show_video:
            shown = frame.copy()

            text = direction if direction else "INITIALIZING"

            cv2.putText(
                shown,
                f"{text} {confidence:.2f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            cv2.imshow(
                "Goalkeeper Detection",
                shown
            )

            cv2.waitKey(1)

        return direction, confidence

    def get_movement(self, samples=10, show_video=False):
        self.previous_frame = None

        votes = []

        self.camera.set_camera(0)

        try:
            for _ in range(3):
                self.camera.get_frame()
                time.sleep(1.0 / self.fps)

            for _ in range(samples + 1):
                direction, _ = self.get_actual_detection(
                    show_video=show_video
                )

                if direction in ("LEFT", "RIGHT"):
                    votes.append(direction)

                time.sleep(1.0 / self.fps)

        finally:
            self.camera.set_camera(1)
            self.previous_frame = None

        if not votes:
            return "STATIC"

        left_votes = votes.count("LEFT")
        right_votes = votes.count("RIGHT")

        if left_votes > right_votes:
            return "LEFT"

        if right_votes > left_votes:
            return "RIGHT"

        return "STATIC"

    def close(self):
        cv2.destroyAllWindows()


#Falta implementar en striker