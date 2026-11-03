import cv2
from vision.camera import Camera
import numpy as np

class BallDetector:
    def __init__(self, session, cam_id=1):
        self.camera = Camera(session, camera=cam_id, colorspace=13, name="ball_cam")

        # El rojo da la vuelta en HSV: M1 cubre el extremo alto (170-179) y M2 el bajo (0-5)
        self.lower_color_m1 = (170, 100, 90)
        self.upper_color_m1 = (179, 255, 255)

        self.lower_color_m2 = (0, 90, 90)
        self.upper_color_m2 = (5, 255, 255)

        self.fov_rad = np.deg2rad(56.3)  # FOV horizontal de las cámaras del NAO
        self.ball_diameter = 0.065
        self.frame_size = None  # (w, h) del último frame leído

        self.trackbars_ready = False

    def setup_trackbars(self):
        for name, low, up in (("M1 Calibration", self.lower_color_m1, self.upper_color_m1),
                              ("M2 Calibration", self.lower_color_m2, self.upper_color_m2)):
            
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, 450, 260)

            cv2.createTrackbar("H min", name, low[0], 179, lambda v: None)
            cv2.createTrackbar("H max", name, up[0], 179, lambda v: None)
            cv2.createTrackbar("S min", name, low[1], 255, lambda v: None)
            cv2.createTrackbar("S max", name, up[1], 255, lambda v: None)
            cv2.createTrackbar("V min", name, low[2], 255, lambda v: None)
            cv2.createTrackbar("V max", name, up[2], 255, lambda v: None)
            
        self.trackbars_ready = True

    def update_from_trackbars(self):
        def read(name):
            g = lambda t: cv2.getTrackbarPos(t, name)
            return (g("H min"), g("S min"), g("V min")), (g("H max"), g("S max"), g("V max"))

        self.lower_color_m1, self.upper_color_m1 = read("M1 Calibration")
        self.lower_color_m2, self.upper_color_m2 = read("M2 Calibration")

    def print_ranges(self):
        print(f"self.lower_color_m1 = {self.lower_color_m1}")
        print(f"self.upper_color_m1 = {self.upper_color_m1}")
        print(f"self.lower_color_m2 = {self.lower_color_m2}")
        print(f"self.upper_color_m2 = {self.upper_color_m2}")

    def detect_ball(self, frame):
        blur = cv2.GaussianBlur(frame, (9, 9), 0)
        hsv_frame = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        m1 = cv2.inRange(hsv_frame, self.lower_color_m1, self.upper_color_m1)
        m2 = cv2.inRange(hsv_frame, self.lower_color_m2, self.upper_color_m2)

        mask = cv2.bitwise_or(m1, m2)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, mask

        c = max(contours, key=cv2.contourArea)
        (x, y), r = cv2.minEnclosingCircle(c)

        if r < 5:
            return None, mask

        return (int(x), int(y), int(r)), mask

    def get_actual_detection(self, show_video=False):
        frame = self.camera.get_frame()

        if frame is None:
            return None, None

        self.frame_size = (frame.shape[1], frame.shape[0])

        if show_video:
            if not self.trackbars_ready:
                self.setup_trackbars()
            self.update_from_trackbars()

        detection, mask = self.detect_ball(frame)

        if show_video:
            shown = frame.copy()

            if detection:
                x, y, r = detection
                cv2.circle(shown, (x, y), r, (0, 255, 0), 2)
                cv2.circle(shown, (x, y), 5, (0, 0, 255), -1)

            cv2.imshow("Ball Detection", shown)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(1) & 0xFF == ord("p"):
                self.print_ranges()

        if detection is None:
            return None, mask

        x, y, r = detection
        x -= frame.shape[1] // 2
        y -= frame.shape[0] // 2

        return (x, y, r), mask

    # Recibe la detección (x, y, r) de get_actual_detection (ya centrada) y
    # regresa (X, Y, Z) en metros respecto a la cámara. Z es la distancia sobre
    # el eje óptico, no sobre el piso. Regresa None si la pelota toca el borde
    # del frame, porque el radio ya no es confiable.
    def get_ball_position(self, detection):
        if detection is None or self.frame_size is None:
            return None

        x, y, r = detection
        w, h = self.frame_size

        if abs(x) + r >= w / 2.0 or abs(y) + r >= h / 2.0:
            return None

        f = (w / 2.0) / np.tan(self.fov_rad / 2.0)

        d = 2.0 * r
        Z = f * self.ball_diameter / d
        X = x * Z / f
        Y = y * Z / f

        return X, Y, Z

    def close(self):
        self.camera.close()
        cv2.destroyAllWindows()
