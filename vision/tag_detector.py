import cv2
import numpy as np
from pupil_apriltags import Detector
from vision.camera import Camera

class TagDetector:
    def __init__(self, session):
        # QVGA: 4 veces menos datos por WiFi que VGA; quad_decimate se queda en 1.0 porque ya es imagen chica
        self.camera = Camera(session, camera=0, resolution=1, colorspace=0, name="tag_cam")
        self.camera_params = self.get_camera_params()
        self.at_detector = Detector(families="tag36h11",
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_sigma=0.8,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0
                                    )
        self.tag_size = 0.15
        self.min_margin = 20
        self.max_hamming = 0
        self.max_tag_id = 24

    def get_camera_params(self):
        frame = self.camera.get_frame()

        while frame is None:
            frame = self.camera.get_frame()

        width = frame.shape[1]
        height = frame.shape[0]

        hfov = self.camera.video.getHorizontalFOV(0)
        vfov = self.camera.video.getVerticalFOV(0)

        fx = (width / 2) / np.tan(hfov / 2)
        fy = (height / 2) / np.tan(vfov / 2)

        cx = width / 2
        cy = height / 2

        return fx, fy, cx, cy

        

    def is_valid(self, detection):
        return (0 <= detection.tag_id <= self.max_tag_id and detection.hamming <= self.max_hamming
                and detection.decision_margin >= self.min_margin)

    def find_tags(self, frame):
        # Regresa todas las detecciones crudas y el tag válido más cercano (o None)
        detections = self.at_detector.detect(frame,
                                             estimate_tag_pose=True,
                                             camera_params=self.camera_params,
                                             tag_size=self.tag_size)

        valid = [d for d in detections if self.is_valid(d)] #type: ignore
        closest = min(valid, key=lambda d: d.pose_t[2][0]) if valid else None

        return detections, closest

    def get_angles(self, detection):
        theta = np.arctan2(detection.pose_R[0][2], detection.pose_R[2][2])
        bearing = np.arctan2(detection.pose_t[0][0], detection.pose_t[2][0])
        return theta, bearing

    def detect_tag(self, frame):
        _, closest = self.find_tags(frame)

        if closest is None:
            return None, None, None

        theta, bearing = self.get_angles(closest)

        return closest.tag_id, theta, bearing

    def draw_detection(self, img, detection, color, thickness):
        corners = detection.corners.astype(int)
        cv2.polylines(img, [corners.reshape(-1, 1, 2)], True, color, thickness)
        cx, cy = detection.center.astype(int)
        cv2.circle(img, (cx, cy), 4, (0, 0, 255), -1)

        text = f"id={detection.tag_id} m={detection.decision_margin:.0f}"
        if detection.pose_t is not None:
            text += f" z={detection.pose_t[2][0]:.2f}m"

        cv2.putText(img, text, (corners[:, 0].min(), corners[:, 1].min() - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def show(self, frame, detections, closest):
        # Rojo = descartado por los filtros, verde = válido, amarillo = el más cercano (el que se usa)
        shown = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if frame.ndim == 2 else frame.copy()

        for d in detections: #type: ignore
            if d is closest:
                continue
            color = (0, 255, 0) if self.is_valid(d) else (0, 0, 255)
            self.draw_detection(shown, d, color, 2)

        if closest is not None:
            self.draw_detection(shown, closest, (0, 255, 255), 3)
            theta, bearing = self.get_angles(closest)
            status = f"tag {closest.tag_id}  theta={np.degrees(theta):.0f}  bearing={np.degrees(bearing):.0f} deg"
        else:
            status = "sin tag"

        cv2.putText(shown, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Tag Detection", shown)
        cv2.waitKey(1)

    def get_actual_detection(self, show_video=False):
        frame = self.camera.get_frame()

        if frame is None:
            return None, None, None

        detections, closest = self.find_tags(frame)

        if show_video:
            self.show(frame, detections, closest)

        if closest is None:
            return None, None, None

        theta, bearing = self.get_angles(closest)

        return closest.tag_id, theta, bearing

    def close_tag_detector(self):
        self.camera.close()
        cv2.destroyAllWindows()



