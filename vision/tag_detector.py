import cv2
import numpy as np
from pupil_apriltags import Detector
from vision.camera import Camera

class TagDetector:
    def __init__(self, session):
        self.camera = Camera(session, colorspace=0)
        self.camera_params = self.get_camera_params()
        self.at_detector = Detector(families="tag36h11",
                                    nthreads=1,
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

        

    def detect_tag(self, frame):
        detections = self.at_detector.detect(frame,
                                             estimate_tag_pose=True,
                                             camera_params=self.camera_params,
                                             tag_size=self.tag_size)
        distances = {}

        for i in detections: #type: ignore
            if 0 <= i.tag_id <= self.max_tag_id and i.hamming <= self.max_hamming and i.decision_margin >= self.min_margin:
                distances[i] = i.pose_t[2][0]

        if len(distances) == 0: #type: ignore
            return None, None, None

        closest_tag = min(distances, key=lambda d: distances[d])

        theta = np.arctan2(closest_tag.pose_R[0][2], closest_tag.pose_R[2][2])
        bearing = np.arctan2(closest_tag.pose_t[0][0], closest_tag.pose_t[2][0])

        return closest_tag.tag_id, theta, bearing

    def get_actual_detection(self, show_video=False):
        frame = self.camera.get_frame()

        if frame is None:
            return None, None, None

        tag_id, theta, bearing = self.detect_tag(frame)

        return tag_id, theta, bearing

    def close_tag_detector(self):
        self.camera.close()
        cv2.destroyAllWindows()



