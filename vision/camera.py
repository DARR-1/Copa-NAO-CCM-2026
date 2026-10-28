import numpy as np

class Camera:
    # índices de cámara: 0 = superior, 1 = inferior
    # resoluciones: 1 = QVGA (320x240), 2 = VGA (640x480), 3 = 4VGA (1280x960)
    # espacios de color: 0 = escala de grises (Y), 11 = RGB, 13 = BGR
    def __init__(self, session, camera=0, resolution=2, colorspace=0, fps=15):
        self.video = session.service("ALVideoDevice")
        self.name = "maze_cam"
        self.clean_old_subscriptions()
        self.handle = self.video.subscribeCamera(self.name, camera, resolution, colorspace, fps)

    def clean_old_subscriptions(self):
        for sub in self.video.getSubscribers():
            if sub.startswith(self.name):
                self.video.unsubscribe(sub)

    def get_frame(self):
        img = self.video.getImageRemote(self.handle)
        if img is None:
            return None
        w, h, layers = img[0], img[1], img[2]
        frame = np.frombuffer(img[6], dtype=np.uint8)
        return frame.reshape((h, w, layers)) if layers > 1 else frame.reshape((h, w))

    def set_camera(self, camera):
        self.video.setActiveCamera(self.handle, camera)

    def close(self):
        self.video.unsubscribe(self.handle)