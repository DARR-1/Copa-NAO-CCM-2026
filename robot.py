import qi
from vision.camera import Camera
from vision.ball_detector import BallDetector
#from vision.tag_detector import TagDetector
import time
import math

IMU_YAW = "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value"
HEAD_MIDDLE = "Device/SubDeviceList/Head/Touch/Middle/Sensor/Value"
HEAD_REAR = "Device/SubDeviceList/Head/Touch/Rear/Sensor/Value"

BOARD_SIZE = 5

class Robot:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.app = qi.Application(["Robot", "--qi-url=tcp://{}:{}".format(ip, port)])
        self.app.start()

        self.motion_service = self.app.session.service("ALMotion")
        self.posture_service = self.app.session.service("ALRobotPosture")
        self.tts_service = self.app.session.service("ALTextToSpeech")
        self.memory_service = self.app.session.service("ALMemory")

        self.camera = Camera(self.app.session)
        self.ball_detector = BallDetector(self.app.session)
        # self.tag_detector = TagDetector(self.app.session)

        self.delta_yaw = 0.0

        self.move_config = [
            ["MaxStepX", 0.08], # we recommend 0.060 meters for StepX for more stability. Better use 0.080 meters only when walking on flat hard floors.
            ["MaxStepY", 0.16],
            ["MaxStepTheta", 0.524],

            ["MaxStepFrequency", 0.8],
            ["StepHeight", 0.04],

            ["TorsoWx", 0],
            ["TorsoWy", 0],
        ]

    def init(self):
        self.motion_service.wakeUp()
        self.posture_service.goToPosture("Stand", 0.5)

    def say(self, text):
        self.tts_service.say(text)
    
    def move(self, v_x, v_y, v_theta):
        self.motion_service.moveToward(v_x, v_y, v_theta, self.move_config)

    def move_to(self, x, y, theta):
        self.motion_service.moveTo(x, y, theta, self.move_config)

    def wait_for_start(self):
        while True:
            if self.memory_service.getData(HEAD_MIDDLE) == True:
                break

    def get_camera(self):
        return self.camera
    
    def get_yaw(self):
        return self.motion_service.getRobotPosition(True)[2] + self.delta_yaw  # Wz is an angle between [-pi, pi]

    def get_head_yaw_angle(self):
        return self.motion_service.getAngles(["HeadYaw"], True)[0]
    
    def get_rear_head_touch(self):
        return self.memory_service.getData(HEAD_REAR)

    def get_current_cell(self):
        x, y, _ = self.motion_service.getRobotPosition(True)
        cell_x = round(x / 0.244)
        cell_y = round(y / 0.244)

        cell_id = cell_x + cell_y * BOARD_SIZE

        return cell_id

    def set_yaw(self, yaw):
        current_yaw = self.get_yaw()
        self.delta_yaw = (yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi  # [-pi, pi]

    def look(self, pitch, yaw):
        self.motion_service.setAngles(["HeadPitch", "HeadYaw"], [pitch, yaw], 0.1)

    def rest(self):
        self.motion_service.rest()

    def stop_move(self):
        self.motion_service.stopMove()

    def stop(self):
        self.motion_service.killAll()