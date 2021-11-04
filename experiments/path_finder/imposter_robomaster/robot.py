import time
import threading
import numpy as np

CHASSIS_LEAD = 'chassis_lead'
GIMBAL_LEAD = 'gimbal_lead'


class Robot:
    def __init__(self):
        self.robot_mode = GIMBAL_LEAD
        self.chassis = Chassis(self)
        self.gimbal = Gimbal(self)

    def initialize(self, conn_type, sn=None):
        pass

    def set_robot_mode(self, mode='gimbal_lead'):
        self.robot_mode = mode

    def close(self):
        pass


class Chassis:
    def __init__(self, robot):
        self.robot = robot
        self.position = np.array([0., 0., 0.])
        self._drive_speed = np.array([0., 0., 0.])
        self.last_update = time.time()
        self.position_subscription = None
        self.pos_sub_terminate = False

    def _update(self):
        now = time.time()
        self.position += self._drive_speed * (now - self.last_update)
        self.last_update = now

    def sub_position(self, freq, callback):
        self.pos_sub_terminate = False

        def pos_sub():
            while not self.pos_sub_terminate:
                self._update()
                callback(tuple(self.position))
                time.sleep(1/freq)
        self.position_subscription = threading.Thread(target=pos_sub)
        self.position_subscription.start()

    def unsub_position(self):
        self.pos_sub_terminate = True
        self.position_subscription.join()

    def sub_imu(self, freq, callback):
        pass

    def unsub_imu(self):
        pass

    def drive_speed(self, x, y, z):
        self._update()
        self._drive_speed = np.array([x, y, z], dtype=float)


class Gimbal:
    def __init__(self, robot):
        self.robot = robot

    def recenter(self):
        pass


class Camera:
    def start_video_stream(self, display):
        pass

    def read_cv2_image(self, strategy):
        return np.random.rand(100, 100)
