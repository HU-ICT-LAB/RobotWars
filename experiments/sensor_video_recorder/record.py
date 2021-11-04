import datetime
from PIL import Image

from robomaster import robot
import keyboard
import numpy as np


class SensorLogger:
    def __init__(self, sensor_title: str):
        self.filename = f"{sensor_title}.csv"
        self.sensor_log = []

    def log(self, sensor_input: tuple):
        self.sensor_log.append((datetime.datetime.now(), *sensor_input))

    def save(self):
        with open(self.filename, 'a') as file:
            for row in self.sensor_log:
                file.write(f"{';'.join(map(str, row))}\n")


srobot = robot.Robot()
srobot.initialize(conn_type="sta", sn="159CGAC0050QS0")
print("connected")
speed = 0.5
srobot.set_robot_mode(robot.CHASSIS_LEAD)
srobot.gimbal.recenter()
srobot.camera.start_video_stream(display=False)

position_logger = SensorLogger('position')
srobot.chassis.sub_position(freq=20, callback=position_logger.log)
imu_logger = SensorLogger('imu')
srobot.chassis.sub_imu(freq=20, callback=imu_logger.log)

while True:
    velocity = np.array([0., 0., 0.])
    if keyboard.is_pressed('w'):
        velocity += np.array([1, 0, 0]) * speed
        print('w')
    if keyboard.is_pressed('s'):
        velocity += np.array([-1, 0, 0]) * speed
        print('s')
    if keyboard.is_pressed('a'):
        velocity += np.array([0, -1, 0]) * speed
        print('a')
    if keyboard.is_pressed('d'):
        velocity += np.array([0, 1, 0]) * speed
        print('d')
    if keyboard.is_pressed('q'):
        velocity += np.array([0, 0, -70]) * speed
    if keyboard.is_pressed('e'):
        velocity += np.array([0, 0, 70]) * speed
    if keyboard.is_pressed('p'):
        break

    x, y, z = velocity
    srobot.chassis.drive_speed(x, y, z)

    np_img = srobot.camera.read_cv2_image(strategy='newest')
    im = Image.fromarray(np_img)
    im.save(f"video_frames/{datetime.datetime.now()}.png")

print('done constrolling')
srobot.chassis.unsub_position()
print('unsubscribed')
position_logger.save()
imu_logger.save()
print('saved')
srobot.close()
