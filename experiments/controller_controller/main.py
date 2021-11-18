from robomaster import robot as robomaster_robot
from robomaster import blaster
import pygame
import datetime
import time
import keyboard
from PIL import Image
import base64
from pathlib import Path
relative_yaw = 0
x_speed = []
y_speed = []
z_speed = []
pitch_speed = []
yaw_speed = []
fire_blaster = []

class SensorLogger:
    """This class records the data of the robots subscribe functions and save it to a csv."""

    def __init__(self, sensor_title: str, extra_sensor_data=None):
        self.title = sensor_title
        self.filename = f"{sensor_title}.csv"
        self.sensor_log = []
        self.extra_sensor_log = []
        self.extern_sensor = extra_sensor_data

    def log(self, sensor_input: tuple):
        """Add the incoming data to the log together with the datetime."""
        self.sensor_log.append([datetime.datetime.now(), *sensor_input])

    def log_gimbal(self, sensor_input: tuple):
        """Add the incoming data to the gimbal log together with the datetime."""
        global relative_yaw
        relative_yaw = sensor_input[1]
        self.sensor_log.append([datetime.datetime.now(), sensor_input[0], sensor_input[1]])

    def combine_pos_data(self):
        """
        This function is to replace the z position that is not working with the calculated z through
        the gimbal.
        """
        if self.extern_sensor is not None:
            self.extra_sensor_log = [self.extern_sensor.sensor_log[i][3] - self.extern_sensor.sensor_log[i][1]
                                     for i in range(len(self.extern_sensor.sensor_log))]
        if len(self.sensor_log) > len(self.extra_sensor_log):
            l = len(self.extra_sensor_log)
        else:
            l = len(self.sensor_log)
        self.sensor_log = [self.sensor_log[i][:2] + self.extra_sensor_log[i] for i in range(l)] # todo mis niet zo effiecent

    def save(self):
        """Save the log to a csv file."""
        if self.extra_sensor_log:   # Checks if there is the extra data for the position.
            self.combine_pos_data()
        with open(self.filename, 'a') as file:
            for row in self.sensor_log:
                file.write(f"{';'.join(map(str, row))}\n")


# This code has been created and tested with a PS4 controller, but should in theory work with any controller recognized by your OS
pygame.init()
joystick = pygame.joystick.Joystick(0)
robot = robomaster_robot.Robot()
robot.initialize(conn_type="sta")
print("Connected")
robot.set_robot_mode(robomaster_robot.GIMBAL_LEAD)
robot.gimbal.recenter()

# Generate loggers
gimbal_logger = SensorLogger("gimbal_logger")
position_logger = SensorLogger("position_logger", gimbal_logger)
imu_logger = SensorLogger("imu_logger")
chassis_speed_logger = SensorLogger("chassis_speed_logger")
gimbal_speed_logger = SensorLogger("gimbal_speed_logger")
blaster_fire_logger = SensorLogger("blaster_fire_logger")

# Subscribe to sensors of the RoboMaster
robot.gimbal.sub_angle(freq=20, callback=gimbal_logger.log_gimbal)
robot.chassis.sub_position(freq=20, callback=position_logger.log)
robot.chassis.sub_imu(freq=20, callback=imu_logger.log)
robot.camera.start_video_stream(display=True)


speed = 0.5 # todo
done = False
while not done:
    # velocity = np.array([0., 0., 0.])
    # if keyboard.is_pressed('w'):
    #     velocity += np.array([1, 0, 0]) * speed
    #     print('w')
    # if keyboard.is_pressed('s'):
    #     velocity += np.array([-1, 0, 0]) * speed
    #     print('s')
    # if keyboard.is_pressed('a'):
    #     velocity += np.array([0, -1, 0]) * speed
    #     print('a')
    # if keyboard.is_pressed('d'):
    #     velocity += np.array([0, 1, 0]) * speed
    #     print('d')
    # if keyboard.is_pressed('q'):
    #     velocity += np.array([0, 0, -70]) * speed
    # if keyboard.is_pressed('e'):
    #     velocity += np.array([0, 0, 70]) * speed
    # if keyboard.is_pressed('p'):
    #     break
    #
    # x, y, z = velocity
    # robot.chassis.drive_speed(x, y, z)

    # Read img frame and saves img frame to video_frames folder
    # np_img = robot.camera.read_cv2_image(strategy='newest')
    # im = Image.fromarray(np_img)
    # im_path = str(Path(r"video_frames") / f"{datetime.datetime.now().strftime('%m_%d_%Y_%H_%M_%S_%f3')}") +".png"
    # im = base64.b64encode(im)   # todo Check if this encodes the images correctly.
    # im.save(im_path)
    if keyboard.is_pressed("p"):
        break
    try:
        pygame.event.pump()
        # Gimbal and Chassis speed
        x = -joystick.get_axis(1)*2 if abs(joystick.get_axis(1)) > 0.01 else 0    # Drift prevention measure
        y = joystick.get_axis(0)*2 if abs(joystick.get_axis(0)) > 0.01 else 0
        z = relative_yaw*5
        pitch = -joystick.get_axis(4)*100 if abs(joystick.get_axis(4)) > 0.01 else 0
        yaw = joystick.get_axis(3)*300 if abs(joystick.get_axis(3)) > 0.01 else 0
        # Gimbal and Chassis control
        robot.gimbal.drive_speed(pitch, yaw)
        robot.chassis.drive_speed(x=x, y=y, z=z)
        # Append current speed measurement to the corresponding list
        chassis_speed_logger.log((x, y, z))
        gimbal_speed_logger.log((pitch, yaw))
        # Checks blaster fire
        if joystick.get_button(7):
            robot.blaster.fire(blaster.WATER_FIRE)
            blaster_fire_logger.log((True,))
        elif joystick.get_button(6):
            robot.blaster.fire(blaster.INFRARED_FIRE)
            blaster_fire_logger.log((True,))
        else:
            blaster_fire_logger.log((False,))
        time.sleep(0.05)
        # Image save
        np_img = robot.camera.read_cv2_image(strategy='newest')
        im = Image.fromarray(np_img)
        im_path = str(Path(r"video_frames") / f"{datetime.datetime.now().strftime('%m_%d_%Y_%H_%M_%S_%f3')}") + ".png"
        im.save(im_path)
    except KeyboardInterrupt:
        done = True

robot.camera.stop_video_stream()
print('done constrolling')
robot.chassis.unsub_position()
robot.chassis.unsub_imu()
robot.gimbal.unsub_angle()
print('unsubscribed')
print('saving logs')
gimbal_logger.save()
position_logger.save()
imu_logger.save()
chassis_speed_logger.save()
gimbal_speed_logger.save()
blaster_fire_logger.save()
print('saved')
robot.close()
