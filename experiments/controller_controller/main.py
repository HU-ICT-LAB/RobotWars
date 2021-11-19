from .SensorLogger import SensorLogger

from robomaster import robot as robomaster_robot
from robomaster import blaster
import pygame
import datetime
import time
import keyboard
from PIL import Image
import base64
from pathlib import Path



# This code has been created and tested with a PS4 controller, but should in theory work with any controller recognized by your OS
pygame.init()
joystick = pygame.joystick.Joystick(0)
robot = robomaster_robot.Robot()
robot.initialize(conn_type="sta")
print("Connected")
robot.set_robot_mode(robomaster_robot.GIMBAL_LEAD)
robot.gimbal.recenter()

# Generate loggers
sensor_logger = SensorLogger("experience1")

# Subscribe to sensors of the RoboMaster
robot.chassis.sub_position(freq=20, callback=sensor_logger.log_chassis)
robot.gimbal.sub_angle(freq=20, callback=sensor_logger.log_gimbal)
robot.chassis.sub_imu(freq=20, callback=sensor_logger.log_imu)
robot.camera.start_video_stream(display=True)


speed = 0.5 # todo remove if you remove keyboard
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
        z = sensor_logger.relative_yaw*5
        pitch = -joystick.get_axis(4)*100 if abs(joystick.get_axis(4)) > 0.01 else 0
        yaw = joystick.get_axis(3)*300 if abs(joystick.get_axis(3)) > 0.01 else 0
        # Gimbal and Chassis control
        robot.gimbal.drive_speed(pitch, yaw)
        robot.chassis.drive_speed(x=x, y=y, z=z)
        # Append current speed measurement to the corresponding list
        sensor_logger.log_chassis_speed((x, y, z))
        sensor_logger.log_gimbal_speed((pitch, yaw))
        # Checks blaster fire
        if joystick.get_button(7):
            robot.blaster.fire(blaster.WATER_FIRE)
            sensor_logger.log_blaster_fire(True)
        elif joystick.get_button(6):
            robot.blaster.fire(blaster.INFRARED_FIRE)
            sensor_logger.log_blaster_fire(True)
        else:
            sensor_logger.log_blaster_fire(False)
        time.sleep(0.05)
        # Image save
        np_img = robot.camera.read_cv2_image(strategy='newest') # TODO convert img to blob png
        im = Image.fromarray(np_img)
        im_path = str(Path(r"video_frames") / f"{datetime.datetime.now().strftime('%m_%d_%Y_%H_%M_%S_%f3')}") + ".png"
        im.save(im_path)
        sensor_logger.create_row()
    except KeyboardInterrupt:
        done = True

robot.camera.stop_video_stream()
print('done constrolling')
robot.chassis.unsub_position()
robot.chassis.unsub_imu()
robot.gimbal.unsub_angle()
print('unsubscribed')
print('saving logs')
sensor_logger.save()
print('saved')
robot.close()
