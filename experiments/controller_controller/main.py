"""This progrom creates controls the robot manually to create experience for Reinforcement Learning."""
from SensorLogger import SensorLogger
from db_logger import DBLogger
from robomaster import robot as robomaster_robot
from robomaster import blaster
import pygame
import time
import keyboard
from PIL import Image



# This code has been created and tested with a PS4 controller, but should in theory work with any controller
# recognized by your OS.
pygame.init()
joystick = pygame.joystick.Joystick(0)
robot = robomaster_robot.Robot()
# sn = "159CGAC0050R76"
sn = "169CGAC0050QS0"
robot.initialize(conn_type="sta")#, sn=sn)
print("Connected")
robot.set_robot_mode(robomaster_robot.GIMBAL_LEAD)
robot.gimbal.recenter()

# Generate log
db_logger = DBLogger(sn=sn)
sensor_logger = SensorLogger("experience1")

# Subscribe to sensors of the RoboMaster
# robot.chassis.sub_position(freq=20, callback=sensor_logger.log_chassis)
# robot.gimbal.sub_angle(freq=20, callback=sensor_logger.log_gimbal)
# robot.chassis.sub_imu(freq=20, callback=sensor_logger.log_imu)
robot.chassis.sub_position(freq=20, callback=db_logger.handle_position)
robot.gimbal.sub_angle(freq=20, callback=db_logger.handle_angle)
robot.chassis.sub_imu(freq=20, callback=db_logger.handle_imu)
robot.camera.start_video_stream(display=True)


done = False
while not done:
    if keyboard.is_pressed("p"):
        break
    try:
        pygame.event.pump()
        # Gimbal and Chassis speed
        x = -joystick.get_axis(1) * 1.2 if abs(joystick.get_axis(1)) > 0.01 else 0    # Drift prevention measure
        y = joystick.get_axis(0) * 1.2 if abs(joystick.get_axis(0)) > 0.01 else 0
        z = sensor_logger.relative_yaw * 5
        pitch = -joystick.get_axis(3) * 75 if abs(joystick.get_axis(3)) > 0.01 else 0
        yaw = joystick.get_axis(2) * 225 if abs(joystick.get_axis(2)) > 0.01 else 0
        # Gimbal and Chassis control
        robot.gimbal.drive_speed(pitch, yaw)
        robot.chassis.drive_speed(x=x, y=y, z=z)
        # Append current speed measurement to the corresponding list
        # Checks blaster fire
        if joystick.get_button(3):
            robot.blaster.fire(blaster.WATER_FIRE)
            fire_check = True
        elif joystick.get_button(2):
            robot.blaster.fire(blaster.INFRARED_FIRE)
            fire_check = True
        else:
            fire_check = False
        time.sleep(0.05)
        # Image save
        np_img = robot.camera.read_cv2_image(strategy='newest')
        im = Image.fromarray(np_img)
        # im.save(im_path)
        db_logger.log_action_point(camera_frame=im, action_chassis_speed=(x, y, z),
                                   action_gimbal_speed=(pitch, yaw), action_blaster_fire=fire_check)
    except KeyboardInterrupt:
        done = True

robot.camera.stop_video_stream()
print('done constrolling')
robot.chassis.unsub_position()
robot.chassis.unsub_imu()
robot.gimbal.unsub_angle()
print('unsubscribed')
robot.close()
