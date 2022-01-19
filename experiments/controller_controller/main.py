from robomaster import robot as robomaster_robot
from robomaster import blaster
import pygame
import time

relative_yaw = 0


def handle_gimbal_angle(gimbal_angle):
    global relative_yaw
    pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = gimbal_angle
    relative_yaw = yaw_angle


# This code has been created and tested with a PS4 controller, but should in theory work with any controller recognized by your OS
pygame.init()
joystick = pygame.joystick.Joystick(0)
robot = robomaster_robot.Robot()
robot.initialize(conn_type="sta")
print("Connected")
robot.set_robot_mode(robomaster_robot.GIMBAL_LEAD)
robot.gimbal.recenter()
robot.gimbal.sub_angle(freq=20, callback=handle_gimbal_angle)
robot.camera.start_video_stream(display=True)

done = False
while not done:
    try:
        pygame.event.pump()
        robot.gimbal.drive_speed(-joystick.get_axis(3)*100, joystick.get_axis(2)*300)
        robot.chassis.drive_speed(x=-joystick.get_axis(1)*2, y=joystick.get_axis(0)*2, z=relative_yaw*5)
        if joystick.get_button(7):
            robot.blaster.fire(blaster.WATER_FIRE)
        elif joystick.get_button(6):
            robot.blaster.fire(blaster.INFRARED_FIRE)
        time.sleep(0.05)
    except KeyboardInterrupt:
        done = True

robot.close()
