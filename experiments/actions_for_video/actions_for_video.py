"""This is the script for the action the robot can perform in the video."""
import time
from robomaster import robot
from robomaster import led
import random as ran


def start_settings(srobot: robot.Robot):
    """Recenters, enables chassis lead and turn led off."""
    srobot.gimbal.recenter().wait_for_completed()
    srobot.set_robot_mode(robot.CHASSIS_LEAD)
    srobot.led.set_gimbal_led(comp=led.COMP_ALL, r=0, g=0, b=150, effect=led.EFFECT_ON)
    return srobot


def scene1(srobot: robot.Robot):
    """Drives to camera and rotates chassis 90 degrees in 1 movement."""
    srobot.chassis.move(x=0.95, y=0, z=90, xy_speed=1.2, z_speed=70).wait_for_completed()


def scene2(srobot: robot.Robot):
    """Lets turn on in leds in steps of 5."""
    srobot.led.set_gimbal_led(comp=led.COMP_TOP_ALL, r=0, g=0, b=150,
                              led_list=[0], effect=led.EFFECT_ON)
    time.sleep(1)
    srobot.led.set_gimbal_led(comp=led.COMP_TOP_ALL, r=0, g=0, b=150,
                              led_list=[0, 1], effect=led.EFFECT_ON)
    time.sleep(1)
    srobot.led.set_gimbal_led(comp=led.COMP_TOP_ALL, r=0, g=0, b=150,
                              led_list=[0, 1, 2], effect=led.EFFECT_ON)
    time.sleep(1)
    srobot.led.set_gimbal_led(comp=led.COMP_TOP_ALL, r=0, g=0, b=150,
                              led_list=[0, 1, 2, 3], effect=led.EFFECT_ON)
    time.sleep(1)
    srobot.led.set_gimbal_led(comp=led.COMP_TOP_ALL, r=0, g=0, b=150,
                              led_list=[0, 1, 2, 3, 4], effect=led.EFFECT_ON)


def scene3(srobot: robot.Robot, loop_amount: int):
    """Drives sideways and rotates the led while the led get a random color every time."""
    srobot.chassis.move(y=0.45, xy_speed=1.2).wait_for_completed()
    for i in range(loop_amount):
        for j in range(0, 8):
            led1 = j % 8
            led2 = (j + 1) % 8
            led3 = (j + 2) % 8
            srobot.led.set_gimbal_led(comp=led.COMP_TOP_ALL, r=ran.randint(0, 250), g=ran.randint(0, 250),
                                      b=ran.randint(0, 250),
                                      led_list=[led1, led2, led3], effect=led.EFFECT_ON)
            time.sleep(0.1)


def scene4(srobot: robot.Robot):
    """Turn lights red and turns gimbal to the camera."""
    srobot.led.set_gimbal_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)
    srobot.set_robot_mode(robot.FREE)
    time.sleep(1)
    srobot.gimbal.move(pitch=0, yaw=70, yaw_speed=100).wait_for_completed()


def scene5(srobot: robot.Robot):
    """Turn chassis back to the camera and knocks it over."""
    srobot.chassis.move(z=-80, z_speed=45).wait_for_completed()
    srobot.gimbal.recenter()
    srobot.chassis.move(x=0.45, xy_speed=2).wait_for_completed()


def scene6(srobot: robot.Robot, loop_count: int = 3, x_distance: float = 1, xy_speed: float = 1.3, z_speed: int = 100):
    """Drives in a line forward while rotating 360 degrees."""
    for _ in range(loop_count):
        srobot.chassis.move(x=x_distance, z=360, xy_speed=xy_speed, z_speed=z_speed)


if __name__ == '__main__':
    srobot = robot.Robot()
    srobot.initialize(conn_type="sta")
    start_settings(srobot)
    time.sleep(2)
    scene1(srobot)
    time.sleep(2)
    scene2(srobot)
    time.sleep(2)
    scene3(srobot, loop_amount=5)
    # scene 4 is directly after scene 3
    scene4(srobot)
    time.sleep(1)
    # Can if u want:
    # scene5(srobot)

    # Separate scene
    # scene6(srobot, loop_count=4)
    srobot.close()
