from robomaster import robot
from robomaster import blaster

if __name__ == '__main__':
    srobot = robot.Robot()
    srobot.initialize(conn_type="sta")

    # ep_chassis.drive_speed(x=x_val, y=0, z=0, timeout=5)
    srobot.chassis.move(x=0.5, y=0, z=0, xy_speed=1)
    srobot.gimbal.move(pitch=15, yaw=-45).wait_for_completed()
    srobot.chassis.move(x=-0.5, y=0, z=0, xy_speed=1).wait_for_completed()
    srobot.gimbal.recenter().wait_for_completed()
    for i in range(1):
        srobot.gimbal.move(pitch=15, yaw=45).wait_for_completed()
        srobot.blaster.fire(fire_type=blaster.INFRARED_FIRE, times=3)#.wait_for_completed()
        # srobot.gimbal.move(pitch=-10, yaw=-60)
        # srobot.blaster.fire(fire_type=blaster.INFRARED_FIRE, times=3)#.wait_for_completed()
        srobot.chassis.move(x=0.5, y=0, z=0, xy_speed=1).wait_for_completed()
        srobot.chassis.move(x=0, y=0, z=90, xy_speed=1).wait_for_completed()
        srobot.gimbal.recenter().wait_for_completed()

    srobot.close()
