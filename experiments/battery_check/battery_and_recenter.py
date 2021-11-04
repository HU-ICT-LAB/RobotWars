import time
from robomaster import robot


def sub_info_handler(batter_info):
    """Print the battery percentage"""
    print("Battery: {0}%.".format(batter_info))


if __name__ == '__main__':
    srobot = robot.Robot()
    srobot.initialize(conn_type="sta")

    srobot.battery.sub_battery_info(1, sub_info_handler)
    time.sleep(2)
    srobot.battery.unsub_battery_info()

    srobot.gimbal.recenter().wait_for_completed()

    srobot.close()
