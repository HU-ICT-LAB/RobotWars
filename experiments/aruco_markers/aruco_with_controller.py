from robomaster import robot as robomaster_robot
from robomaster import blaster
import pygame
import yaml
import numpy as np
from cv2 import cv2
from cv2.cv2 import aruco

from draw_map import create_map

relative_yaw = 0


def handle_gimbal_angle(gimbal_angle):
    global relative_yaw
    pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = gimbal_angle
    relative_yaw = yaw_angle


room_name = "test_corner.yaml"    # TODO
with open(room_name, "r") as file:
    room_data = yaml.load(file, Loader=yaml.FullLoader)["aruco_codes"]

# This code has been created and tested with a PS4 controller, but should in theory work with any controller recognized by your OS
pygame.init()
screen = create_map(room_name)  # TODO
joystick = pygame.joystick.Joystick(0)
robot = robomaster_robot.Robot()
robot.initialize(conn_type="sta")
print("Connected")
robot.set_robot_mode(robomaster_robot.GIMBAL_LEAD)
robot.gimbal.recenter()
robot.gimbal.sub_angle(freq=20, callback=handle_gimbal_angle)
robot.camera.start_video_stream(display=False, resolution='360p')

window_name = "aruco demo"
cv2.namedWindow(window_name)
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
aruco_params = aruco.DetectorParameters_create()
camera_matrix = np.array([
    [307.31529983, 0., 317.97061071],
    [0., 306.69658253, 174.31811448],
    [0., 0., 1.]
])
camera_dist = np.array([[-0.05377188, -0.00942119, 0.00065274, 0.00023877, -0.00474089]])
marker_size = 14.1  # cm

while cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
    frame = robot.camera.read_cv2_image(strategy='newest')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params, cameraMatrix=camera_matrix,
                                                 distCoeff=camera_dist)
    if ids is not None:
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_dist)
        robot_x_coords = []
        robot_y_coords = []
        for i, (marker_id, marker_corners) in enumerate(zip(ids, corners)):
            rvec, tvec = ret[0][i, 0, :], ret[1][i, 0, :]
            coords = [i / 10 for i in room_data.get(int(marker_id))["coordinates"][0:2]]
            robot_x_coords.append(coords[1]-tvec[0])
            robot_y_coords.append(tvec[2]+coords[0]) # TODO maybe not so clean with this order
            # aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_dist, rvec, tvec, 10)

            corners = marker_corners.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            cv2.putText(frame, str(marker_id[0]), (int(bottomRight[0]), int(bottomRight[1])), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255))
            cv2.putText(frame, f"{(tvec*10).round()}", (int(topRight[0]), int(topRight[1])), cv2.FONT_HERSHEY_SIMPLEX, .6,
                        (255, 255, 255))
        calc_coords = [sum(robot_x_coords)/len(robot_x_coords), sum(robot_y_coords)/len(robot_y_coords)]
        pygame.draw.circle(screen, [0, 0, 0], calc_coords, 2, 2) # TODO
        pygame.display.update()
    cv2.imshow(window_name, frame)
    k = cv2.waitKey(10)

    pygame.event.pump()
    robot.gimbal.drive_speed(-joystick.get_axis(3)*50, joystick.get_axis(2)*150)
    robot.chassis.drive_speed(x=-joystick.get_axis(1)*.5, y=joystick.get_axis(0)*.5, z=relative_yaw*5)
    if joystick.get_button(0):
        robot.blaster.fire(blaster.WATER_FIRE)
    elif joystick.get_button(1):
        robot.blaster.fire(blaster.INFRARED_FIRE)

cv2.destroyAllWindows()
robot.camera.stop_video_stream()
robot.close()
