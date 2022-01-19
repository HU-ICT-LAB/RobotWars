"""Demo file for aruco localisation."""
import numpy as np
from cv2 import cv2
from cv2.cv2 import aruco
from robomaster import robot

window_name = "aruco demo"
cv2.namedWindow(window_name)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
aruco_params = cv2.aruco.DetectorParameters_create()
camera_matrix = np.array([
    [307.31529983, 0., 317.97061071],
    [0., 306.69658253, 174.31811448],
    [0., 0., 1.]
])
camera_dist = np.array([[-0.05377188, -0.00942119, 0.00065274, 0.00023877, -0.00474089]])
marker_size = 14.1  # cm

srobot = robot.Robot()
srobot.initialize(conn_type="sta")
srobot.gimbal.recenter()
srobot.camera.start_video_stream(display=False, resolution='360p')

while cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
    frame = srobot.camera.read_cv2_image(strategy='newest')

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params,
                                                 cameraMatrix=camera_matrix, distCoeff=camera_dist)
    if ids is not None:
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_dist)
        for i, (marker_id, marker_corners) in enumerate(zip(ids, corners)):
            rvec, tvec = ret[0][i, 0, :], ret[1][i, 0, :]
            # aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_dist, rvec, tvec, 10)

            corners = marker_corners.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            cv2.putText(frame, str(marker_id[0]), (int(bottomRight[0]), int(bottomRight[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255))
            cv2.putText(frame, f"{(tvec * 10).round()}", (int(topRight[0]), int(topRight[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255))

    cv2.imshow(window_name, frame)
    k = cv2.waitKey(1)

cv2.destroyAllWindows()
srobot.camera.stop_video_stream()
srobot.close()
