import numpy as np
from cv2 import cv2
from cv2.cv2 import aruco

window_name = "aruco demo"
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_EXPOSURE, 1)
cv2.namedWindow(window_name)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
aruco_params = cv2.aruco.DetectorParameters_create()
camera_matrix = np.array([
    [603.56119201, 0., 331.95528498],
    [0., 602.60960566, 238.35983292],
    [0., 0., 1.]
])
camera_dist = np.array([[0.1139639, 0.05564501, 0.00678356, 0.00489607, -0.93592243]])
marker_size = 10  # cm

while cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
    ret, frame = cam.read()
    if not ret:
        print("failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params, cameraMatrix=camera_matrix, distCoeff=camera_dist)
    if ids is not None:
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_dist)
        rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
        print(tvec)
        aruco.drawDetectedMarkers(frame, corners)
        aruco.drawAxis(frame, camera_matrix, camera_dist, rvec, tvec, 10)

    cv2.imshow(window_name, frame)
    k = cv2.waitKey(1)

cam.release()
cv2.destroyAllWindows()
