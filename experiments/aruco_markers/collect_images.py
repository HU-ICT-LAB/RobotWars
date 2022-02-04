"""Collect images for calibration."""
import cv2
from robomaster import robot

srobot = robot.Robot()
srobot.initialize(conn_type="sta")  # SN is the Serial Number of the specific robot
srobot.gimbal.recenter()
srobot.camera.start_video_stream(display=False, resolution='360p')
window_name = "Collect Images"
cv2.namedWindow(window_name)
img_counter = 0
CHECKERBOARD = (7, 7)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

while cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
    frame = srobot.camera.read_cv2_image(strategy='newest')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH
                                             + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        visual = cv2.drawChessboardCorners(frame.copy(), CHECKERBOARD, corners2, ret)
    else:
        visual = frame
    cv2.imshow(window_name, visual)

    k = cv2.waitKey(1)
    if k % 256 == 32 and ret:
        # SPACE pressed

        img_name = f"./images/opencv_frame_{img_counter}.png"
        cv2.imwrite(img_name, frame)
        print(f"{img_name} written!")
        img_counter += 1

cv2.destroyAllWindows()
srobot.camera.stop_video_stream()
srobot.close()
