"""Track an other robomaster with the build in robomaster recognizer."""
from robomaster import robot, blaster
import cv2
import imutils
import keyboard

gimbal_speed = 170
gvx, gvy = 0, 0
img_w, img_h = 280, 280
bounding_boxes = []


def detect_robot(robot_info):
    """Clear the bounding_boxes list and safes new bounding boxes."""
    bounding_boxes.clear()
    # (x, y) is the middle of the bounding box, w is the width of the bounding box and h is the height of the bounding
    for x, y, w, h in robot_info:
        bounding_boxes.append((x, y, w, h))


# Create GUI
window_name = "Robot"
cv2.namedWindow(window_name, cv2.WINDOW_GUI_EXPANDED)
cv2.resizeWindow(window_name, img_w, img_h)

# Initialize robot in starting postion
srobot = robot.Robot()
srobot.initialize(conn_type="sta")
srobot.gimbal.recenter().wait_for_completed()

# Start camera and detection
srobot.camera.start_video_stream(display=False, resolution="360p")
srobot.vision.sub_detect_info(name="robot", callback=detect_robot)

while True:
    img = srobot.camera.read_cv2_image(strategy="newest")
    img = imutils.resize(img, width=img_w, height=img_h)

    # Draw bounding boxes.
    for x, y, w, h in bounding_boxes:
        cv2.rectangle(img, (int((x - w / 2) * img.shape[1]),
                            int((y - h / 2) * img.shape[0])),
                      (int((x + w / 2) * img.shape[1]),
                       int((y + h / 2) * img.shape[0])),
                      (255, 0, 0), 2)

    if len(bounding_boxes) > 0:
        # Aims at the first bounding box in the list
        x, y, w, h = bounding_boxes[0]
        dx = x - 0.5
        dy = y - 0.5
        print(dx, dy)
        gvx, gvy = dx * gimbal_speed, -dy * gimbal_speed
        # Set led to green when a robot is seen
        srobot.led.set_gimbal_led(r=0, g=255, b=0)
        # Fire at the robot
        srobot.blaster.fire(fire_type=blaster.INFRARED_FIRE)
    else:
        gvx *= 0.7
        gvy *= 0.7
        # Set led to red when no robot is seen
        srobot.led.set_gimbal_led(r=255, g=0, b=0)

    srobot.gimbal.drive_speed(yaw_speed=gvx, pitch_speed=gvy)

    cv2.imshow(window_name, img)
    cv2.waitKey(1)
    # Press [x] to stop the program
    if keyboard.is_pressed("x"):
        break

cv2.destroyAllWindows()

# Stops the gimbal and recenters it for future test
srobot.gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
srobot.gimbal.recenter()

# Stops info gathering and camera stream
srobot.vision.unsub_detect_info()
srobot.camera.stop_video_stream()

srobot.close()
