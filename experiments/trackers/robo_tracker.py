from robomaster import robot, blaster
import cv2
import imutils
import time

gimbal_speed = 1.5  # Speed to move the gimbal by
gvx, gvy = 0, 0  # Starting gimbal velocities
bounding_boxes = []


class RobotInfo:

    def __init__(self, x, y, w, h):
        self._x = x
        self._y = y
        self._w = w
        self._h = h

    @property
    def pt1(self):
        return int((self._x - self._w / 2) * img_w), int((self._y - self._h / 2) * img_h)

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * img_w), int((self._y + self._h / 2) * img_h)

    @property
    def center(self):
        return int(self._x * img_w), int(self._y * img_h)


def on_detect_person(person_info):
    number = len(person_info)
    bounding_boxes.clear()
    for i in range(0, number):
        x, y, w, h = person_info[i]
        bounding_boxes.append(RobotInfo(x, y, w, h))
        print("robot: x:{0}, y:{1}, w:{2}, h:{3}".format(x, y, w, h))


# Setup the window to draw to
window_name = 'Robot'
cv2.namedWindow(window_name, cv2.WINDOW_GUI_EXPANDED)
cv2.resizeWindow(window_name, 800, 800)

# Connect/setup the robot
srobot = robot.Robot()
srobot.initialize(conn_type="sta", sn="159CGAC0050QS0")  # SN is the Serial Number of the specific robot

srobot.gimbal.recenter().wait_for_completed()
# srobot.gimbal.move(pitch=-20, yaw=-10).wait_for_completed()

srobot.camera.start_video_stream(display=False, resolution='360p')
result = srobot.vision.sub_detect_info(name="robot", callback=on_detect_person)
time.sleep(3)
img_h = 250
img_w = 250
mid_map = (img_w//2, img_h//2)
while True:
    # Get the newest image from the camera and resize
    img = srobot.camera.read_cv2_image(strategy='newest')
    img = imutils.resize(img, width=img_w)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Draw the bounding_boxes
    for i in range(0, len(bounding_boxes)):
        cv2.rectangle(img, bounding_boxes[i].pt1, bounding_boxes[i].pt2, (255, 0, 0))
        test = []
    cv2.rectangle(img, (40, 40), (50, 50), (0, 255, 0))

    if len(bounding_boxes) > 0:

        # x speed
        if bounding_boxes[0].center[0] > mid_map[0]:
            gvx = 10
        elif bounding_boxes[0].center[0] < mid_map[0]:
            gvx = -10
        # y speed
        if bounding_boxes[0].center[1] > mid_map[1]:
            gvy = -10
        elif bounding_boxes[0].center[1] < mid_map[1]:
            gvy = 10
        srobot.led.set_gimbal_led(r=0, g=255, b=0)  # Set the gimbal lights to green
        srobot.blaster.fire(fire_type=blaster.WATER_FIRE, times=1)  # Fire the blaster

    # if len(bounding_boxes) > 0:
    #     dx = sum([x + w / 2 for x, _, w, _ in bounding_boxes]) / len(bounding_boxes) - img.shape[
    #         1] / 2  # Average horizontal offset to center
    #     dy = sum([y + h / 2 for _, y, _, h in bounding_boxes]) / len(bounding_boxes) - img.shape[0] / 2  # Average vertical offset to center
    #     gvx, gvy = dx * gimbal_speed, -dy * gimbal_speed  # Update gimbal velocities using the offsets to center

    #    srobot.led.set_gimbal_led(r=0, g=255, b=0)  # Set the gimbal lights to green
    #    srobot.blaster.fire(fire_type=blaster.WATER_FIRE, times=1)  # Fire the blaster
    else:
        # Apply friction to the gimbal
        gvx *= 0.7
        gvy *= 0.7

        srobot.led.set_gimbal_led(r=255, g=0, b=0)  # Set the gimbal lights to red

    # Update the physical gimbal velocity using the newly calculated
    srobot.gimbal.drive_speed(pitch_speed=gvy, yaw_speed=gvx)

    # Draw the image to the window
    cv2.imshow(window_name, img)
    cv2.waitKey(1)

    # Stop the loop when the [X] window button is pressed
    if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
        break

# Set everything back to normal
cv2.destroyAllWindows()
srobot.gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
srobot.gimbal.recenter()
srobot.camera.stop_video_stream()

srobot.close()
