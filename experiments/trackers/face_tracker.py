from robomaster import robot
import cv2
import imutils

gimbal_speed = 1.5  # Speed to move the gimbal by
gvx, gvy = 0, 0  # Starting gimbal velocities

# Load the Haar Cascade
cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# Setup the window to draw to
window_name = 'Robot'
cv2.namedWindow(window_name, cv2.WINDOW_GUI_EXPANDED)
cv2.resizeWindow(window_name, 800, 800)

# Connect/setup the robot
srobot = robot.Robot()
srobot.initialize(conn_type="sta", sn="159CGAC0050QS0")  # SN is the Serial Number of the specific robot

srobot.gimbal.recenter()
srobot.camera.start_video_stream(display=False, resolution='360p')

while True:
    # Get the newest image from the camera and resize
    img = srobot.camera.read_cv2_image(strategy='newest')
    img = imutils.resize(img, width=250)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect objects
    bounding_boxes = cascade.detectMultiScale(gray, 1.1, 4)
    # Draw the bounding_boxes
    for (x, y, w, h) in bounding_boxes:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if len(bounding_boxes) > 0:
        dx = sum([x + w / 2 for x, _, w, _ in bounding_boxes]) / len(bounding_boxes) - img.shape[
            1] / 2  # Average horizontal offset to center
        dy = sum([y + h / 2 for _, y, _, h in bounding_boxes]) / len(bounding_boxes) - img.shape[0] / 2  # Average vertical offset to center
        gvx, gvy = dx * gimbal_speed, -dy * gimbal_speed  # Update gimbal velocities using the offsets to center

        srobot.led.set_gimbal_led(r=0, g=255, b=0)  # Set the gimbal lights to green
        # srobot.blaster.fire(fire_type=blaster.WATER_FIRE, times=1)  # Fire the blaster
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
