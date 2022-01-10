import cv2

cam = cv2.VideoCapture(0)
window_name = "Collect Images"
cv2.namedWindow(window_name)
img_counter = 8

while cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
    ret, frame = cam.read()
    if not ret:
        print("failed to grab frame")
        break
    cv2.imshow(window_name, frame)

    k = cv2.waitKey(1)
    if k % 256 == 32:
        # SPACE pressed

        img_name = f"./images/opencv_frame_{img_counter}.png"
        cv2.imwrite(img_name, frame)
        print(f"{img_name} written!")
        img_counter += 1

cam.release()

cv2.destroyAllWindows()
