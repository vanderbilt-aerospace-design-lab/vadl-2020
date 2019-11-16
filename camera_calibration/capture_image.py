import numpy as np
import cv2

IMAGE_DIR = "./calibration_images_fisheye"

# Read images from camera
cap = cv2.VideoCapture(0)
ct = 0
while(True):
    # Capture frame-by-frame
    ret, img = cap.read()

    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    elif cv2.waitKey(1) & 0xFF == ord('s'):
        cv2.imwrite(IMAGE_DIR + '/calib_image_{}.png'.format(ct), img)
        print("Saved image {}".format(ct))
        ct += 1