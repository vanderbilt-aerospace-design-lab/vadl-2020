import cv2
from marker_detection.camera import VideoStreamer
import argparse

IMAGE_DIR = "/media/usb/calibration_images_arducam"

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Capture images on a webcam or Pi.')
parser.add_argument('-d','--display', default=0,
                    help="Display camera stream. 1 = Yes, 0 = No")
args = vars(parser.parse_args())

def main():
    ct = 0
    vs = VideoStreamer(use_pi=1)
    while True:
        frame = vs.read()

        if args["display"]:
            cv2.imshow("Image", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        elif cv2.waitKey(1) & 0xFF == ord('s'):
            cv2.imwrite(IMAGE_DIR + '/calib_image_{}.jpg'.format(ct), frame)
            print("Saved image {}".format(ct))
            ct += 1

    # When everything done, release the capture
    vs.stop()

if __name__=="__main__":
    main()
