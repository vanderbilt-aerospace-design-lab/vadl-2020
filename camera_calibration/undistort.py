import cv2
import sys
import numpy as np
from utils import file_utils

DIR = "camera_calibration/undistorted_images"
FILE_BASE = "undistorted_image"
FILE_BASE_COMPARISON = "undistorted_image_comparison"
EXT = "jpg"
FILE_NAME = file_utils.create_file_name_chronological(DIR, FILE_BASE, EXT)
FILE_NAME_COMPARISON = file_utils.create_file_name_chronological(DIR, FILE_BASE_COMPARISON, EXT)

K, D = file_utils.load_yaml("camera_calibration/calibration_parameters/arducam.yaml")
DIM = (640, 480)

def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Save image
    cv2.imwrite(FILE_NAME, undistorted_img)

    # Save image comparison
    concat_img = np.concatenate((img, undistorted_img), axis=1)
    cv2.imwrite(FILE_NAME_COMPARISON, concat_img)

if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)
