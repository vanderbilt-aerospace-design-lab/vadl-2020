import cv2
import sys
import numpy as np
from utils import file_utils

DIR = "camera_calibration/undistorted_images"
FILE_BASE = "undistorted_image_bal_0"
FILE_BASE_COMPARISON = "undistorted_image_comparison_bal_0"
EXT = "jpg"
FILE_NAME = DIR + "/" + FILE_BASE + ".jpg"
FILE_NAME_COMPARISON = DIR + "/" + FILE_BASE + ".jpg"

K, D = file_utils.load_yaml("camera_calibration/calibration_parameters/arducam.yaml")
DIM = (640, 480)

def undistort(img_path, balance=0, dim2=None, dim3=None):
    img = cv2.imread(img_path)
    # dim1 = img.shape[:2]
    #
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort

    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1

    scaled_K = K * dim1[0] / DIM[0] # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0

    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image.
    # OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)

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