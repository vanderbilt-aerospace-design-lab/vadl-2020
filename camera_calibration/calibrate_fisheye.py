# --------------------------------------------------------------------------------------------- #
#
# This is a script for calibrating a single fisheye camera. It requires that you first have taken a
# series of calibration images using a chessboard pattern. All of the parameters at the top
# should be set according to your particular usage before running the script. For background,
# see the following link:
#
# https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
#
# The code explained in this link was modified to create this script.
#
# Usage instructions:
# Make sure the calibration image directory and checkerboard size are defined correctly. 
# The script will calculate the camera parameters and save them to a file.
#
# --------------------------------------------------------------------------------------------- #

# Import required packages
import cv2
assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
import yaml
import glob

OPEN_PATH = "calibration_images_arducam_filtered" # Calibration image directory
SAVE_PATH = "calibration_parameters/arducam.yaml" # Path to save calibration parameters

# Checkerboard (rows, columns)
CHECKERBOARD = (7,9)

# termination criteria for finding sub-pixel corner positions
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW

objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

_img_shape = None
imgpoints = [] # 2d points in image plane.
objpoints = [] # 3d point in real world space

# Retrieve calibration images; enter name of directory
images = glob.glob(OPEN_PATH + "/*.jpg')

# Extract corner information from each of the images
for fname in images:

    # Load image
    img = cv2.imread(fname)

    # Check the image shape 
    if _img_shape == None:
        _img_shape = img.shape[:2]
    else:
        assert _img_shape == img.shape[:2], "All images must share the same size."

    # Convert to grayscale
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, 
                                             cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria) # Subpixel refinement
        imgpoints.append(corners)

N_OK = len(objpoints)
K = np.zeros((3, 3)) # Camera matrix
D = np.zeros((4, 1)) # Distortion coefficients
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)] # Rotational position of each corner
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)] # Translational position of each corner

# Perform fiseye calibration
rms, _, _, _, _ = \
    cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )

# Print results
print("Found " + str(N_OK) + " valid images for calibration")
print("DIM=" + str(_img_shape[::-1]))
print("K=np.array(" + str(K.tolist()) + ")")
print("D=np.array(" + str(D.tolist()) + ")")

# Save calibration camera matrix and distortion coefficients to file for later use
data = {"camera_matrix": np.asarray(K).tolist(),
        "dist_coefficients": np.asarray(D).tolist(),
        "image_points": np.asarray(imgpoints).tolist(),
        "object_points": np.asarray(objpoints).tolist()}

out = open(SAVE_PATH, 'w')
yaml.dump(data, out)
