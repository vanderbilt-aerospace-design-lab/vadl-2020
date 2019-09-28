# --------------------------------------------------------------------------------------------- #
#
# This is a script for calibrating a single camera. It requires that you first have taken a
# series of calibration images using a chessboard pattern. All of the parameters at the top
# should be set according to your particular usage before running the script. For background,
# see the following links:
#
# https://docs.opencv.org/3.2.0/dc/dbb/tutorial_py_calibration.html
# https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
#
# The code explained in these links was modified to create this script.
#
# Usage instructions:
# Press 'y' for each frame if the marked corners seem accurate. Otherwise, press any other key
# and that frame won't be used for calibration. Be sure to specify the correct paths for loading
# the images and saving the results.
#
# --------------------------------------------------------------------------------------------- #

# Import required packages
import cv2
import os
import yaml
import numpy as np

# PARAMETERS FOR OUR SETUP - MAKE SURE THESE ARE CORRECT
CB_COLS = 9
CB_ROWS = 7
CB_SQUARE_SIZE = 19
IMG_SIZE = (480, 640)
OPEN_PATH = "./calibration_images/camera_b/"
SAVE_PATH = "./calibration_data/camera_b.yaml"

# termination criteria for finding sub-pixel corner positions
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# load path names of images to be used for calibration
fnames = os.listdir(OPEN_PATH)

# list to store 'image points' - these are the coordinates of the corners that
# we find in the image using findChessBoardCorners.
image_points = []

# list to store 'object points' - these are the 'true' XYZ coordinates of the
# chessboard corners. Units are whatever we make them to be. The object points
# will be the same for all images. We compute them once, then use for every image
op_single_img = np.zeros((CB_ROWS * CB_COLS, 3), np.float32)
op_single_img[:, 0:2] = np.mgrid[0:CB_ROWS, 0:CB_COLS].T.reshape(-1,2)
op_single_img = op_single_img * CB_SQUARE_SIZE
object_points = []

# Extract corner information from each of the images
for fname in fnames:

    # Load image, convert to grayscale
    img = cv2.imread(OPEN_PATH + fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find corners in the image
    ret, corners = cv2.findChessboardCorners(gray, (CB_ROWS, CB_COLS), None)

    # If corners found, continue, else skip this image, print a warning
    if ret:

        # Improve corner accuracy
        better_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Display to make sure corner points are correct
        cv2.drawChessboardCorners(img, (CB_ROWS, CB_COLS), better_corners, ret)
        cv2.imshow('img', img)

        # Wait for response
        if cv2.waitKey(0) & 0xFF == ord('y'):

            # Add points to the full list as well as another copy of object points
            object_points.append(op_single_img)
            image_points.append(better_corners)

            # Done with this frame
            print "-- processed " + fname + " --"

        # Otherwise, skip the frame
        else:
            print "-- skipped frame " + fname + " --"

    else:
        print "-- unable to find corners in " + fname + ", skipped frame --"

cv2.destroyAllWindows()

# Calibrate using the corner data
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, IMG_SIZE,
                                                   None, None, flags=cv2.CALIB_ZERO_TANGENT_DIST)

# Print results
print "\n=========\n RESULTS\n=========\n"
print "Dist. Coeffifients: "
print dist
print "\nCamera Matrix"
print mtx
print "\nData saved to " + SAVE_PATH

# Save calibration camera matrix and distortion coefficients to file for later use
data = {"camera_matrix": np.asarray(mtx).tolist(),
        "dist_coefficients": np.asarray(dist).tolist(),
        "image_points": np.asarray(image_points).tolist(),
        "object_points": np.asarray(object_points).tolist()}
out = open(SAVE_PATH, 'w')
yaml.dump(data, out)
