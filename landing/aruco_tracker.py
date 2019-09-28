import cv2
import cv2.aruco as aruco

MARKER_LENGTH = 0.05 # Length of marker side in meters

def create_and_save_marker(dict):
    marker_img = aruco.drawMarker(dict, 20, 600)
    cv2.imshow("Aruco axes", marker_img)
    k = cv2.waitKey(0)
    if k == 27:  # wait for ESC key to exit
        cv2.destroyAllWindows()
    elif k == ord('s'):  # wait for 's' key to save and exit
        cv2.imwrite('aruco_marker.png', marker_img)
        cv2.destroyAllWindows()


# TODO: Camera calibration
cameraMatrix = 0
distCoeffs = 0

# TODO: Test image
img = 0

# Create aruco dictionary that the marker is a part of
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_param = aruco.DetectorParameters_create() # Default parameters

create_and_save_marker(aruco_dict)

# corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=aruco_param)
#
# # If a marker is found, estimate the pose
# if ids is not None:
#     corners = corners[0] # Corners are given clockwise (starts where?)
#     rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, cameraMatrix, distCoeffs)
#
#     # Draw axis for debugging
#     img = aruco.drawAxis(img, cameraMatrix, distCoeffs, rvec, tvec, 0.1)
#     cv2.imshow("Aruco axes", img)
#     k = cv2.waitKey(0)
#     if k == 27:  # wait for ESC key to exit
#         cv2.destroyAllWindows()
#     elif k == ord('s'):  # wait for 's' key to save and exit
#         cv2.imwrite('sum.png', img)
#         cv2.destroyAllWindows()

