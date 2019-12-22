import cv2
import cv2.aruco as aruco
from marker_detection import MarkerDetector
from builtins import super

# Files
IMAGE_FILE = 'marker_test.jpg'
VIDEO_FILE_SAVE = 'videos/aruco_detection_0.mp4'
# VIDEO_FILE_STREAM = "../flight_videos/flight_1.mp4"
VIDEO_FILE_STREAM = 0

DEBUG = 1

class ArucoTracker(MarkerDetector):
    def __init__(self, debug=0):
        super(ArucoTracker, self).__init__(debug)
        self.marker_length = 0.159

        # Create aruco dictionary
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_param = aruco.DetectorParameters_create()  # Default parameters

        self.rvec = None
        self.tvec = None

    def debug_images(self):
        if self.debug:
            if self.marker_found:
                self.detected_img = aruco.drawAxis(self.img, self.camera_mat, self.dist_coeffs, self.rvec, self.tvec, 0.1)
                cv2.imshow("Aruco Tracker", self.detected_img)
                print(self.tvec)
            else:
                cv2.imshow("Aruco Tracker", self.img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

    def create_and_save_marker(self, dict):
        marker_img = aruco.drawMarker(dict, 20, 600)
        cv2.imshow("Aruco axes", marker_img)
        k = cv2.waitKey(0)
        if k == 27:  # wait for ESC key to exit
            cv2.destroyAllWindows()
        elif k == ord('s'):  # wait for 's' key to save and exit
            cv2.imwrite('aruco_marker.png', marker_img)
            cv2.destroyAllWindows()

    def track_marker(self, img, alt=25):
        self.img = img

        # Find the marker
        corners, ids, rejectedImgPoints = aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_param)
        # If a marker is found, estimate the pose
        if ids is not None:
            corners = corners[0] # Corners are given clockwise starting at top left
            self.rvec, self.tvec = aruco.estimatePoseSingleMarkers(corners, self.marker_length,
                                                                               self.camera_mat, self.dist_coeffs)

            # Reduce extra array dimensions
            self.rvec = self.rvec[0][0]
            self.tvec = self.tvec[0][0]

            # Draw axis for debugging
            self.marker_found = True
            self.debug_images()
            return True

        self.marker_found = False
        self.detected_img = self.img
        self.debug_images()
        return False

    def get_marker_pose(self):
        return self.tvec

    def get_marker_rotation(self):
        return self.rvec


def main():
    aruco_detector = ArucoTracker(DEBUG)

    # Set up video capture
    cap = cv2.VideoCapture(VIDEO_FILE_STREAM)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(VIDEO_FILE_SAVE, fourcc, aruco_detector.frame_rate, aruco_detector.resolution)

    while True:
        ret, img = cap.read()
        marker_found = aruco_detector.track_marker(img)

        if marker_found and aruco_detector.debug:
            out.write(aruco_detector.get_detected_image())

    out.release()


if __name__ == "__main__":
    main()


