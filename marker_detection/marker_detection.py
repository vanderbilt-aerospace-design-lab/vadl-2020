import cv2
import numpy as np
from utils import cv_utils

# Files
IMAGE_FILE = 'marker_test.jpg'
VIDEO_FILE_SAVE = 'videos/marker_detection_0.mp4'
VIDEO_FILE_STREAM = "../flight_videos/flight_100ft.mp4"
# VIDEO_FILE_STREAM = 0

CALIBRATION_FILE = "../camera_calibration/calibration_data/arducam.yaml"

ALT_THRESH = 25

class MarkerDetector:
    def __init__(self, debug=0):
        self.err_x = 0
        self.err_y = 0
        self.marker_length = 17 * 0.0254 # Meters
        self.marker_area = self.marker_length * self.marker_length
        self.resolution = (1920, 1080)
        self.frame_rate = 30.0
        self.marker_found = False
        self.img = None
        self.undistort_img = None
        self.lab_space_img = None
        self.sum_thresh = None
        self.detected_img = None
        self.camera_mat, self.dist_coeffs = cv_utils.load_yaml(CALIBRATION_FILE)
        self.focal_length = self.camera_mat[1][1]
        self.debug = debug

    def debug_images(self):
        if self.debug:
            reduced_dim = (self.resolution[0] / 4, self.resolution[1] / 4)
            three_thresh = cv2.merge((self.sum_thresh,self.sum_thresh,self.sum_thresh))

            # Resize
            img1 = cv2.resize(self.img, reduced_dim, interpolation=cv2.INTER_AREA)
            img2 = cv2.resize(self.undistort_img, reduced_dim, interpolation=cv2.INTER_AREA)
            img3 = cv2.resize(self.lab_space_img, reduced_dim, interpolation=cv2.INTER_AREA)
            img4 = cv2.resize(three_thresh, reduced_dim, interpolation=cv2.INTER_AREA)
            row1 = np.concatenate((img1, img2), axis=1)
            row2 = np.concatenate((img3, img4), axis=1)
            all_imgs = np.concatenate((row1, row2), axis=0)

            cv2.imshow("Original, Undistorted, Lab, Thresholded", all_imgs)
            cv2.imshow('Detected Image', self.detected_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

    def visualize_marker_pose(self, extLeft, extRight, extTop, extBot, cX, cY, depth, c):
        if self.debug:
            self.detected_img = self.img.copy()
            # Draw extreme points on image
            cv2.circle(self.detected_img, extLeft, 8, (255, 0, 0), -1)
            cv2.circle(self.detected_img, extRight, 8, (255, 0, 0), -1)
            cv2.circle(self.detected_img, extTop, 8, (255, 0, 255), -1)
            cv2.circle(self.detected_img, extBot, 8, (0, 0, 255), -1)

            # Draw center
            cv2.circle(self.detected_img, (cX, cY), 7, (255, 255, 255), -1)
            cv2.putText(self.detected_img, "center", (cX + 20, cY + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw depth
            cv2.putText(self.detected_img, "Distance: {}".format(np.around(depth, 1)), (cX - 40, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw XY error
            cv2.putText(self.detected_img, "XY Error: {}, {}".format(np.around(self.err_x, 1), np.around(self.err_y, 1)), (cX - 40, cY - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw contours on image
            cv2.drawContours(self.detected_img, [c], 0, (0, 255, 0), 3)

    def is_square(self, approx):
        if len(approx) == 4:

            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)

            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            return True if 0.75 <= ar <= 1.25 else False
        return False

    def get_marker_location(self):
        return self.err_x, self.err_y

    def get_detected_image(self):
        return self.detected_img

    def find_marker(self, img, alt=25):
        self.img = img

        # Undistort image to get rid of fisheye distortion
        self.undistort_img = cv2.undistort(self.img, self.camera_mat, self.dist_coeffs)

        # Convert to LAB color space
        # LAB color space is used to separate lightness from color attributes
        # L is dark (0) to light (255)
        # A is Green (0) to Red (255)
        # B is Blue (0) to Yellow (255)
        self.lab_space_img = cv2.cvtColor(self.undistort_img, cv2.COLOR_BGR2Lab)

        # Blur the image to reduce noise
        self.lab_space_img = cv2.GaussianBlur(self.lab_space_img, (5,5), 0)

        # Lightness is thresholded since the yellow tarp will most likely be the lightest object in the image.
        # At lower altitudes, the marker takes up the majority of the image, and OTSU normalization is ideal, since
        # it dynamically chooses the threshold value to segment between "light" and "not light". At higher altitudes,
        # the marker is too small for OTSU to threshold well, so a binary threshold is used.
        if alt < ALT_THRESH:
            # Dynamically threshold lightness
            retval2, thresh_light = cv2.threshold(self.lab_space_img[:, :, 0], 200, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            cv2.imshow("Thresh_lightness", thresh_light)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
        else:
            # Binary threshold lightness
            retval2, thresh_light = cv2.threshold(self.lab_space_img[:, :, 0], 200, 255, cv2.THRESH_BINARY)
            cv2.imshow("Thresh_lightness", thresh_light)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

        # Threshold yellow; Everything from 0 to 127 in the B space is made 0 - these are blueish colors
        retval1, thresh_yellow = cv2.threshold(self.lab_space_img[:, :, 2], 127, 255, cv2.THRESH_BINARY)
        cv2.imshow("Thresh_yellow", thresh_yellow)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

        # Combine the yellow and lightness thresholds, letting through only the pixels that are white in each image
        self.sum_thresh = cv2.bitwise_and(thresh_yellow, thresh_light)
        cv2.imshow("sum_thresh", self.sum_thresh)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

        # self.sum_thresh = cv2.dilate(self.sum_thresh, se2)
        # cv2.imshow("dilate", self.sum_thresh)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()
        #
        # self.sum_thresh = cv2.erode(self.sum_thresh, se1)
        # cv2.imshow("dilate and erode", self.sum_thresh)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()

        # Create kernels for dilution and erosion operations; larger ksize means larger pixel neighborhood where the
        # operation is taking place
        se1 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(8, 8))
        se2 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(50, 50))

        # Perform "opening." This is erosion followed by dilation, which reduces noise. The ksize is pretty small,
        # otherwise the white in the marker is eliminated
        self.sum_thresh = cv2.morphologyEx(self.sum_thresh, cv2.MORPH_OPEN, se1)

        # Perform "closing." This is dilution followed by erosion, which fills in black gaps within the marker. This is
        # necessary if the lightness threshold is not able to get the entire marker at lower altitudes
        self.sum_thresh = cv2.morphologyEx(self.sum_thresh, cv2.MORPH_CLOSE, se2)
        cv2.imshow("Closing", self.sum_thresh)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

        # Find contours
        # TODO: The return values here are dependent on Opencv 2,3,or 4, so either be sure of which version
        #  we are using or add some code to check the version
        _, contours, hierarchy = cv2.findContours(self.sum_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # Find corners of the contour
        if len(contours) >= 1:

            # Take contour w/ max area; the marker will always be the largest contour in the image
            c = max(contours, key=cv2.contourArea)

            # Approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)
            # Keep going if the contour is a square
            if self.is_square(approx):
                c = approx

                # Find extreme points
                extLeft = tuple(c[c[:, :, 0].argmin()][0])
                extRight = tuple(c[c[:, :, 0].argmax()][0])
                extTop = tuple(c[c[:, :, 1].argmin()][0])
                extBot = tuple(c[c[:, :, 1].argmax()][0])

                # Find the contour center
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Find the distance to the platform
                # TODO: This will most likely be replaced with depth from the Realsense camera
                pixel_length = peri / 4
                depth = (self.focal_length / pixel_length) * self.marker_length

                # Determine the *approximate* scale factor
                scale_factor = self.marker_length / pixel_length

                # Find positional error from center of platform; Pixels
                self.err_x = cX - (self.resolution[0] / 2)
                self.err_y = (self.resolution[1] / 2) - cY

                # Convert to inches
                self.err_x *= scale_factor
                self.err_y *= scale_factor

                self.visualize_marker_pose(extLeft, extRight, extTop, extBot,
                                                               cX, cY, depth, c)
                self.debug_images()
                return True

        self.detected_img = self.img.copy()
        self.debug_images()
        return False

def main():
    marker_detector = MarkerDetector(1)

    # Set up video capture
    cap = cv2.VideoCapture(VIDEO_FILE_STREAM)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(VIDEO_FILE_SAVE, fourcc, marker_detector.frame_rate, marker_detector.resolution)

    while True:
        ret, img = cap.read()
        marker_found = marker_detector.find_marker(img)

        if marker_found and marker_detector.debug:
            out.write(marker_detector.get_detected_image())

    out.release()
if __name__ == "__main__":
    main()
