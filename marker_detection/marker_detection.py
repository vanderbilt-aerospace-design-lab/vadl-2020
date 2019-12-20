import cv2
import numpy as np
from utils import cv_utils

# Files
IMAGE_FILE = 'marker_test.jpg'
VIDEO_FILE_SAVE = 'videos/marker_detection_0.mp4'
VIDEO_FILE_STREAM = "../flight_videos/flight_0.mp4"
# VIDEO_FILE_STREAM = 0

CALIBRATION_FILE = "../camera_calibration/calibration_data/arducam.yaml"

class MarkerDetector:
    def __init__(self):
        self.err_x = 0
        self.err_y = 0
        self.marker_length = 17 # Inches
        self.marker_area = self.marker_length * self.marker_length
        self.resolution = (1920, 1080)
        self.frame_rate = 30.0
        self.marker_found = False
        self.img = None
        self.undistort_image = None
        self.lab_space_img = None
        self.sum_thresh = None
        self.detected_img = None
        self.camera_mat, self.dist_coeffs = cv_utils.load_yaml(CALIBRATION_FILE)
        self.focal_length = self.camera_mat[1][1]
        self.debug = 1


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
            if self.detected_img is not None:
                cv2.imshow('Detected Image', self.detected_img)
            else:
                cv2.imshow('Detected Image', self.img)
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
            return True if 0.85 <= ar <= 1.15 else False
        return False

    def get_marker_location(self):
        return self.err_x, self.err_y

    def get_detected_image(self):
        return self.detected_img

    def find_marker(self, img):
        self.img = img

        # Undistort image
        self.undistort_img = cv2.undistort(self.img, self.camera_mat, self.dist_coeffs)

        # TODO: Choose between LAB and HSV color space
        self.lab_space_img = cv2.cvtColor(self.undistort_img, cv2.COLOR_BGR2HSV)

        # Blur the image to reduce noise
        self.lab_space_img = cv2.GaussianBlur(self.lab_space_img, (5,5), 0)

        # Threshold image based on yellow; otsu thresholding
        retval1, thresh1 = cv2.threshold(self.lab_space_img[:, :, 2], -50, 255, cv2.THRESH_OTSU)

        # TODO: Is this necessary?
        # Binary OR Otsu thresholding
        retval2, thresh2 = cv2.threshold(self.lab_space_img[:, :, 0], 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        self.sum_thresh = cv2.bitwise_and(thresh1, thresh2)

        # TODO: Is this necessary?
        # TODO: Try adding a Guassian blur
        # Dilate and erode the image
        # This reduces the noise residual in the threshold
        se1 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(20, 20))
        se2 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(7, 7))
        self.sum_thresh = cv2.dilate(self.sum_thresh, se2)
        self.sum_thresh = cv2.erode(self.sum_thresh, se1)

        # Find contours
        # TODO: The return values here are dependent on Opencv 2,3,or 4, so either be sure of which version
        #  we are using or add some code to check the version
        _, contours, hierarchy = cv2.findContours(self.sum_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # Find corners of the contour
        if len(contours) >= 1:

            # Take contour w/ max area
            c = max(contours, key=cv2.contourArea)

            # Approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.015 * peri, True)
            # Keep going if the contour is a square
            if self.is_square(approx):
                c = approx
                peri = cv2.arcLength(c, True)

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
                pixel_length = peri / 4
                depth = (self.focal_length / pixel_length) * self.marker_length
                scale_factor = self.marker_length / pixel_length

                # Find positional error from center of platform
                # Pixels
                self.err_x = cX - (self.resolution[0] / 2)
                self.err_y = (self.resolution[1] / 2) - cY

                # Convert to inches
                self.err_x *= scale_factor
                self.err_y *= scale_factor

                self.visualize_marker_pose(extLeft, extRight, extTop, extBot,
                                                               cX, cY, depth, c)
                self.debug_images()
                return True
        self.detected_img = None
        self.debug_images()
        return False

    # # Record video and track the marker position.
    # def track_marker(self):
    #
    #     # Load calibration parameters
    #     camera_mat, dist_coeffs = cv_utils.load_yaml(CALIBRATION_FILE)
    #
    #     # Set up video capture
    #     cap = cv2.VideoCapture(VIDEO_FILE_STREAM)
    #     fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #     out = cv2.VideoWriter(VIDEO_FILE_SAVE, fourcc, FRAME_RATE, RESOLUTION)
    #
    #     while True:
    #         ret, img = cap.read()
    #         # img = cv2.imread("marker_test.jpg")
    #
    #         # Undistort image
    #         undistort_img = cv2.undistort(img, camera_mat, dist_coeffs)
    #
    #         # TODO: Choose between LAB and HSV color space
    #         lab_space_img = cv2.cvtColor(undistort_img, cv2.COLOR_BGR2Lab)
    #
    #         # Threshold image based on yellow; otsu thresholding
    #         retval1, thresh1 = cv2.threshold(lab_space_img[:, :, 2], 0, 255, cv2.THRESH_OTSU)
    #
    #         # TODO: Is this necessary?
    #         # Binary OR Otsu thresholding
    #         retval2, thresh2 = cv2.threshold(lab_space_img[:, :, 0], 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    #         sum_thresh = cv2.bitwise_and(thresh1, thresh2)
    #
    #         # TODO: Is this necessary?
    #         # TODO: Try adding a Guassian blur
    #         # Dilate and erode the image
    #         # This reduces the noise residual in the threshold
    #         se1 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(20,20))
    #         se2 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(7, 7))
    #         sum_thresh = cv2.dilate(sum_thresh, se2)
    #         sum_thresh = cv2.erode(sum_thresh, se1)
    #
    #         # Find contours
    #         # TODO: The return values here are dependent on Opencv 2,3,or 4, so either be sure of which version
    #         #  we are using or add some code to check the version
    #         _, contours, hierarchy = cv2.findContours(sum_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    #
    #         # Find corners of the contour
    #         if len(contours) >= 1:
    #             final_contour = None
    #             # Take contour w/ max area
    #             c = max(contours, key=cv2.contourArea)
    #
    #             # Approximate the contour
    #             peri = cv2.arcLength(c, True)
    #             approx = cv2.approxPolyDP(c, 0.03 * peri, True)
    #
    #             if len(approx) == 4:
    #                 c = approx
    #                 peri = cv2.arcLength(c, True)
    #
    #             # Find extreme points
    #             extLeft = tuple(c[c[:, :, 0].argmin()][0])
    #             extRight = tuple(c[c[:, :, 0].argmax()][0])
    #             extTop = tuple(c[c[:, :, 1].argmin()][0])
    #             extBot = tuple(c[c[:, :, 1].argmax()][0])
    #
    #             # Find the contour center
    #             M = cv2.moments(c)
    #             cX = int(M["m10"] / M["m00"])
    #             cY = int(M["m01"] / M["m00"])
    #
    #             # Find the distance to the platform
    #             pixel_length = peri / 4
    #             focal_length = camera_mat[1][1]  # Taken from the calibration file
    #             depth = (focal_length / pixel_length) * MARKER_LENGTH
    #             scale_factor = MARKER_LENGTH / pixel_length
    #
    #             # Find positional error from center of platform
    #             # Pixels
    #             err_x = cX - (RESOLUTION[0]/2)
    #             err_y = (RESOLUTION[1]/2) - cY
    #
    #             # Convert to inches
    #             err_x *= scale_factor
    #             err_y *= scale_factor
    #
    #             detected_img = visualize_marker_pose(img, extLeft, extRight, extTop, extBot,
    #                                                  cX, cY, depth, err_x, err_y, c)
    #
    #             # Write to video
    #             out.write(detected_img)
    #
    #             # Display all steps for troubleshooting
    #             if DEBUG:
    #                 images = [img, undistort_img, lab_space_img, sum_thresh, detected_img]
    #                 debug_images(images)
    #             else :
    #                 # Just show final image
    #                 cv2.imshow('img', detected_img)
    #                 if cv2.waitKey(1) & 0xFF == ord('q'):
    #                     break
    #
    #     out.release()

def main():
    marker_detector = MarkerDetector()

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
