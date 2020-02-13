from camera import VideoStreamer, VideoWriter
from utils import file_utils
from abc import abstractmethod
from math import pi
import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time
from utils import file_utils

CALIBRATION_FILE = "camera_calibration/calibration_parameters/arducam.yaml"
POSE_DIR = "marker_detection/pose_data"
file_utils.make_dir(POSE_DIR)
POSE_FILE = file_utils.create_file_name_date() + ".txt" # Default pose file name

DEFAULT_FREQ = 30 # Hz

''' Marker Tracker Classes
    These classes are used to track a marker using a single camera. They maintain various image and marker data, and
    can be used to retrieve the pose relative to the camera. Each class is used to detect a different kind of marker.
    The classes are derived from the VideoStreamer class, which is used to retrieve images from a camera or video file 
    in a parallel thread.'''

# Abstract Base Class. Each marker tracker needs to define marker length, pose, the current frame,
# and some variables used for visualization.
class MarkerTracker(VideoStreamer):
    def __init__(self,
                 src=0,
                 use_pi=-1,
                 resolution=480,
                 framerate=30,
                 marker_length=0.24,
                 freq=DEFAULT_FREQ,
                 debug=0,
                 video_dir=None,
                 video_file=None,
                 pose_file=POSE_FILE):

        super(MarkerTracker, self).__init__(src=src,
                                            use_pi=use_pi,
                                            resolution=resolution,
                                            framerate=framerate)
        self.marker_length = marker_length
        self.pose = None # Stored as (1,3) np array of x,y,z
        self.marker_found = False
        self.cur_frame = None
        self.detected_frame = None # Store visual information about the marker
        self.scale_factor = None
        self.debug = debug
        self.true_alt = 0
        self.frequency = freq
        self.period = 1 / float(self.frequency)
        self.cur_frame_time = 0 # time at the beginning of tracking frame

        # Load camera calibration parameters
        self.camera_mat, self.dist_coeffs = file_utils.load_yaml(os.path.abspath(CALIBRATION_FILE))
        self.focal_length = self.camera_mat[1][1]

        # Save video and pose file
        if self.debug:
            self.video_writer = VideoWriter(video_dir=video_dir,
                                            video_file=video_file,
                                            resolution=resolution,
                                            framerate=framerate)

            # Create pose file name and open it
            # Name can be accepted by itself or with ".txt" at the end
            if pose_file is None:
                self.pose_file = POSE_DIR + "/" + POSE_FILE
            elif not pose_file.endswith(".txt"):
                self.pose_file = POSE_DIR + "/" + pose_file + ".txt"
            else:
                self.pose_file = POSE_DIR + "/" + pose_file

            self.pose_file = file_utils.open_file(self.pose_file)

        self.start_time = time.time()


    def get_true_alt(self):
        return self.true_alt

    def is_marker_found(self):
        return self.marker_found

    def get_marker_length(self):
        return self.marker_length

    def get_pose(self):
        return self.pose

    def get_debug(self):
        return self.debug

    def get_detected_frame(self):
        return self.detected_frame

    def get_scale_factor(self):
        return self.scale_factor

    def get_camera_mat(self):
        return self.camera_mat

    def get_dist_coeffs(self):
        return self.dist_coeffs

    def get_focal_length(self):
        return self.focal_length

    def set_pose(self, pose):
        self.pose = pose

    # Maintain a specific frequency of tracking
    def wait(self):
        try:
            time.sleep(self.period - (time.time() - self.cur_frame_time))
        except IOError:
            pass
            #print("WARNING: Desired frequency is too fast")

    def stop(self):
        self.vs.stop()
        if self.debug:
            self.video_writer.stop()

    # Used for debugging or recording purposes. Saves the pose to a .txt file.
    def save_pose(self):
        if self.pose is not None:
            self.pose_file.write("{} {} {} {} 0 0 0 0\n".format(time.time() - self.start_time,
                                                                self.pose[0],
                                                                self.pose[1],
                                                                self.pose[2]))

    # This is where the marker tracking happens. Various image processing is performed, and if the marker is detected,
    # the pose will be updated and a boolean will be returned.
    @abstractmethod
    def track_marker(self, alt):
        pass

    # Used for debugging or recording purposes. Will display images to your display and save a video.
    @abstractmethod
    def visualize(self):
        pass

    # Calculate the pixel to meters conversion for marker length
    @abstractmethod
    def calculate_scale_factor(self, args):
        pass

# Tracks a yellow marker.
# OLD MARKER LENGTH = 3.048
class ColorMarkerTracker(MarkerTracker):
    def __init__(self,
                 src=0,
                 use_pi=-1,
                 resolution=480,
                 framerate=30,
                 marker_length=0.24,
                 freq=DEFAULT_FREQ,
                 debug=0,
                 video_dir=None,
                 video_file=None,
                 pose_file=POSE_FILE):

        super(ColorMarkerTracker, self).__init__(src=src,
                                                 use_pi=use_pi,
                                                 resolution=resolution,
                                                 framerate=framerate,
                                                 marker_length=marker_length,
                                                 freq=freq,
                                                 debug=debug,
                                                 video_dir=video_dir,
                                                 video_file=video_file,
                                                 pose_file=pose_file)

        # Below the altitude threshold, dynamic Otsu thresholding is performed on the lightness spectrum
        # in the LAB color space. Above the threshold, binary thresholding is performed. This was determined to be the
        # best approach for detecting the yellow marker in a large altitude range. The altitude threshold should be
        # empirically determined.
        self.alt_thresh = 25

        # Stores each image processing step
        self.undistort_frame = None
        self.lab_space_frame = None
        self.thresh_yellow_frame = None
        self.thresh_light_frame = None
        self.thresh_sum_frame = None
        self.opening_frame = None
        self.processed_frame = None # Final thresholded frame
        self.depth = -1

    # This method tracks the color yellow. First, the image is undistorted to account for any lens distortion. Next,
    # it is converted from BGR to LAB color space. The LAB color space is great for color detection since it separates
    # the image into lightness (L), green -> red (*a), and blue -> yellow (*b). Various thresholds (explained within)
    # are applied to identify the yellow, and the thresholds are further refined to eliminate noise.
    #
    # After color detection, we search for a rectangle shape using contour approximation. This rules out all noise
    # left over. The pose is calculated and scaled using the known marker length.
    def track_marker(self, alt=25):
        self.cur_frame_time = time.time()
        self.cur_frame = self.read()
        self.true_alt = alt

        # Undistort image to get rid of fisheye distortion
        # TODO: Remove this undistortion?
        # TODO: Take video of takeoff, ascent, and travel to sample zone. Determine height needed to detect zone w/ and
        # TODO: w/o distortion, then make the decision
        self.undistort_frame = cv2.undistort(self.cur_frame, self.camera_mat, self.dist_coeffs)

        # Convert to LAB color space
        # LAB color space is used to separate lightness from color attributes
        # L is dark (0) to light (255)
        # A is Green (0) to Red (255)
        # B is Blue (0) to Yellow (255)
        self.lab_space_frame = cv2.cvtColor(self.undistort_frame, cv2.COLOR_BGR2Lab)

        # Blur the image to reduce noise
        self.lab_space_frame = cv2.GaussianBlur(self.lab_space_frame, (5,5), 0)

        # Lightness is thresholded since the yellow tarp will most likely be the lightest object in the image.
        # At lower altitudes, the marker takes up the majority of the image, and OTSU normalization is ideal, since
        # it dynamically chooses the threshold value to segment between "light" and "not light". At higher altitudes,
        # the marker is too small for OTSU to threshold well, so a binary threshold is used.
        if alt < self.alt_thresh:
            # Dynamically threshold lightness
            retval2, self.thresh_light_frame = cv2.threshold(self.lab_space_frame[:, :, 0], 200, 255,
                                                             cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        else:
            # Binary threshold lightness
            retval2, self.thresh_light_frame = cv2.threshold(self.lab_space_frame[:, :, 0], 200, 255,
                                                             cv2.THRESH_BINARY)

        # Threshold yellow; Everything from 0 to 127 in the B space is made 0 - these are blueish colors
        retval1, self.thresh_yellow_frame = cv2.threshold(self.lab_space_frame[:, :, 2], 127, 255,
                                                          cv2.THRESH_BINARY)

        # Combine the yellow and lightness thresholds, letting through only the pixels that are white in each image
        self.thresh_sum_frame = cv2.bitwise_and(self.thresh_yellow_frame, self.thresh_light_frame)

        # Create kernels for dilution and erosion operations; larger ksize means larger pixel neighborhood where the
        # operation is taking place
        se1 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(8, 8))
        se2 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(50, 50))

        # Perform "opening_frame." This is erosion followed by dilation, which reduces noise. The ksize is pretty small,
        # otherwise the white in the marker is eliminated
        self.opening_frame = cv2.morphologyEx(self.thresh_sum_frame, cv2.MORPH_OPEN, se1)

        # Perform "closing." This is dilution followed by erosion, which fills in black gaps within the marker. This is
        # necessary if the lightness threshold is not able to get the entire marker at lower altitudes
        self.processed_frame = cv2.morphologyEx(self.opening_frame, cv2.MORPH_CLOSE, se2)

        # Find contours
        # TODO: The return values here are dependent on Opencv 2,3,or 4, so either be sure of which version
        #  we are using or add some code to check the version
        _, contours, hierarchy = cv2.findContours(self.processed_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # Find corners of the contour
        if len(contours) >= 1:

            # Take contour w/ max area; the marker will always be the largest contour in the image
            c = max(contours, key=cv2.contourArea)

            # Approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)
            # Keep going if the contour is a square
            if self.is_rectangle(approx):
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

                # Find positional error from center of platform; Pixels
                err_x = cX - (self.get_resolution()[0] / 2)
                err_y = (self.get_resolution()[1] / 2) - cY

                # Calculate the scale factor using the pixel marker length
                self.calculate_scale_factor(peri)

                # Convert to meters
                err_x *= self.scale_factor
                err_y *= self.scale_factor

                self.set_pose(np.array([err_x, err_y, 0]))

                # Debug
                self.marker_found = True
                if self.get_debug():
                    self.visualize_marker_pose(extLeft, extRight, extTop, extBot,
                                                               cX, cY, self.depth, c)
                    self.visualize()
                    self.save_pose()

                return True

        # Debug
        self.pose = None
        self.marker_found = False
        if self.get_debug():
            self.visualize()
            self.save_pose()

        return False

    # For debugging and recording. If on the Pi, we only want to save videos, since the GUI may not be available.
    def visualize(self):
        if not self.marker_found:
            self.detected_frame = self.cur_frame
        else:
            print("Marker Pose: {}".format(self.pose))

        if self.use_pi:
            self.video_writer.write(self.detected_frame)
        else:
            # Reduce all 9 image dimensions so they can be merged together
            reduced_dim = (self.resolution[0] / 3, self.resolution[1] / 3)

            # Convert the mask images into three-channel images so they can be merged with the rest
            thresh_yellow_frame_three = cv2.merge(
                (self.thresh_yellow_frame, self.thresh_yellow_frame, self.thresh_yellow_frame))
            thresh_light_frame_three = cv2.merge(
                (self.thresh_light_frame, self.thresh_light_frame, self.thresh_light_frame))
            thresh_sum_frame_three = cv2.merge(
                (self.thresh_sum_frame, self.thresh_sum_frame, self.thresh_sum_frame))
            opening_frame_three = cv2.merge((self.opening_frame, self.opening_frame, self.opening_frame))
            processed_three = cv2.merge((self.processed_frame, self.processed_frame, self.processed_frame))

            # Resize
            img1 = cv2.resize(self.read(), reduced_dim, interpolation=cv2.INTER_AREA)
            img2 = cv2.resize(self.undistort_frame, reduced_dim, interpolation=cv2.INTER_AREA)
            img3 = cv2.resize(self.lab_space_frame, reduced_dim, interpolation=cv2.INTER_AREA)
            img4 = cv2.resize(thresh_yellow_frame_three, reduced_dim, interpolation=cv2.INTER_AREA)
            img5 = cv2.resize(thresh_light_frame_three, reduced_dim, interpolation=cv2.INTER_AREA)
            img6 = cv2.resize(thresh_sum_frame_three, reduced_dim, interpolation=cv2.INTER_AREA)
            img7 = cv2.resize(opening_frame_three, reduced_dim, interpolation=cv2.INTER_AREA)
            img8 = cv2.resize(processed_three, reduced_dim, interpolation=cv2.INTER_AREA)
            img9 = cv2.resize(self.detected_frame, reduced_dim, interpolation=cv2.INTER_AREA)

            # Merge into one image
            row1 = np.concatenate((img1, img2, img3), axis=1)
            row2 = np.concatenate((img4, img5, img6), axis=1)
            row3 = np.concatenate((img7, img8, img9), axis=1)
            all_frames = np.concatenate((row1, row2, row3), axis=0)

            # Deal with any rounding errors
            if (all_frames.shape[0], all_frames.shape[1]) != self.resolution:
                all_frames = cv2.resize(all_frames, self.resolution, interpolation=cv2.INTER_AREA)

            # Display
            cv2.namedWindow("Original, Undistorted, Lab // "
                            "Yellow Thresh, Light Thresh, Sum Thresh // "
                            "opening_frame, Closing, Final", cv2.WINDOW_FULLSCREEN)
            cv2.imshow("Original, Undistorted, Lab // "
                       "Yellow Thresh, Light Thresh, Sum Thresh // "
                       "opening_frame, Closing, Final", all_frames)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

            # Save video
            self.video_writer.write(all_frames)

    # Draw the marker corners and pose info on the image for debugging.
    def visualize_marker_pose(self, extLeft, extRight, extTop, extBot, cX, cY, depth, c):
        self.detected_frame = self.cur_frame

        # Draw extreme points on image
        cv2.circle(self.detected_frame, extLeft, 8, (255, 0, 0), -1)
        cv2.circle(self.detected_frame, extRight, 8, (255, 0, 0), -1)
        cv2.circle(self.detected_frame, extTop, 8, (255, 0, 255), -1)
        cv2.circle(self.detected_frame, extBot, 8, (0, 0, 255), -1)

        # Draw center
        cv2.circle(self.detected_frame, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(self.detected_frame, "center", (cX + 20, cY + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Draw depth
        cv2.putText(self.detected_frame, "Distance: {}".format(np.around(depth, 1)), (cX - 40, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Draw true altitude
        cv2.putText(self.detected_frame, "Altitude: {}".format(np.around(self.true_alt, 1)), (cX - 40, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Draw XY error
        cv2.putText(self.detected_frame,
                    "XY Error: {}, {}".format(np.around(self.pose[0], 1), np.around(self.pose[1], 1)),
                    (cX - 40, cY - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Draw contours on image
        cv2.drawContours(self.detected_frame, [c], 0, (0, 255, 0), 3)

    def calculate_scale_factor(self, peri):
        # Find the distance to the platform
        # TODO: This will most likely be replaced with depth from the Realsense camera
        pixel_length = peri / 4
        self.depth = (self.focal_length / pixel_length) * self.marker_length

        # Determine the *approximate* scale factor
        self.scale_factor = self.marker_length / pixel_length

    # Checks if the contour approximation is a rectangle.
    def is_rectangle(self, approx):

        if len(approx) == 4:

            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)

            # a rectangle will have an aspect ratio that is within this range
            return True if 0.6 <= ar <= 1.5 else False
        return False


# Tracks an Aruco marker. Marker length is expected in meters
class ArucoTracker(MarkerTracker):
    def __init__(self,
                 src=0,
                 use_pi=-1,
                 resolution=480,
                 framerate=30,
                 marker_length=0.159,
                 freq=DEFAULT_FREQ,
                 debug=0,
                 video_dir=None,
                 video_file=None,
                 pose_file=POSE_FILE):

        super(ArucoTracker, self).__init__(src=src,
                                           use_pi=use_pi,
                                           resolution=resolution,
                                           framerate=framerate,
                                           marker_length=marker_length,
                                           freq=freq,
                                           debug=debug,
                                           video_dir=video_dir,
                                           video_file=video_file,
                                           pose_file=pose_file)

        # Create aruco dictionary
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_param = aruco.DetectorParameters_create()  # Default parameters

        self.rvec = None

    # The Aruco marker is tracked here. The pose is saved if the marker is found.
    # Returns True if found, else False.
    def track_marker(self, alt=0):
        self.cur_frame_time = time.time()

        # Update current image
        self.cur_frame = self.read()

        # Detect the marker
        corners, ids, rejectedImgPoints = aruco.detectMarkers(self.cur_frame, self.aruco_dict,
                                                              parameters=self.aruco_param)

        # If a marker is found, estimate the pose
        if ids is not None:
            marker_corner = corners[0]  # Corners are given clockwise starting at top left
            self.rvec, self.pose = aruco.estimatePoseSingleMarkers(marker_corner, self.marker_length,
                                                                   self.camera_mat, self.dist_coeffs)

            # Reduce extra array dimensions
            self.rvec = self.rvec[0][0]
            self.pose = self.pose[0][0]

            # Calculate the scale factor conversion from marker length in pixels to meters
            self.calculate_scale_factor(marker_corner)

            # Debug
            self.marker_found = True
            if self.get_debug():
                if self.detected_frame is None:
                    self.detected_frame = self.cur_frame

                aruco.drawDetectedMarkers(self.detected_frame, corners, ids)
                self.detected_frame = aruco.drawAxis(self.cur_frame, self.camera_mat, self.dist_coeffs,
                                                     self.rvec, self.pose, 0.1)
                self.visualize()
                self.save_pose()
        else:
            # Debug
            self.pose = None
            self.marker_found = False
            if self.get_debug():
                self.visualize()
                self.save_pose()

    # Visualize the Aruco marker detection. If on the Pi, only saves a video, since the GUI may not be available.
    def visualize(self):

        if not self.marker_found:
            self.detected_frame = self.cur_frame
        else:
            # Put marker pose on image
            str_position = "Marker Position (m) x=%2.2f y=%2.2f z=%2.2f"%(self.pose[0], self.pose[1], self.pose[2])
            cv2.putText(self.detected_frame, str_position, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0))
            #print("Marker Pose: {}".format(self.pose))

        # Draw UAV body axis for reference
        cv2.line(self.detected_frame,
                 pt1=(self.resolution[0] / 2, self.resolution[1] / 2),
                 pt2=(100, self.resolution[1] / 2),
                 color=(0, 0, 255),
                 thickness=5)
        cv2.putText(self.detected_frame,"X",(75, self.resolution[1] / 2 + 5), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255))
        cv2.line(self.detected_frame,
                 pt1=(self.resolution[0] / 2, self.resolution[1] / 2),
                 pt2=(self.resolution[0] / 2, 50),
                 color=(0, 255, 0),
                 thickness=5)
        cv2.putText(self.detected_frame,"Y",(self.resolution[0] / 2 - 5, 35), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0))

        if not self.use_pi:
            cv2.imshow("Aruco Tracker", self.detected_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

        self.video_writer.write(self.detected_frame)

    def calculate_scale_factor(self, corners):
        # Calculate the pixel scale factor (pixel -> meters unit conversion)
        x = corners[0][1][0] - corners[0][0][0]
        y = corners[0][1][1] - corners[0][0][1]
        marker_length_pixels = np.sqrt(np.square(x) + np.square(y))

        self.scale_factor = self.marker_length / marker_length_pixels

    # Use this function to create and save an Aruco marker if you do not already have one.
    # Type s once the marker is displayed to save the image
    def create_and_save_marker(self):

        # Second param is ID - change if you want a different marker
        # Third param is border bits - leave it large
        marker_img = aruco.drawMarker(self.aruco_dict, 20, 600)

        cv2.imshow("Aruco axes", marker_img)
        k = cv2.waitKey(0)
        if k == 27:  # wait for ESC key to exit
            cv2.destroyAllWindows()
        elif k == ord('s'):  # wait for 's' key to save and exit
            cv2.imwrite('marker_detection/images/aruco_marker.png', marker_img)
            cv2.destroyAllWindows()

    def get_marker_rotation(self):
        return self.rvec

    def get_pose_body_coords(self):
        # Flip signs because aruco has bottom right and down as positive axis
        pose = np.array([self.pose[0], -self.pose[1]])

        # Convert camera resolution from pixels to meters; reshape from dimensions (1,2) to (2,)
        cam_dimensions = self.scale_camera_dimensions()

        return cam_dimensions / 2 - pose

    def scale_camera_dimensions(self):
        w = 2 * self.pose[2] * np.tan(72.5 * pi/180)
        h = 2 * self.pose[2] * np.tan(38.5 * pi/180)
        return np.array([w, h])


