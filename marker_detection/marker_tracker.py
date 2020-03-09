import os
import cv2
import time
import numpy as np
import cv2.aruco as aruco
from abc import abstractmethod

# Custom packages
from camera import VideoStreamer, VideoWriter
from utils import file_utils

CALIBRATION_FILE = "camera_calibration/calibration_parameters/arducam.yaml"
POSE_DIR = "marker_detection/logs/pose_data"
file_utils.make_dir(POSE_DIR)
POSE_FILE = file_utils.create_file_name_date() + ".txt" # Default pose file name

## Default Marker Tracker variables ##
SRC = 0                 # 0 for webcam or a string to video file
USE_PI = -1             # > 0 if running on the Pi
RESOLUTION = 480        # Num. of vertical pixels in camera resolution
FRAMERATE = 30          # Camera framerate
FPS_VID = 15            # Saved video framerate
FREQ = 20               # Default frequency at which to track the marker (Hz)
DEBUG = 0               # Debug mode (described below)
VIDEO_DIR = None        # Directory to save debug video
VIDEO_FILE = None       # Name of debug video file

# Default red marker dimensions; Meters
RED_MARKER_WIDTH = 4 * 0.0254
RED_MARKER_HEIGHT = 1 * 0.0254

# Default yellow marker dimensions; Meters
YELLOW_MARKER_WIDTH = 2.44
YELLOW_MARKER_HEIGHT = 2.44

# Default Aruco marker dimensions; Meters
ARUCO_MARKER_WIDTH = 0.159
ARUCO_MARKER_HEIGHT = 0.159

# Aspect Ratio margin; subtracted from minimum A.R. and added to maximum A.R.
# Change this to let through more approximate rectangles or to be stricter about the detected marker shape
AR_MARGIN = 0.4


# ---------------------------------------- Marker Tracker Classes ------------------------------------------------- #
#   These classes are used to track a marker using a single camera. Tracking consists of image processing,
#   some filtering to eliminate false positives, and pose estimation. Each class detect a different kind of marker,
#   such as different colored markers or an Aruco marker. The classes are derived from the VideoStreamer class,
#   which is used to retrieve images from a camera or video file in a parallel thread.
#
#   The classes currently only work for rectangular markers, but could  easily be extended to a new marker type.
#   All that is required is to define new marker tracking functions and redefine some variables.
# ----------------------------------------------------------------------------------------------------------------- #

# Abstract base class. Each child marker tracker class must define functions to track the marker,
# calculate its depth from the camera, and to visualize the algorithms.
class MarkerTracker(VideoStreamer):
    def __init__(self,
                 src=SRC,
                 use_pi=USE_PI,
                 resolution=RESOLUTION,
                 framerate=FRAMERATE,
                 fps_vid=FPS_VID,
                 marker_width=YELLOW_MARKER_WIDTH,
                 marker_height=YELLOW_MARKER_HEIGHT,
                 freq=FREQ,
                 debug=DEBUG,
                 video_dir=VIDEO_DIR,
                 video_file=VIDEO_FILE,
                 pose_file=POSE_FILE):

        # Initialize parent class
        super(MarkerTracker, self).__init__(src=src,
                                            use_pi=use_pi,
                                            resolution=resolution,
                                            framerate=framerate)

        # Marker dimensions; marker_length represents the largest dimension of the marker
        self.marker_width = marker_width
        self.marker_height = marker_height
        self.marker_length = np.amax((self.marker_width, self.marker_height))

        # Width / Height or Height / Width
        # Stored as a list of [min, max]
        aspect_ratio_min = np.amin((self.marker_width / self.marker_height, self.marker_height / self.marker_width))
        aspect_ratio_max = np.amax((self.marker_width / self.marker_height, self.marker_height / self.marker_width))
        self.aspect_ratio = [aspect_ratio_min - AR_MARGIN, aspect_ratio_max + AR_MARGIN]

        # Fast way to check what kind of marker this is
        # Current options are: red, yellow, aruco
        self.marker_type = None

        # Tracking frequency
        self.frequency = freq
        self.period = 1 / float(self.frequency)
        self.cur_frame_time = 0  # set at the beginning of each tracking loop

        # Images
        self.cur_frame = None # Most recent frame from the camera
        self.detected_frame = None # Visualize the detected marker

        # The actual altitude of the UAV.
        # This is needed to determine whether a dynamic or binary threshold is used.
        self.true_alt = 0

        # Conversion between pixels and meters; pose = pixel_pose * scale_factor
        self.scale_factor = None

        # Marker tracking results
        self.depth = -1 # Depth (z) of marker from camera
        self.pose = None  # Stored as (1,3) np array of x,y,z
        self.marker_found = False  # Updated each time a marker search is performed

        # Load the camera calibration parameters
        self.camera_mat, self.dist_coeffs = file_utils.load_yaml(os.path.abspath(CALIBRATION_FILE))
        self.focal_length = self.camera_mat[1][1] # Used to calculate the scale factor

        # Log various files for debugging
        # 0: No debugging
        # 1: Save a pose file
        # 2: Save pose and video files; displays the video on a laptop
        # 3: Save pose and video files and print the pose to console; displays the video on a laptop
        self.debug = int(debug)

        # Save video and pose files
        if self.debug > 0 :
            # Create pose file name and open it
            # Name can be accepted by itself or with ".txt" at the end
            if pose_file is None:
                self.pose_file = POSE_DIR + "/" + POSE_FILE
            elif not pose_file.endswith(".txt"):
                self.pose_file = POSE_DIR + "/" + pose_file + ".txt"
            else:
                self.pose_file = POSE_DIR + "/" + pose_file

            self.pose_file = file_utils.open_file(self.pose_file)

        if self.debug > 1:

            # A VideoWriter object saves a video visualizing the marker tracking steps
            self.video_writer = VideoWriter(video_dir=video_dir,
                                            video_file=video_file,
                                            resolution=resolution,
                                            framerate=fps_vid)

        self.start_time = time.time()

    ## --------Get Methods ---------- ##

    def get_true_alt(self):
        return self.true_alt

    def is_marker_found(self):
        return self.marker_found

    def get_marker_length(self):
        return self.marker_length

    def get_marker_type(self):
        return self.marker_type

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

    ## -------------------------- ##

    def set_pose(self, pose):
        self.pose = pose

    # Maintain a specific frequency of tracking
    # This is useful if your algorithm runs faster than necessary. If your algorithm run time is slower
    # than the desired frequency, the frequency is ignored and it runs at its max speed.
    def wait(self):
        try:
            time.sleep(self.period - (time.time() - self.cur_frame_time))
        except IOError:
            pass

    # Gracefully shut down the VideoStreamer and VideoWriter objects.
    def stop(self):
        self.vs.stop()
        if self.debug > 1:
            self.video_writer.stop()


    # This is where the marker tracking happens. Various image processing is performed to detect the marker.
    # If the marker is detected, its pose relative to the camera is estimated.
    # Returns True if marker is found
    # Else returns False
    @abstractmethod
    def track_marker(self, alt):
        pass


    # Calculate the pixel to meters conversion to obtain the marker length.
    @abstractmethod
    def calculate_scale_factor(self):
        pass


    # Debug method to call at the end of each tracking loop. Should implement multiple levels of debugging
    # 1: Save pose file
    # 2: Save pose and video files and display video if on a laptop
    # 3: Save pose and video files, display video if on a laptop, and print pose statements
    def debug_mode(self):

        if self.get_debug() > 0:
            self.save_pose()
        if self.get_debug() > 1:
            self.visualize()
        if self.get_debug() > 2:
            print("Marker Pose: {}".format(self.pose))


    # Used for debugging or recording purposes. Saves the pose to a .txt file.
    def save_pose(self):
        if self.pose is not None:
            self.pose_file.write("{} {} {} {} 0 0 0 0\n".format(time.time() - self.start_time,
                                                                self.pose[0],
                                                                self.pose[1],
                                                                self.pose[2]))


    # Used only in debug mode to visualize the image processing algorithms.
    @abstractmethod
    def visualize(self):
        pass


# --------------------------------- ColorMarkerTracker Class ---------------------------------------------- #

# This is an abstract base class for tracking colored markers. They can be of any shape and color.
# The child classes must define the marker detection and pose estimation functions specific to that marker.

# --------------------------------------------------------------------------------------------------------- #
class ColorMarkerTracker(MarkerTracker):
    def __init__(self,
                 src=SRC,
                 use_pi=USE_PI,
                 resolution=RESOLUTION,
                 framerate=FRAMERATE,
                 fps_vid=FPS_VID,
                 marker_width=YELLOW_MARKER_WIDTH,
                 marker_height=YELLOW_MARKER_HEIGHT,
                 freq=FREQ,
                 debug=DEBUG,
                 video_dir=VIDEO_DIR,
                 video_file=VIDEO_FILE,
                 pose_file=POSE_FILE):

        # Initialize parent class
        super(ColorMarkerTracker, self).__init__(src=src,
                                                 use_pi=use_pi,
                                                 resolution=resolution,
                                                 framerate=framerate,
                                                 fps_vid=fps_vid,
                                                 marker_width=marker_width,
                                                 marker_height=marker_height,
                                                 freq=freq,
                                                 debug=debug,
                                                 video_dir=video_dir,
                                                 video_file=video_file,
                                                 pose_file=pose_file)

        # Image processing frames
        self.undistort_frame = None # Undistorted image
        self.lab_space_frame = None # LAB Color Space
        self.hsv_frame = None # HSV image
        self.thresh_color_frame = None # Image threshold on color
        self.thresh_light_frame = None # Image threshold on lightness
        self.thresh_sum_frame = None # Sum of the image thresholds
        self.opening_frame = None # "Opening," defined w/in the track_marker method
        self.processed_frame = None  # Final processed frame

        # Below the altitude threshold, dynamic Otsu thresholding is performed on the lightness spectrum
        # in the LAB color space. Above the threshold, binary thresholding is performed. This was determined to be the
        # best approach for detecting the yellow marker in a large altitude range. The altitude threshold should be
        # empirically determined.
        self.alt_thresh = 20

        # Contour variables
        self.peri = None # Marker perimeter (pixels)
        self.marker_approx = None # Marker contour approximation

        # Pose estimation variables
        self.extLeft = None # Leftmost pixel of marker
        self.extRight = None # Rightmost pixel of marker
        self.extTop = None # Topmost pixel of marker
        self.extBot = None # Bottom-most pixel of marker
        self.cX = None # Marker horizontal distance from camera center
        self.cY = None # Marker vertical distance from camera center


    # Performs marker detection and pose estimation. The image processing and pose estimation is specific to each
    # marker, but the general algorithm is the same. First the image is thresholded on color and lightness to
    # remove everything except the marker from the image. Then the marker is found by looking for all shapes in
    # the threshold image and finding the one that most closely resembles the marker. If a shape is found, then
    # its pose is estimated.
    def track_marker(self, alt=25):

        # Perform image processing to extract the marker
        self.threshold_image()

        # Find all shapes within the threshold. A shape is any blob of pixels that form a closed body.
        shapes = self.find_shapes()

        # Continue if at least one shape is found
        if len(shapes) >= 1:

            # Extract the marker from all shapes
            self.find_marker(shapes)

            # Estimate the marker pose relative to the camera
            if self.marker_found:
                self.estimate_pose()

        self.debug_mode()

        return self.marker_found


    # Find all shapes within a thresholded image.
    def find_shapes(self):

        # Find contours
        # TODO: The return values here are dependent on Opencv 2,3,or 4, so either be sure of which version
        #  we are using or add some code to check the version
        _, contours, hierarchy = cv2.findContours(self.processed_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        return contours


    # Find the marker out of multiple detected shapes.
    def find_marker(self, shapes):

        # Take contour w/ max area; the marker will always be the largest contour in the image (for now)
        c = max(shapes, key=cv2.contourArea)

        # Approximate the contour
        self.peri = cv2.arcLength(c, True)
        self.marker_approx = cv2.approxPolyDP(c, 0.03 * self.peri, True)

        # Keep going if the contour is the marker
        self.marker_found = True if self.is_rectangle() else False

    # Check if the current potential marker is a rectangle. This is just an aspect ratio check, so make the
    # aspect ratio bounds less constraining if they filter out the marker, or more constraining if they let through
    # a lot of noise.
    def is_rectangle(self):
        if len(self.marker_approx) == 4:

            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(self.marker_approx)
            ar = w / float(h)

            # a rectangle will have an aspect ratio that is within this range
            return True if self.aspect_ratio[0] <= ar <= self.aspect_ratio[1] else False
        return False


    # Estimate the pose from the camera center to the marker center.
    def estimate_pose(self):
        c = self.marker_approx

        # Find extreme points
        self.extLeft = tuple(c[c[:, :, 0].argmin()][0])
        self.extRight = tuple(c[c[:, :, 0].argmax()][0])
        self.extTop = tuple(c[c[:, :, 1].argmin()][0])
        self.extBot = tuple(c[c[:, :, 1].argmax()][0])

        # Find the contour center
        M = cv2.moments(c)
        self.cX = int(M["m10"] / M["m00"])
        self.cY = int(M["m01"] / M["m00"])

        # Find positional error from center of platform; Pixels
        err_x = self.cX - (self.resolution[0] / 2)
        err_y = (self.resolution[1] / 2) - self.cY

        # Calculate the scale factor using the pixel marker length
        self.calculate_scale_factor()

        # Convert to meters
        err_x *= self.scale_factor
        err_y *= self.scale_factor

        # Update the pose estimate
        self.set_pose(np.array([err_x, err_y, self.depth]))


    # Calculate the scale factor that converts the marker in pixel length to the marker in meters.
    def calculate_scale_factor(self):

        # Find the distance to the marker
        # pixel_length = self.peri / 4
        (x, y, w, h) = cv2.boundingRect(self.marker_approx)
        pixel_length = np.amax((w, h))

        # Calculate the depth to the marker
        self.depth = (self.focal_length / pixel_length) * self.marker_length

        # Determine the *approximate* scale factor
        self.scale_factor = self.marker_length / pixel_length


    # Visualize all the steps of the marker detection algorithm. This will be saved to a video file if being run
    # on a laptop, otherwise it just saves the detected frame with marker pose information.
    def visualize(self):

        # Annotate image with marker pose information
        self.visualize_marker_pose()

        if self.use_pi:

            # Don't do heavy image processing if on the Pi, just save a video
            self.video_writer.write(self.detected_frame)
        else:

            # Reduce all 9 image dimensions so they can be merged together
            reduced_dim = (self.resolution[0] / 3, self.resolution[1] / 3)

            # Convert the mask images into three-channel images so they can be merged with the rest
            thresh_color_frame_three = cv2.merge(
                (self.thresh_color_frame, self.thresh_color_frame, self.thresh_color_frame))
            thresh_light_frame_three = cv2.merge(
                (self.thresh_light_frame, self.thresh_light_frame, self.thresh_light_frame))
            thresh_sum_frame_three = cv2.merge(
                (self.thresh_sum_frame, self.thresh_sum_frame, self.thresh_sum_frame))
            opening_frame_three = cv2.merge((self.opening_frame, self.opening_frame, self.opening_frame))
            processed_three = cv2.merge((self.processed_frame, self.processed_frame, self.processed_frame))

            # Resize
            img1 = cv2.resize(self.read(), reduced_dim, interpolation=cv2.INTER_AREA)
            img2 = cv2.resize(self.undistort_frame, reduced_dim, interpolation=cv2.INTER_AREA)

            if self.marker_type == "yellow":
                img3 = cv2.resize(self.lab_space_frame, reduced_dim, interpolation=cv2.INTER_AREA) # Yellow
            else:
                img3 = cv2.resize(self.hsv_frame, reduced_dim, interpolation=cv2.INTER_AREA) # Red

            img4 = cv2.resize(thresh_color_frame_three, reduced_dim, interpolation=cv2.INTER_AREA)
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
                            "Color Thresh, Light Thresh, Sum Thresh // "
                            "opening_frame, Closing, Final", cv2.WINDOW_FULLSCREEN)
            cv2.imshow("Original, Undistorted, Lab // "
                       "Color Thresh, Light Thresh, Sum Thresh // "
                       "opening_frame, Closing, Final", all_frames)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

            # Save video
            self.video_writer.write(all_frames)

    # Draw the marker pose onto the original image. This will be saved to a video file and displayed on a laptop
    # if the debug mode is >= 1.
    def visualize_marker_pose(self):
        self.detected_frame = self.cur_frame

        if self.marker_found:
            # Draw extreme points on image
            cv2.circle(self.detected_frame, self.extLeft, 8, (255, 0, 0), -1)
            cv2.circle(self.detected_frame, self.extRight, 8, (255, 0, 0), -1)
            cv2.circle(self.detected_frame, self.extTop, 8, (255, 0, 255), -1)
            cv2.circle(self.detected_frame, self.extBot, 8, (0, 0, 255), -1)

            # Draw center
            cv2.circle(self.detected_frame, (self.cX, self.cY), 7, (255, 255, 255), -1)
            cv2.putText(self.detected_frame, "center", (self.cX + 20, self.cY + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw depth
            cv2.putText(self.detected_frame, "Distance: {}".format(np.around(self.depth, 1)),
                        (self.cX - 40, self.cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw true altitude
            cv2.putText(self.detected_frame, "Altitude: {}".format(np.around(self.true_alt, 1)),
                        (self.cX - 40, self.cY),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw XY error
            cv2.putText(self.detected_frame,
                        "XY Error: {}, {}".format(np.around(self.pose[0], 1), np.around(self.pose[1], 1)),
                        (self.cX - 40, self.cY - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw contours on image
            cv2.drawContours(self.detected_frame, [self.marker_approx], 0, (0, 255, 0), 3)


    # Perform thresholding to extract the marker from the image. This function is dependent on the marker color.
    @abstractmethod
    def threshold_image(self):
        pass


# Tracks a red marker.
class RedMarkerTracker(ColorMarkerTracker):

    def __init__(self,
                 src=SRC,
                 use_pi=USE_PI,
                 resolution=RESOLUTION,
                 framerate=FRAMERATE,
                 fps_vid=FPS_VID,
                 marker_width=RED_MARKER_WIDTH,
                 marker_height=RED_MARKER_HEIGHT,
                 freq=FREQ,
                 debug=DEBUG,
                 video_dir=VIDEO_DIR,
                 video_file=VIDEO_FILE,
                 pose_file=POSE_FILE):

        # Initialize parent class
        super(RedMarkerTracker, self).__init__(src=src,
                                               use_pi=use_pi,
                                               resolution=resolution,
                                               framerate=framerate,
                                               fps_vid=fps_vid,
                                               marker_width=marker_width,
                                               marker_height=marker_height,
                                               freq=freq,
                                               debug=debug,
                                               video_dir=video_dir,
                                               video_file=video_file,
                                               pose_file=pose_file)

        self.marker_type = "red"
        self.lower_red_frame = None
        self.upper_red_frame = None

    # Threshold the image on red and lightness
    def threshold_image(self, alt=25):

        self.cur_frame_time = time.time()
        self.cur_frame = self.read()  # Grab the current camera frame
        self.true_alt = alt

        # Undistort image to get rid of fisheye distortion
        # TODO: Remove this undistortion?
        # TODO: Take video of takeoff, ascent, and travel to sample zone. Determine height needed to detect zone w/ and
        # TODO: w/o distortion, then make the decision
        # self.undistort_frame = cv2.undistort(self.cur_frame, self.camera_mat, self.dist_coeffs)
        self.undistort_frame = self.cur_frame

        # Convert to HSV
        self.hsv_frame = cv2.cvtColor(self.undistort_frame, cv2.COLOR_BGR2HSV)

        # Range for lower red HSV values
        self.lower_red_frame = cv2.inRange(self.hsv_frame, (0, 120, 70), (10, 255, 255))

        # Range for upper range HSV values
        self.upper_red_frame = cv2.inRange(self.hsv_frame, (170, 120, 70), (180, 255, 255))

        # Add together the two red thresholds
        self.thresh_color_frame = self.lower_red_frame + self.upper_red_frame

        # TODO: Determine if this actually helps in the field
        # Convert to LAB color space
        # LAB color space is used to separate lightness from color attributes
        # L is dark (0) to light (255)
        # A is Green (0) to Red (255)
        # B is Blue (0) to Yellow (255)
        self.lab_space_frame = cv2.cvtColor(self.cur_frame, cv2.COLOR_BGR2Lab)

        # TODO: Determine if this actually helps in the field
        # Lightness is thresholded since the yellow tarp will most likely be the lightest object in the image.
        # At lower altitudes, the marker takes up the majority of the image, and OTSU normalization is ideal, since
        # it dynamically chooses the threshold value to segment between "light" and "not light". At higher altitudes,
        # the marker is too small for OTSU to threshold well, so a binary threshold is used.
        retval2, self.thresh_light_frame = cv2.threshold(self.lab_space_frame[:, :, 0], 80, 255,
                                                         cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        # Generating the final mask to detect red color
        # self.thresh_sum_frame = cv2.bitwise_and(self.thresh_red_color_frame, self.thresh_light_frame)
        self.thresh_sum_frame = self.thresh_color_frame

        # Blur the image to reduce noise
        self.thresh_sum_frame = cv2.GaussianBlur(self.thresh_sum_frame, (5, 5), 0)

        # Create kernels for dilution and erosion operations; larger ksize means larger pixel neighborhood where the
        # operation is taking place
        se1 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(4, 4))
        se2 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(4, 4))
        # se1 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(8, 8))
        # se2 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(50, 50))

        # Perform "opening_frame." This is erosion followed by dilation, which reduces noise. The ksize is pretty small,
        # otherwise the white in the marker is eliminated
        self.opening_frame = cv2.morphologyEx(self.thresh_sum_frame, cv2.MORPH_OPEN, se1)

        # Perform "closing." This is dilution followed by erosion, which fills in black gaps within the marker. This is
        # necessary if the lightness threshold is not able to get the entire marker at lower altitudes
        self.processed_frame = cv2.morphologyEx(self.opening_frame, cv2.MORPH_CLOSE, se2)


# Tracks a yellow marker.
class YellowMarkerTracker(ColorMarkerTracker):

    def __init__(self,
                 src=SRC,
                 use_pi=USE_PI,
                 resolution=RESOLUTION,
                 framerate=FRAMERATE,
                 fps_vid=FPS_VID,
                 marker_width=YELLOW_MARKER_WIDTH,
                 marker_height=YELLOW_MARKER_HEIGHT,
                 freq=FREQ,
                 debug=DEBUG,
                 video_dir=VIDEO_DIR,
                 video_file=VIDEO_FILE,
                 pose_file=POSE_FILE):

        # Initialize parent class
        super(YellowMarkerTracker, self).__init__(src=src,
                                                  use_pi=use_pi,
                                                  resolution=resolution,
                                                  framerate=framerate,
                                                  fps_vid=fps_vid,
                                                  marker_width=marker_width,
                                                  marker_height=marker_height,
                                                  freq=freq,
                                                  debug=debug,
                                                  video_dir=video_dir,
                                                  video_file=video_file,
                                                  pose_file=pose_file)

        self.marker_type = "yellow"


    # This method tracks the color yellow. First, the image is undistorted to account for any lens distortion. Next,
    # it is converted from BGR to LAB color space. The LAB color space is great for color detection since it separates
    # the image into lightness (L), green -> red (*a), and blue -> yellow (*b). Various thresholds (explained within)
    # are applied to identify the yellow, and the thresholds are further refined to eliminate noise.
    def threshold_image(self, alt=25):

        self.cur_frame_time = time.time()
        self.cur_frame = self.read() # Grab the current camera frame
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

        ## Notes
        # Dynamic thresholding works terribly in brightness; maybe just get rid of it?
        # 127 yellow thresh lets in a lot but it's ok because binary light thresh filters it all out
        # Q: Does binary thresh work well in non-bright conditions?
        if np.abs(alt) < self.alt_thresh:
            # Dynamically threshold lightness
            retval2, self.thresh_light_frame = cv2.threshold(self.lab_space_frame[:, :, 0], 200, 255,
                                                             cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        else:
            # Binary threshold lightness
            retval2, self.thresh_light_frame = cv2.threshold(self.lab_space_frame[:, :, 0], 200, 255,
                                                             cv2.THRESH_BINARY)

        # Threshold yellow; Everything from 0 to 127 in the B space is made 0 - these are blueish colors
        retval1, self.thresh_color_frame = cv2.threshold(self.lab_space_frame[:, :, 2], 127, 255,
                                                          cv2.THRESH_BINARY)

        # Combine the yellow and lightness thresholds, letting through only the pixels that are white in each image
        self.thresh_sum_frame = cv2.bitwise_and(self.thresh_color_frame, self.thresh_light_frame)

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


# ----------------------------------------- ArucoTracker ------------------------------------------- #
# Tracks an Aruco marker. The main benefit of an Aruco marker is simple and already implemented
# marker detection and pose estimation functions. In addition, they can run much faster than most
# image processing algorithms.
# -------------------------------------------------------------------------------------------------- #
class ArucoTracker(MarkerTracker):

    def __init__(self,
                 src=SRC,
                 use_pi=USE_PI,
                 resolution=RESOLUTION,
                 framerate=FRAMERATE,
                 fps_vid=FPS_VID,
                 marker_width=ARUCO_MARKER_WIDTH,
                 marker_height=ARUCO_MARKER_HEIGHT,
                 freq=FREQ,
                 debug=DEBUG,
                 video_dir=VIDEO_DIR,
                 video_file=VIDEO_FILE,
                 pose_file=POSE_FILE):

        # Initialize parent class
        super(ArucoTracker, self).__init__(src=src,
                                           use_pi=use_pi,
                                           resolution=resolution,
                                           framerate=framerate,
                                           fps_vid=fps_vid,
                                           marker_width=marker_width,
                                           marker_height=marker_height,
                                           freq=freq,
                                           debug=debug,
                                           video_dir=video_dir,
                                           video_file=video_file,
                                           pose_file=pose_file)

        # Create aruco dictionary
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_param = aruco.DetectorParameters_create()  # Default parameters
        self.corners = None
        self.ids = None
        self.marker_corner = None
        self.marker_type = "aruco"

        self.rvec = None


    # The Aruco marker is tracked here. The pose is saved if the marker is found.
    # Returns True if found, else False.
    def track_marker(self, alt=0):

        # Perform image processing to extract the marker
        self.threshold_image()

        # Continue if at least one shape is found
        if self.ids is not None:
            self.marker_found = True

            # Estimate the marker pose relative to the camera
            self.estimate_pose()

        self.debug_mode()

    # Threshold the image to find the Aruco marker.
    def threshold_image(self):
        self.cur_frame_time = time.time()

        # Update current image
        self.cur_frame = self.read()

        # Detect the marker
        self.corners, self.ids, rejectedImgPoints = aruco.detectMarkers(self.cur_frame, self.aruco_dict,
                                                              parameters=self.aruco_param)


    # Estimate the marker pose relative to the camera center.
    def estimate_pose(self):
        self.marker_corner = self.corners[0]  # Corners are given clockwise starting at top left
        self.rvec, self.pose = aruco.estimatePoseSingleMarkers(self.marker_corner, self.marker_length,
                                                               self.camera_mat, self.dist_coeffs)

        # Reduce extra array dimensions
        self.rvec = self.rvec[0][0]
        self.pose = self.pose[0][0]


    # Visualize the Aruco marker detection. If on the Pi, only saves a video, since the GUI may not be available.
    def visualize(self):

        self.detected_frame = self.cur_frame

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

        # Draw the marker axis and pose on image if the marker was found
        if self.marker_found:

            aruco.drawDetectedMarkers(self.detected_frame, self.corners, self.ids)
            self.detected_frame = aruco.drawAxis(self.detected_frame, self.camera_mat, self.dist_coeffs,
                                                 self.rvec, self.pose, 1.5)

            str_position = "Marker Position (m) x=%2.2f y=%2.2f z=%2.2f" % (self.pose[0], self.pose[1], self.pose[2])
            cv2.putText(self.detected_frame, str_position, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0))

        # Display the image if running on a laptop
        if not self.use_pi:
            cv2.imshow("Aruco Tracker", self.detected_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

        # Save the frame to video
        self.video_writer.write(self.detected_frame)


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

    # Return the marker rotation
    def get_marker_rotation(self):
        return self.rvec
