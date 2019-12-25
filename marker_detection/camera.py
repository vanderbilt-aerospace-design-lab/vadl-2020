import time
from imutils.video import VideoStream
from imutils.video import FileVideoStream
import cv2
import datetime
import time

# specify as relative or absolute
VIDEO_DIR = "marker_detection/videos"

''' Camera Class

    This high-level class retains camera parameters, such as resolution and frame rate. It also keeps track of 
    if this camera is being used on a Raspberry Pi, since the Pi requires different code for retrieving images.
'''
class Camera(object):
    def __init__(self,
                 use_pi=-1,
                 resolution=(640, 480),
                 framerate=30):
        self.resolution = resolution
        self.framerate = framerate
        self.use_rpi = 1 if use_pi > 0 else 0


    def get_resolution(self):
        return self.resolution

    def get_framerate(self):
        return self.framerate


''' VideoStreamer Class

    This class is used to stream video from an arbitrary camera source. Once the source is defined, all the methods 
    are the same no matter what camera or computer you are using. It utilizes the VideoStream class, created by 
    Adrian Rosebrock and well documented at these links:
    
    https://www.pyimagesearch.com/2016/01/04/unifying-picamera-and-cv2-videocapture-into-a-single-class-with-opencv/
    https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/
    https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/
    
    In short, it performs all camera I/O in a separate thread, which reduces I/O latency. It also standardizes methods
    between a USB cam and a Picam.
    
    src: Used only for cv2.VideoCapture(src). 0 means 1st camera, 1 is 2nd camera, etc... If you are on a laptop, then
         0 is the webcam, while 1 may be a camera plugged in by USB. On the Pi, 0 would be the Picam. src can also be a
         video file name, passed in as a string.
    use_pi: Make this > 0 if you are running this on the Pi.    
    resolution: Camera resolution
    framerate: Camera frame rate
'''
class VideoStreamer(Camera):
    def __init__(self,
                 src=0,
                 use_pi=-1,
                 resolution=(640, 480),
                 framerate=30):

        super(VideoStreamer, self).__init__(use_pi=use_pi,
                                            resolution=resolution,
                                            framerate=framerate)

        # VideoStream class is used for live stream.
        # FileVideoStream class is used for streaming from a saved video.
        if isinstance(src, int):
            self.vs = VideoStream(src=src,
                                  usePiCamera=use_pi > 0,
                                  resolution=resolution,
                                  framerate=framerate).start()
        else:
            self.vs = FileVideoStream(path=src)

        # Let camera warm up
        time.sleep(1.0)

    # Returns the frame as a np array
    def read(self):
        if isinstance(self.vs, FileVideoStream):
            return self.vs.stream.read()[1]
        else:
            return self.vs.read()

    # Terminate the capture thread.
    def stop(self):
        self.vs.stop()

''' VideoWriter Class

    This class is used to save videos. It will automatically name your file using the current date and time if you 
    do not specify a name. Default encoding is .avi, since this has been proven to work best with cv2's funky video 
    writing methods.
'''
class VideoWriter(Camera):
    def __init__(self,
                 video_dir=VIDEO_DIR,
                 video_file=None,
                 ext=".avi",
                 resolution=(640, 480),
                 framerate=30):

        super(VideoWriter, self).__init__(resolution=resolution,
                                          framerate=framerate)

        self.ext = ext
        self.video_file = video_file
        self.video_dir = video_dir
        if self.video_file is None:
            self.video_file = self.create_file_name()
        if self.video_dir is None:
            self.video_dir = VIDEO_DIR

        # Codec encoding. You shouldn't have to mess with this, and it is highly recommended you don't!
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.writer = cv2.VideoWriter(self.video_dir + "/" + self.video_file + self.ext, self.fourcc, framerate, resolution, True)

        self.frame_start = 0

    # Generates a file name based off the current date and time.
    def create_file_name(self):
        date = datetime.datetime.now()
        return str(date.year) + "_" + str(date.month) + "_" \
               + str(date.day) + "_" + str(date.hour) + "_" + str(date.minute)

    # Write a single frame to the video output.
    def write(self, frame):
        self.frame_start = time.time()
        self.writer.write(frame)
        self.wait()

    # Waits the appropriate amount of time to maintain the proper FPS.
    def wait(self):
        wait_time = (1.0 / self.framerate) - time.time() + self.frame_start
        if wait_time > 0:
            time.sleep(wait_time)

    # Release the output when done (rarely needed - video is not corrupted if you use Ctrl-C or turn off your computer)
    def release(self):
        self.writer.release()
