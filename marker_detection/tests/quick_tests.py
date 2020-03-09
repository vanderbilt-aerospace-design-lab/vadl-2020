from utils import dronekit_utils
from marker_detection import marker_tracker

marker_tracker = marker_tracker.RedMarkerTracker(src=0,
                                                    debug=0,
                                                    marker_width=3.14,
                                                    marker_height=3.0)

while True:
    marker_tracker.track_marker(alt=0)
    if marker_tracker.is_marker_found():
        print(marker_tracker.get_pose())
