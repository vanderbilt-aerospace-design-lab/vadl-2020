# ---------------------------------------------------------------------------------- #
#
# This script executes the simplest version of the SLI launch-day mission sequence
# - All navigation is done using GPS positioning
# - GPS positions of the sample zones are hard coded before launch
# - Landing and sampling are each assumed to be done manually
#
# ---------------------------------------------------------------------------------- #

from utils.dronekit_utils import *
from utils.controller_utils import *

if __name__ == "__main__":

    # Connect to flight controller
    vehicle = connect_vehicle()

    # Connect to RC controller
    rc_controller = Controller(vehicle)
    rc_controller.print_channels()

    vehicle.close()
    # Establish connection with Feather M0 for winch and tool control


    # Wait for 'Go' command from controller


    # Choose next sample zone to fly to


    # Takeoff


    # Fly to sample zone


    # Switch to Loiter


    # Wait for 'Go' command from controller


    # Return to launch


    # Switch to Loiter

