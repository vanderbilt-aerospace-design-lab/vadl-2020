
import dronekit

class Controller:

    def __init__(self, vehicle):

        # Vehicle connection to read pixhawk state from
        self.vehicle = vehicle

        # Automatically track various modes
        self.connected = False
        self.winch_mode = "IDLE"
	self.scoop_mode = "OPEN"
        self.autonomy_mode = 0

    	self.vehicle.add_attribute_listener("channels", self.refresh)

    # TODO add logic for autonommy modes, automatic winch control 
    def refresh(self, vehicle_obj, attr_name, values):

	# Update winch mode
	if values['8'] < 1000:
	    self.winch_mode = "RAISE_MANUAL"
	elif values['8'] > 2000:
	    self.winch_mode = "LOWER_MANUAL"
	else:
	    self.winch_mode = "IDLE"

	# Update scoop mode
	if self.scoop_mode == "OPEN" and values['7'] > 1500:
	    self.scoop_mode = "CLOSED"
	elif self.scoop_mode == "CLOSED" and values['7'] <= 1500:
	    self.scoop_mode = "OPEN"

	# Update autonomy mode

    def print_controller_states(self):
	print("Winch:\t" + self.winch_mode + "\tScoop:\t" + str(self.scoop_mode))



