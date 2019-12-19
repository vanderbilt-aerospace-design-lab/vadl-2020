# -------------------------------- #
# --- IMPORT REQUIRED PACKAGES --- #
# -------------------------------- #

import RPi.GPIO as GPIO
import time
import dronekit

# ------------------------ #
# --- HELPER FUNCTIONS --- #
# ------------------------ #

def set_active_cmd_num(out_1, out_2, out_3):

	GPIO.output(16, out_1)
	GPIO.output(20, out_2)
	GPIO.output(19, out_3)


def set_active_cmd_str(cmd):

	if cmd == "IDLE":
		set_active_cmd_num(0, 0, 0)
	elif cmd == "LOWER_MANUAL":
		set_active_cmd_num(0, 0, 1)
	elif cmd == "RAISE_MANUAL":
		set_active_cmd_num(0, 1, 0)
	elif cmd == "LOWER_AUTO":
		set_active_cmd_num(0, 1, 1)
	elif cmd == "RAISE_AUTO":
		set_active_cmd_num(1, 0, 0)
	elif cmd == "SCOOP":
		set_active_cmd_num(1, 0, 1)
	elif cmd == "DEPOSIT":
		set_active_cmd_num(1, 1, 0)


# ----------------- #
# --- MAIN CODE --- #
# ----------------- #

### INITIAL PIN SETUP ### 

# Configure pin numbering scheme to BCM
GPIO.setmode(GPIO.BCM)

# Set pin as an output
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(26, GPIO.IN)

# Set pin to high/low
GPIO.output(16, GPIO.LOW)
GPIO.output(20, GPIO.LOW)
GPIO.output(19, GPIO.LOW)

### VEHICLE CONNECTION ###

# Connect to vehicle
vehicle = dronekit.connect('dev/ttyAMA0', wait_ready=True, baud=921600)
print("MAVLink connection established")

# Add a listener for RC channel 8 
channel_8_val = 1500

@vehicle.on_message("RC_CHANNELS")
def ch8_listener(self, name, message):
	channel_8_val = message.chan8_raw

### CONTROL CODE ###

# Keep track of heartbeat
last_heartbeat = vehicle.last_heartbeat

# Stop when no heartbeat in 5 seconds
while vehicle.last_heartbeat - last_heartbeat < 5:

	# Update last_heartbeat
	last_heartbeat = vehicle.last_heartbeat

	# Check value of channel 8 and do corresponding action
	if channel_8_val < 1000:
		set_active_cmd_str("RAISE_MANUAL")
	elif channel_8_val > 2000:
		set_active_cmd_str("LOWER_MANUAL")
	else:
		set_active_cmd_str("IDLE")


### SHUTDOWN OPS ###

print("Vehicle disconnected, shutting down...")

# Reset all pins to default states
GPIO.output(16, GPIO.LOW)
GPIO.output(20, GPIO.LOW)
GPIO.output(19, GPIO.LOW)
GPIO.cleanup()

# Close MAVLink connection
vehicle.close()
