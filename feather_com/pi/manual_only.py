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

def reset_feather():

	GPIO.output(21, 0)
	time.sleep(0.1)
	GPIO.output(21, 1)

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
	elif cmd == "OPEN":
		set_active_cmd_num(1, 0, 1)
	elif cmd == "CLOSE":
		set_active_cmd_num(1, 1, 0)


# ----------------- #
# --- MAIN CODE --- #
# ----------------- #

### INITIAL PIN SETUP ### 

# Configure pin numbering scheme to BCM
GPIO.setmode(GPIO.BCM)

# Set pins as input/output
GPIO.setup(16, GPIO.OUT)  # com
GPIO.setup(20, GPIO.OUT)  # com
GPIO.setup(19, GPIO.OUT)  # com
GPIO.setup(26, GPIO.IN)   # com
GPIO.setup(21, GPIO.OUT)  # feather reset

# Set pins to initial states
GPIO.output(16, GPIO.LOW)
GPIO.output(20, GPIO.LOW)
GPIO.output(19, GPIO.LOW)
GPIO.output(21, GPIO.HIGH)

### VEHICLE CONNECTION ###

# Connect to vehicle
vehicle = dronekit.connect('/dev/ttyAMA0', wait_ready=True, baud=921600)
print("MAVLink connection established")
print("Battery: ") + str(vehicle.battery.voltage) + (" V")

# Add a listener for RC channel 8 
channel_8_val = 1500
channel_7_val = 990

@vehicle.on_message("RC_CHANNELS")
def ch8_listener(self, name, message):

	global channel_8_val
	global channel_7_val
	channel_8_val = message.chan8_raw
	channel_7_val = message.chan7_raw

### CONTROL CODE ###

reset_feather()

# Keep track of heartbeat
last_heartbeat = vehicle.last_heartbeat
scoop_state = "OPEN"

# Stop when no heartbeat in 5 seconds
while vehicle.last_heartbeat - last_heartbeat < 5:

	# Update last_heartbeat
	last_heartbeat = vehicle.last_heartbeat

	# Check if scoop_switch has switched changed
	if scoop_state == "OPEN" and channel_7_val > 1500:
		scoop_state = "CLOSED"
		set_active_cmd_str("CLOSE")
	elif scoop_state == "CLOSED" and channel_7_val <= 1500:
		scoop_state = "OPEN"
		set_active_cmd_str("OPEN")

	# Check value of channel 8 and do corresponding action

	elif channel_8_val < 1000:
		set_active_cmd_str("RAISE_MANUAL")
	elif channel_8_val > 2000:
		set_active_cmd_str("LOWER_MANUAL")
	else:
		set_active_cmd_str("IDLE")
	
	debug_str = "V: " + str(vehicle.battery.voltage)
	debug_str +=  "\tA: " + str(vehicle.battery.current)
	debug_str += "\tP: " + str(vehicle.battery.voltage * vehicle.battery.current)
	print(debug_str)
	time.sleep(0.1)

### SHUTDOWN OPS ###

print("Vehicle disconnected, shutting down...")

# Reset all pins to default states
GPIO.output(16, GPIO.LOW)
GPIO.output(20, GPIO.LOW)
GPIO.output(19, GPIO.LOW)
GPIO.cleanup()

# Close MAVLink connection
vehicle.close()
