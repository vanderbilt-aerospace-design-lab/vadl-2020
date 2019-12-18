
# ----------------------- #
# --- IMPORT PACKAGES --- #
# ----------------------- #

import board
import busio
import digitalio
import pulseio
import time
import adafruit_rfm69

# ------------------------ #
# --- INITIALIZE RADIO --- #
# ------------------------ #

# Radio frequency
RadioFreq = 915.0

# Radio module pins
RAD_CS = digitalio.DigitalInOut(board.RFM69_CS)
RAD_RST = digitalio.DigitalInOut(board.RFM69_RST)
RAD_SPI = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm69 = adafruit_rfm69.RFM69(RAD_SPI, RAD_CS, RAD_RST, RadioFreq, baudrate=9600)

# ------------------------------- #
# --- INITIALIZE MOTOR DRIVER --- #
# ------------------------------- #

# Connect pins
MOT_PWM = pulseio.PWMOut(board.D5)
MOT_DIR_1 = digitalio.DigitalInOut(board.D9)
MOT_DIR_2 = digitalio.DigitalInOut(board.D6)
MOT_STDBY = digitalio.DigitalInOut(board.D10)

# Set pins as outputs
MOT_DIR_1.direction = digitalio.Direction.OUTPUT
MOT_DIR_2.direction = digitalio.Direction.OUTPUT
MOT_STDBY.direction = digitalio.Direction.OUTPUT

# Enable motor, set to forward
MOT_STDBY.value = True
MOT_DIR_1.value = 0
MOT_DIR_2.value = 1
MOT_PWM.duty_cycle = 0

# ------------------------------- #
# --- INITIALIZE COMS WITH PI --- #
# ------------------------------- #

# Set pins as digital IO
PI_IN_1 = digitalio.DigitalInOut(board.A1)
PI_IN_2 = digitalio.DigitalInOut(board.A2)
PI_IN_3 = digitalio.DigitalInOut(board.A3)
PI_OUT = digitalio.DigitalInOut(board.A4)

# Set directions
PI_IN_1.direction = digitalio.Direction.INPUT
PI_IN_2.direction = digitalio.Direction.INPUT
PI_IN_3.direction = digitalio.Direction.INPUT
PI_OUT.direction = digitalio.Direction.OUTPUT

# ----------------------- #
# --- STATE VARIABLES --- #
# ----------------------- #

current_op = "IDLE"
output_val = False
auto_timer = 0.0

# Flags
flag_lower = False
flag_raise = False
flag_complete = False
flag_scoop_complete = False
flag_deposit_complete = False

# Cutoff vals
lower_cutoff = 5.0
raise_cutoff = 5.0

# ------------------------ #
# --- HELPER FUNCTIONS --- #
# ------------------------ #

def set_motor(direction, speed_pct):

    global MOTOR_DIR_1
    global MOTOR_DIR_2
    global MOTOR_PWM

    # Direction
    if direction == "forward":
        MOT_DIR_1 = 0
        MOT_DIR_2 = 1
    elif direction == "backward":
        MOT_DIR_1 = 1
        MOT_DIR_2 = 0
    elif direction == "brake":
        MOT_DIR_1 = 1
        MOT_DIR_2 = 1

    # Speed
    MOT_PWM.duty_cycle = int(speed_pct * (2**16 - 1))


def reset_flags(scoop=True, deposit=True):

    # Declare globals
    global flag_raise
    global flag_lower
    global flag_complete
    global flag_scoop_complete
    global flag_deposit_complete

    flag_lower = False
    flag_raise = False
    flag_complete = False
    flag_scoop_complete = False if scoop else flag_scoop_complete
    flag_deposit_complete = False if deposit else flag_deposit_complete


def do_IDLE():
    
    # Declare globals
    global current_op

    current_op = "IDLE"
    reset_flags()
    set_motor("brake", 0.0)


def do_LOWER_MANUAL():

    # Declare globals
    global current_op

    current_op = "LOWER_MANUAL"
    reset_flags()
    set_motor("forward", 0.75)


def do_RAISE_MANUAL():

    # Declare globals
    global current_op

    current_op = "RAISE_MANUAL"
    reset_flags()
    set_motor("backward", 0.75)


def do_LOWER_AUTO():

    # Declare globals
    global current_op
    global flag_lower
    global flag_complete
    global auto_timer
    global output_val

    current_op = "LOWER_AUTO"

    # If not already lowering, reset flag
    if not flag_lower:
        reset_flags()
        flag_lower = True
        flag_complete = False
        set_motor("forward", 0.75)
        auto_timer = time.monotonic()

    # Check if it's time to shut off motor
    if time.monotonic() - auto_timer > lower_cutoff and not flag_complete:
        set_motor("brake", 0.0)
        flag_complete = True
        output_val = not output_val


def do_RAISE_AUTO():

    # Declare globals
    global current_op
    global flag_raise
    global flag_complete
    global auto_timer
    global output_val

    current_op = "RAISE_AUTO"

    # If not already lowering, reset flag
    if not flag_raise:
        reset_flags()
        flag_raise = True
        flag_complete = False
        set_motor("backward", 0.75)
        auto_timer = time.monotonic()

    # Check if it's time to shut off motor
    if time.monotonic() - auto_timer > raise_cutoff and not flag_complete:
        set_motor("brake", 0.0)
        flag_complete = True
        output_val = not output_val


def do_SCOOP():

    # Declare globals
    global current_op
    global flag_scoop_complete
    global output_val

    current_op = "SCOOP"
    reset_flags(scoop=False)  # Don't reset the scoop_complete flag
    set_motor("brake", 0.0)

    # Send scoop command until acknowledgement received
    while not flag_scoop_complete:

        # Send message
        rfm69.send('SCOOP')

        # Check for response
        response = rfm69.receive(timeout=0.2)
        response = str(response, 'ascii') if response is not None else None

        if response is not None and response == "SCOOP_COMPLETE":
            flag_scoop_complete = True
            output_val = not output_val


def do_DEPOSIT():

    # Declare globals
    global current_op
    global flag_deposit_complete
    global output_val

    current_op = "DEPOSIT"
    reset_flags(deposit=False) # Don't reset the deposit_complete flag
    set_motor("brake", 0.0)

    # Send deposit command until acknowledgement received
    while not flag_deposit_complete:

        # Send message
        rfm69.send('DEPOSIT')

        # Check for response
        response = rfm69.receive(timeout=0.2)
        response = str(response, 'ascii') if response is not None else None

        if response is not None and response == "DEPOSIT_COMPLETE":
            flag_deposit_complete = True
            output_val = not output_val


def do_ERROR():

    # Declare globals
    global current_op

    current_op = "ERROR"
    reset_flags()
    set_motor("brake", 0.0)


# ------------------------- #
# --- MAIN CONTROL LOOP --- #
# ------------------------- #

while True:

    # Read each input pin
    in_1 = int(PI_IN_1.value)
    in_2 = int(PI_IN_2.value)
    in_3 = int(PI_IN_3.value)

    # Concat into 3-bit string for readability
    input_str = str(in_1) + str(in_2) + str(in_3)

    ### DO ACTION FOR THIS MODE ###

    # IDLE mode
    if input_str == "000":
        do_IDLE()

    # LOWER MANUAL mode
    elif input_str == "001":
        do_LOWER_MANUAL()

    # RAISE MANUAL mode
    elif input_str == "010":
        do_RAISE_MANUAL()

    # Automatic lower mode
    elif input_str == "011":
        do_LOWER_AUTO()

    # Automatic raise mode
    elif input_str == "100":
        do_RAISE_AUTO()

    # SCOOP mode
    elif input_str == "101":
        do_SCOOP()

    elif input_str == "110":
        do_DEPOSIT()
   
    # ERROR mode
    else:
        do_ERROR()

    # Print debug information
    debug_str = input_str
    debug_str += "\t" + str(current_op)
    debug_str += "\t" + str(output_val)
    debug_str += "\t" + str(flag_lower)
    debug_str += "\t" + str(flag_raise)
    debug_str += "\t" + str(flag_complete)
    debug_str += "\t" + str(flag_scoop_complete)
    debug_str += "\t" + str(flag_deposit_complete)
    debug_str += "\t" + str(time.monotonic())
    print(debug_str)

    # Set output pin
    PI_OUT.value = output_val

    # Loop at 10Hz
    time.sleep(0.1)