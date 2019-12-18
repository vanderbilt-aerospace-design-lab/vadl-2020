import RPi.GPIO as GPIO
import time

# Configure pin numbering scheme we want to use
GPIO.setmode(GPIO.BCM)

# Set pin as an output
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(26, GPIO.IN)

# Set pin to high/low
GPIO.output(16, GPIO.HIGH)
GPIO.output(20, GPIO.HIGH)
GPIO.output(19, GPIO.HIGH)

time.sleep(25)

# Reset all pins to default states
GPIO.output(16, GPIO.LOW)
GPIO.output(20, GPIO.LOW)
GPIO.output(19, GPIO.LOW)
GPIO.cleanup()
