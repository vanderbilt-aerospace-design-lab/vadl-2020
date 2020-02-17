
import dronekit

class Controller:

    def __init__(self, vehicle):

        # Vehicle connection to read pixhawk state from
        self.vehicle = vehicle

        # Basic controller state
        self.ch1 = 0
        self.ch2 = 0
        self.ch3 = 0
        self.ch4 = 0
        self.ch5 = 0
        self.ch6 = 0
        self.ch7 = 0
        self.ch8 = 0

        # Automatically track various modes
        self.winch_mode = 0
        self.autonomy_mode = 0

        # Add callback for updates to RC_CHANNELS
        self.vehicle.add_attribute_listener("RC_CHANNELS", self.refresh)

    def refresh(self, name, message):

        self.ch1 = message.chan1_raw
        self.ch2 = message.chan2_raw
        self.ch3 = message.chan3_raw
        self.ch4 = message.chan4_raw
        self.ch5 = message.chan5_raw
        self.ch6 = message.chan6_raw
        self.ch7 = message.chan7_raw
        self.ch8 = message.chan8_raw

    def print_channels(self):

        print("Channel 1:\t" + str(self.ch1))
        print("Channel 2:\t" + str(self.ch2))
        print("Channel 3:\t" + str(self.ch3))
        print("Channel 4:\t" + str(self.ch4))
        print("Channel 5:\t" + str(self.ch5))
        print("Channel 6:\t" + str(self.ch6))
        print("Channel 7:\t" + str(self.ch7))
        print("Channel 8:\t" + str(self.ch8))
