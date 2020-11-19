#!/usr/bin/env python
import rospy
from picopter.msg import Motors_msg

class PCA9685:
    from time import sleep
    from smbus import SMBus

    _MODE1 = 0x00
    _LED0_ON_L = 0x06
    _LED0_ON_H = 0x07
    _LED0_OFF_L = 0x08
    _LED0_OFF_H = 0x09
    _ALL_LED_ON_L = 0xFA
    _ALL_LED_ON_H = 0xFB
    _ALL_LED_OFF_L = 0xFC
    _ALL_LED_OFF_H = 0xFD
    _PRE_SCALE = 0xFE


    def __init__(self, i2c_address=0x40, bus_num=1):
        """
        :param int i2c_address: The I2C address of the PCA9685 (default is 0x40).
        :param int bus_num: The SMBus instance of the I2C port (0, for the the Raspberry Pi 1, and 1, for all future models).
        :return None
        """
        if bus_num != 0 and bus_num != 1:
            raise ValueError("The I2C port must be 0 or 1.")

        self._i2c_address = i2c_address
        self._bus = self.SMBus(bus_num)
        self._frequency = 400

        self._write_byte(self._MODE1, 0x00)

        # Sets the on and off positions of all channels to 0: (there may be a better way to do this using some inbuilt function of the PCA9685 - you'd have to check the data sheet)
        self._write_byte(self._ALL_LED_ON_L, 0)
        self._write_byte(self._ALL_LED_ON_H, 0)
        self._write_byte(self._ALL_LED_OFF_L, 0)
        self._write_byte(self._ALL_LED_OFF_H, 0)

        self.start()

    def ros_callback(self, data):
        cmd_1 = (data.M1 * 9.30) + 1070
        cmd_2 = (data.M2 * 9.30) + 1070
        cmd_3 = (data.M3 * 9.30) + 1070
        cmd_4 = (data.M4 * 9.30) + 1070
        
        self.set_pwm(15, cmd_1)
        self.set_pwm(0, cmd_2)
        self.set_pwm(4, cmd_3)
        self.set_pwm(11, cmd_4)


    def start(self):
        self.set_frequency(int(400))
        for i in range(16):
            self.set_pwm(int(i), int(1000))
        self.sub = rospy.Subscriber("motor_cmds", Motors_msg, self.ros_callback)

    def set_frequency(self, frequency):
        """
        Sets the frequency of all channels.
        :param int frequency: The frequency in Hz (24 - 1526).
        :return None
        """
        if not 24 <= frequency <= 1526:
            raise ValueError("Frequency is out of range.")

        prescale_value = int(round(25000000.0 / (4096.0 * frequency) - 1.0))  # this equation is found in the data sheet page 25
        

        current_mode = self._read_byte(self._MODE1)
        if current_mode is None:  # means there was an IOError when reading
            return

        new_mode = (current_mode & 0x7F) | 0x10
        self._write_byte(self._MODE1, new_mode)
        self._write_byte(self._PRE_SCALE, prescale_value)
        self._write_byte(self._MODE1, current_mode)
        self.sleep(0.0005)  # delay 500 microseconds
        self._write_byte(self._MODE1, current_mode | 0x80)

        self._frequency = frequency

        for i in range(16):
            self.set_pwm(int(i), int(1000))


    def set_duty_cycle(self, channel_num, duty_cycle):
        """
        Sets the duty cycle of a specified channel.
        :param int channel_num: The specified channel number (0 - 15).
        :param duty_cycle: The duty cycle in % (0 - 100)
        :return None
        """
        off = round(duty_cycle * 4095.0 / 100.0)  # maps 0-100 to 0-4095
        self._write_channel(channel_num, off)


    def set_pwm(self, channel_num, pulse_width):
        """
        Sets the pulse width of a specified channel.
        :param int channel_num:  The specified channel number (0 - 15).
        :param pulse_width: The pulse width in microseconds.
        :return None
        """
        # 1 / frequency = seconds per cycle
        #   * 1000000.0 = microseconds per cycle
        #   / 4096.0 = microseconds per tick
        # pulse_width / microseconds per tick = off
        off = int(round(pulse_width / (1000000.0 / (self._frequency * 4096.0))))
        if off < 4096:
            self._write_channel(int(channel_num), off)

    def calibrate(self):
        for i in range(16):
            self.set_pwm(i, 2000)

        #print("Plug in Battery.")
        self.sleep(5)  # delay 1 second
        #print("Setting Low Throttle in 1 second.")
        self.sleep(1)
        for i in range(16):
            self.set_pwm(i, 1000)
        #print("Calibration Done.")

    def _write_channel(self, channel_num, off):
        # if you want to write the starting "on" tick:
            # self._write_byte(self._LED0_ON_L + 4 * channel_num, on & 0xFF)  # writes the lower 8 bits of on
            # self._write_byte(self._LED0_ON_H + 4 * channel_num, on >> 8)    # writes the upper 4 bits of on

        # Each LED address is separated by 4 which is why we add a multiple of 4 to the LED0 address
        self._write_byte(self._LED0_OFF_L + 4 * channel_num, off & 0xFF)  # writes the lower 8 bits of off
        self._write_byte(self._LED0_OFF_H + 4 * channel_num, off >> 8)    # writes the upper 4 bits of off


    def _read_byte(self, location):
        try:
            return self._bus.read_byte_data(self._i2c_address, location)
        except IOError:
            #print(f"IOError when reading from the bus at {hex(self._i2c_address)}.")
            return None

    def _write_byte(self, location, value):
        try:
            self._bus.write_byte_data(self._i2c_address, location, value)
        except IOError:
            print "IOError when writing to the bus."
            #print(f"IOError when writing to the bus at {hex(self._i2c_address)}.")

if __name__ == '__main__':
    i2c_address = 0x40
    i2c_port_num = 1
    motordriver = PCA9685(i2c_address, i2c_port_num)
    rospy.init_node('motor_driver', anonymous=True)
    rospy.spin()