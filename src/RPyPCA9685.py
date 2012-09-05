'''
Control up to 16 LEDs or PWM servos using I2C and a PCA9685 IC

This module contains a class representing the PCA9685 IC as used
on the Adafruit 16-Channel 12-bit PWM/Servo Driver I2C interface [1]_
and a few minor helper functions:

:author: TimJay@github
:date: 2012-07-22
:license: Creative Commons BY-NC-SA 3.0 [2]_

[1] http://www.adafruit.com/products/815
[2] http://creativecommons.org/licenses/by-nc-sa/3.0/deed.en_US
'''
from quick2wire.i2c import I2CMaster, reading, writing_bytes
import time


def testBit(int_type, offset):
    '''
    Check if the bit `offset` is set in `int_type`.

    :param int_type: The value in which the bit is to be checked.
    :param offset: The offset for the bit to be checked, starting at 0.
    '''
    mask = 1 << offset
    return(int_type & mask)


def _setBit(int_type, offset):
    '''
    Set the bit `offset` in `int_type`.

    :param int_type: The value in which the bit is to be set.
    :param offset: The offset for the bit to be checked, starting at 0.
    '''
    mask = 1 << offset
    return(int_type | mask)


def _clearBit(int_type, offset):
    '''
    Clear the bit `offset` in `int_type`.

    :param int_type: The value in which the bit is to be cleared.
    :param offset: The offset for the bit to be checked, starting at 0.
    '''
    mask = ~(1 << offset)
    return(int_type & mask)


def getByte(int_type, nth_byte):
    '''
    Get the `nth_byte` of `int_type`.

    :param int_type: The value from which the selected byte will be read.
    :param nth_byte: Which byte to read, starting from 0.
    '''
    return (int_type >> (8 * nth_byte)) & 0xff


class PCA9685:
    '''
    Control the PCA9685 IC as used on the Adafruit 16-Channel 12-bit
    PWM/Servo Driver I2C interface [1]_
    '''

    _PRESCALE = 121  # = round(25MHz / (4096 * update_rate) - 1) [121 ~ 50Hz]

    _PULSE_START = 100

    _PULSE_END_LOW = 305

    _PULSE_END_HIGH = 509

    def REGISTERS(self):
        '''
        Provides a dictionary with the registers of the PCA9685
        '''
        return {"mode1": 0x00,
                "mode2": 0x01,
                "subadr1": 0x02,
                "subadr2": 0x03,
                "subadr3": 0x04,
                "allcalladr": 0x05,
                "all_on_low": 0xfa,
                "all_on_high": 0xfb,
                "all_off_low": 0xfc,
                "all_off_high": 0xfd,
                "pre_scale": 0xfe,
                "testmode": 0xff}

    def MODE1_BITS(self):
        '''
        Provides a dictionary with the bit offsets for mode1 register
        '''
        return {"allcall": 0,
                "sub3": 1,
                "sub2": 2,
                "sub1": 3,
                "sleep": 4,
                "ai": 5,
                "extclk": 6,
                "restart": 7}

    def MODE2_BITS(self):
        '''
        Provides a dictionary with the bit offsets for mode2 register
        '''
        return {"oe0": 0,
                "oe1": 1,
                "outdrv": 2,
                "och": 3,
                "invrt": 4}

    def _valid_servo(self, servo_id):
        '''
        Check if the provided servo_id is valid.
        :param servo_id: Must be in the range [0 .. 15]
        '''
        if not (servo_id >= 0 and servo_id <= 15):
            raise Exception("servo_id must be in the range [0 .. 15]")

    def SERVO_ON_LOW(self, servo_id):
        '''
        Register to set the lower start byte of the pulse of servo_id
        :param servo_id: Must be in the range [0 .. 15]
        '''
        self._valid_servo(servo_id)
        return (0x06 + 4 * servo_id)

    def SERVO_ON_HIGH(self, servo_id):
        '''
        Register to set the higher start byte of the pulse of servo_id
        :param servo_id: Must be in the range [0 .. 15]
        '''
        self._valid_servo(servo_id)
        return (0x07 + 4 * servo_id)

    def SERVO_OFF_LOW(self, servo_id):
        '''
        Register to set the lower stop byte of the pulse of servo_id
        :param servo_id: Must be in the range [0 .. 15]
        '''
        self._valid_servo(servo_id)
        return (0x08 + 4 * servo_id)

    def SERVO_OFF_HIGH(self, servo_id):
        '''
        Register to set the higher stop byte of the pulse of servo_id
        :param servo_id: Must be in the range [0 .. 15]
        '''
        self._valid_servo(servo_id)
        return (0x09 + 4 * servo_id)

    def set_position(self, servo_id, position):
        '''
        Set the position of servo_id taking offset and overscan into account.
        :param servo_id: Must be in the range [0 .. 15]
        :param position: The position is usually in the range [1000 .. 2000]
        '''
        self._valid_servo(servo_id)
        if not (position > 0 and position < 3000):
            raise Exception("position must be in the range [0 .. 3000]")
        value = int((position - 1000) / (2000 - 1000) * (self._PULSE_END_HIGH - self._PULSE_END_LOW) + self._PULSE_END_LOW)
        self.master.transaction(writing_bytes(self.ADDRESS, self.SERVO_OFF_LOW(servo_id), getByte(value, 0)), writing_bytes(self.ADDRESS, self.SERVO_OFF_HIGH(servo_id), getByte(value, 1)))

    def __init__(self, controller, address):
        '''
        Instantiates a master fom quick2wire i2c,
        sets the prescale value to _PRESCALE,
        disables the allcall address,
        initialises all channels to start at 100
        and end at (_PULSE_END_HIGH + _PULSE_END_LOW) / 2.0,
        and activates the PCA9685 by clearing the sleep bit
        '''
        self.master = I2CMaster(controller)
        self.ADDRESS = address
        self._setRegister(self.REGISTERS().get("pre_scale"), self._PRESCALE)
        self._clearBit(self.REGISTERS().get("mode1"), self.MODE1_BITS().get("allcall"))
        self._setRegister(self.REGISTERS().get("all_on_low"), getByte(self._PULSE_START, 0))
        self._setRegister(self.REGISTERS().get("all_on_high"), getByte(self._PULSE_START, 1))
        center = int((self._PULSE_END_HIGH + self._PULSE_END_LOW) / 2.0)
        self._setRegister(self.REGISTERS().get("all_off_low"), getByte(center, 0))
        self._setRegister(self.REGISTERS().get("all_off_high"), getByte(center, 1))
        self._clearBit(self.REGISTERS().get("mode1"), self.MODE1_BITS().get("sleep"))
        time.sleep(0.1)
        self._setBit(self.REGISTERS().get("mode1"), self.MODE1_BITS().get("restart"))
        time.sleep(0.1)

    def __del__(self):
        '''
        Deactivate the PCA9685 by setting the sleep bit
        and frees the master from quick2wire i2c
        '''
        self._setBit(self.REGISTERS().get("mode1"), self.MODE1_BITS().get("sleep"))
        self.master.close()

    def _setRegister(self, register, value):
        self.master.transaction(writing_bytes(self.ADDRESS, register, getByte(value, 0)))

    def _getRegister(self, register):
        return self.master.transaction(writing_bytes(self.ADDRESS, register), reading(self.ADDRESS, 1))[0][0]

    def _getBit(self, register, bit):
        b = self._getRegister(register)
        return testBit(b, bit)

    def _setBit(self, register, bit):
        b = self._getRegister(register)
        self._setRegister(register, _setBit(b, bit))

    def _clearBit(self, register, bit):
        b = self._getRegister(register)
        self._setRegister(register, _clearBit(b, bit))
