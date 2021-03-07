"""
MicroPython implementation of SHT20 driver.

Tested on ESP32.

Montgomery Newcom <montgomery.newcom@gmail.com>

Reference material:
    https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Software/Sensirion_Humidity_Sensors_Software_SHT1x_SHT7x_Sample_Code.pdf
    https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Software/Sensirion_Humidity_Sensors_Software_SHT21_Sample_Code_C-file.zip
    https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Datasheets/Sensirion_Humidity_Sensors_SHT20_Datasheet.pdf
    http://www.embeddedadventures.com/datasheets/Sensirion_Humidity_SHT2x_CRC_Calculation_V1.pdf
"""

from time import sleep_ms
from machine import Pin, SoftI2C
from warnings import warn

class CRCError(Exception):
    'Data failed a CRC check.'
    pass

class NoSuchResolution(Exception):
    'User specified an invalid resolution.'

# SHT20 class to read temperature and humidity sensors
class sht20(object):
    'Driver for the sht20 temperature and humidity sensor.'
    DEVICE_ADDRESS = 0b100_0000

    TRIGGER_T_MEASUREMENT_HOLD = bytearray([0b1110_0011])
    TRIGGER_RH_MEASUREMENT_HOLD = bytearray([0b1110_0101])
    TRIGGER_T_MEASUREMENT_NO_HOLD = bytearray([0b1111_0011])
    TRIGGER_RH_MEASUREMENT_NO_HOLD = bytearray([0b1111_0101])
    
    WRITE_USER_REGISTER = bytearray([0b1110_0110])
    READ_USER_REGISTER = bytearray([0b1110_0111])
    SOFT_RESET = bytearray([0b1111_1110])
    
    RESOLUTION_T14_RH12 = bytearray([0b0000_0000])
    RESOLUTION_T12_RH8  = bytearray([0b0000_0001])
    RESOLUTION_T13_RH10 = bytearray([0b1000_0000])
    RESOLUTION_T11_RH11 = bytearray([0b1000_0001])
    
    END_OF_BATTERY = bytearray([0b0100_0000])
    ENABLE_HEATER = bytearray([0b0000_0100])
    DISABLE_OTP_RELOAD = bytearray([0b0000_0010])
    
    SERIAL_NUMBER_ADDRESS_A = bytearray([0b1111_1010, 0b0000_1111])
    SERIAL_NUMBER_ADDRESS_B = bytearray([0b1111_1100, 0b1100_1001])

    """
    The DHT20 datasheet lists widely varied response times, based upon the
    resolution of the measurement requested.  We'll try to not wait any longer
    than strictly necessary.
    
    These numbers may need tweaking, depending on your implementation.
    
    TempBits: { temp: ms, humi: ms, command: resolution_bytecode }
    """
    RES_VARS = {
            11: { 'tms': 11, 'rhms': 15, 'command': RESOLUTION_T11_RH11 },
            12: { 'tms': 22, 'rhms': 4,  'command': RESOLUTION_T12_RH8 },
            13: { 'tms': 43, 'rhms': 9,  'command': RESOLUTION_T13_RH10 },
            14: { 'tms': 85, 'rhms': 29, 'command': RESOLUTION_T14_RH12 }
        }
    I2C_WAIT_MS = 50
    POLYNOMIAL = 0x131 # P(x)=x^8+x^5+x^4+1 = 1_0011_0001

    # Read Serial Number:
    # https://www.sos.sk/a_info/resource/c/sensirion/Sensirion_Humidity_SHT2x_Electronic_Identification_Code_V1.1.pdf
    # https://github.com/HowardLin/Android_Native_Service_SHT20Demo/blob/master/SHT20_Module_1.2/sht20_driver/sht20.c


    def __init__(self, scl, sda, freq=400000, timeout=255, resolution=14):

        self.scl = Pin(scl)
        self.sda = Pin(sda)
        
        """
        Section 4, Table 5 specifies 0.4 MHz as the maximum frequency.
        """
        if (freq > 400000):
            warn("Setting the clock frequency above 400000 Hz is probably a bad idea.")
        
        self.freq = freq
        
        self.timeout = timeout
        
        if resolution not in self.RES_VARS:
            raise NoSuchResolution("Invalid resolution specified", resolution)
        self.resv = self.RES_VARS[resolution]
        
        """
        Section 5.1 - Start Up Sensor:
            After power-up, the sensor needs at most 15ms, while SCL is
            high, for reaching idle state, i.e. to be ready accepting
            commands from the master (MCU).
        """
        self.i2c = SoftI2C(scl=self.scl, sda=self.sda, freq=self.freq, timeout=self.timeout)
        sleep_ms(15)

        self.i2c.writeto(self.DEVICE_ADDRESS, self.SOFT_RESET)
        
        'Set resolution and DISABLE_OTP_RELOAD.'
        self.i2c.writeto(self.DEVICE_ADDRESS, self.WRITE_USER_REGISTER, self.resv['command'][0] | self.DISABLE_OTP_RELOAD[0])
        self.serial = self._get_device_info()

    @property
    def temperature(self):
        'Return the temperature as a Celcius float.'
        temperature = self._get_data_no_hold(
                self.TRIGGER_T_MEASUREMENT_NO_HOLD,
                self.resv['tms'])
        celcius = temperature * 175.72 / 65536 - 46.85
        return celcius

    @temperature.setter
    def temperature(self, value):
        raise AttributeError("Can't set attribute.")
        
    @property
    def relative_humidity(self):
        'Return the relative humidity as a percentage float.'
        relative_humidity = self._get_data_no_hold(
                self.TRIGGER_RH_MEASUREMENT_NO_HOLD,
                self.resv['rhms'])
        relative_humidity = relative_humidity * 125 / 65536 - 6
        return relative_humidity

    @relative_humidity.setter
    def relative_humidity(self, value):
        raise AttributeError("Can't set attribute.")
    
    def soft_reset(self):
        'Reset the sensor.'
        self.i2c.writeto(self.DEVICE_ADDRESS, self.SOFT_RESET)
        """
        Section 5.5 - Soft Reset
            The soft reset takes less than 15ms.
        """
        sleep_ms(15)

    def _get_data_no_hold(self, command, ms=85):
        'Request data from sensor, wait a moment, read data from the sensor, and verify it with CRC-8 check.'
        data = bytearray(3)
        self.i2c.writeto(self.DEVICE_ADDRESS, command)
        sleep_ms(ms)

        self.i2c.readfrom_into(self.DEVICE_ADDRESS, data)
        value = self._convert_to_integer(data[:2])

        if not self._verify_checksum(data):
            raise CRCError('Data read off I2C bus, but failed CRC-8 check.', data[:2], data[-1])
        return value

    def _get_device_info(self):
        'Get the serial number.'
        'https://www.sos.sk/a_info/resource/c/sensirion/Sensirion_Humidity_SHT2x_Electronic_Identification_Code_V1.1.pdf'

        # Serial Number part A
        self.i2c.writeto(self.DEVICE_ADDRESS, self.SERIAL_NUMBER_ADDRESS_A)
        sna = bytearray(8)
        sleep_ms(self.I2C_WAIT_MS)
        self.i2c.readfrom_into(self.DEVICE_ADDRESS, sna)

        sleep_ms(self.I2C_WAIT_MS)

        # Serial Number part B
        self.i2c.writeto(self.DEVICE_ADDRESS, self.SERIAL_NUMBER_ADDRESS_B)
        snb = bytearray(6)
        sleep_ms(self.I2C_WAIT_MS)
        self.i2c.readfrom_into(self.DEVICE_ADDRESS, snb)

        combined_id = bytearray([
                snb[3], sna[4], sna[0], sna[2],
                sna[4], sna[6], snb[0], snb[1]])
        serial = self._convert_to_integer(combined_id)

        return serial

    def _convert_to_integer(self, byte_array):
        'Use bitwise operators to convert the bytes into integers.'
        integer = None
        for chunk in byte_array:
            if not integer:
                integer = chunk
            else:
                integer = integer << 8
                integer = integer | chunk
        return integer

    def _verify_checksum(self, data):
        crc = 0
        values = data[:2]
        checksum = int(data[-1])
        for value in values:
            crc = crc ^ value
            for _ in range(8, 0, -1):
                if crc & 0x80:
                    crc = (crc << 1) ^ self.POLYNOMIAL
                else:
                    crc <<= 1
        return crc == checksum
