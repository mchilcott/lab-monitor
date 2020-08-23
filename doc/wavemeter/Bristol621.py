import serial
import struct


"""A poor library that is capable of reading wavelength and power from the Bristol 621 wavelength meter.
It disregards sequence numbers, and just uses a few fixed control strings. Seems to work fine though.

CAUTION: This library contains code that is untested, and unused. However, the Wavemeter class should work as expected.
"""

# Format messages[<message id>] = (
#    <name>,
#    <struct format string>,
#    (<struct member 1 name>, <struct member 2 name>...)
# )

# < = little endian
# b = char (B = unsigned)
# h = short (H = unsigned)
# i = int (I = unsigned)
# l = long (L = unsigned)
# f = float
# d = double

tsMeasurementDataType = "<4I4f2Hfd", ("ScanIndex", "Status", "RefFringeCount", "InpFringeCnt", "StartPhase", "EndPhase", "Temperature", "Pressure", "RefPower1", "RefPower2", "InputPower", "Wavelength")
tsScanDataType = "<4I768s", ("NumEntries", "mnReferenceBlockCount","nmReferenceBlockSize", "Filler", "(uint)maFringeData[32][6]")
tsUnitInfoTypeA = "<3I5f8s8si2f4B4fI3f2I128s", ("mnZopLeft", "mnZopRight", "mnSerialNumber", "mnCorrectionWavelength1", "mnCorrectionCoefficient_1", "mnCorrectionWavelength2", "mnCorrectionCoefficient_2", "mnPowerCalibrationCoeff", "maFirmwareVersion", "maSoftwareVersion", "mnMaxRefIntensity", "mnTemperatureOffset", "mnPressureOffset", "mnUnitModel", "meLaserType", "mnRefGainSetting", "mnUnusedC2", "mnSelfCalCoef", "mnSelfCalTemp", "mnPowerCalibrationOffset", "mnSelfCalTempCoef", "mnInpGainThreshA", "mnInpGainThreshB", "mnInpGainThreshC", "mnUnused7", "mnUnused8", "(float)maDetectorResponsivity[16][2]")
tsUnitInfoTypeB = "<2fI4f2H8s8si2f4B4fI3f2I128s128s", ("mnEtlAThick", "mnEtlBThick", "mnSerialNumber", "mnCorrectionWavelength1", "mnCorrectionCoefficient_1", "mnCorrectionWavelength2", "mnCorrectionCoefficient_2", "mnCalLocA", "mnCalLocB", "maFirmwareVersion", "maSoftwareVersion", "mnMaxRefIntensity", "mnTemperatureOffset", "mnPressureOffset", "mnUnitModel", "meLaserType", "mnRefGainSetting", "mnUnusedC2", "mnSelfCalCoef", "mnSelfCalTemp", "mnPowerCalibrationOffset", "mnSelfCalTempCoef", "mnInpGainThreshA", "mnInpGainThreshB", "mnInpGainThreshC", "mnUnused7", "mnUnused8", "(float)maDetectorResponsivity[16][2]" "(float)maDetectorCalCurves[16][2]")

"""
struct tsHeartBeatType
{
    unsigned int            mnScanIndex;
    unsigned int            mnStatus;
    tsMeasurementDataType   msMeasData;
    tsScanDataType          msScanData;
    double                  maWaveLengths[MAX_K];
};
"""

auto_send_off = 0
auto_send_unused = 1
auto_send_measurement = 2
auto_send_scan = 3

lambda_unit_nm = 0
lambda_unit_GHz = 1
lambda_unit_cm1 = 2

power_unit_mW = 0
power_unit_dB = 1

medium_vacuum = 0
medium_air = 1

acquire_mode_measure = 0
acquire_mode_calibration = 1
acquire_mode_align = 2


messages = {
    0x22: "Power Units?",
    0x23: "Wavelength Units?",
    0x25: "Wavelength Medium?",
    0x28: "Set Auto Send",
    0x30: "Read-back Measurement",
    0x40: "Request Power",
    0x45: "Readback Lambda 1",
    0x60: "Measurement Data Response",
    0x61: "Unit Info Response",
    0x68: "Power Read",
    0x69: "Read ADC Response",
    0x6A: "Scan Data Response",
    0x6B: "Lambda Response",
    0x6C: "Lambda 2 Reponse",
    0x6E: "Measurement Heartbeat",
    0x6F: "Head best message"
        }

def lookup_message(msg_id):
    try:
        return messages[msg_id]
    except KeyError:
        return "Unknown Message"


# Sequence numbers are 1, but this doesn't seem to matter if you reuse it, or skip, etc
request_wavelength = b"\x7E\x45\x01\x45\x01\x00\x00\x00\x00"
# Note that wavelength is in wavenumbers by default - Payload is a float

request_power = b"\x7E\x40\x01\x40\x01\x00\x00\x00\x00"
# No idea what return format is for this.

request_measurement = b"\x7E\x30\x01\x30\x01\x00\x00\x00\x00"
# Return format for this also unknown

# These ones work
set_wavelength_nm  = b"\x7E\x23\x01\x22\x01\x01\x00\x00\x00" + struct.pack("b",lambda_unit_nm) +b"\x00"
set_wavelength_ghz = b"\x7E\x23\x01\x23\x01\x01\x00\x00\x00" + struct.pack("b",lambda_unit_GHz) +b"\x00"
set_wavelength_cm1 = b"\x7E\x23\x01\x20\x01\x01\x00\x00\x00" + struct.pack("b",lambda_unit_cm1) +b"\x00"

# Not really verified to work
set_medium_vacuum = b"\x7E\x25\x01\x24\x01\x01\x00\x00\x00" + struct.pack("b",medium_vacuum) +b"\x00"
set_medium_air    = b"\x7E\x25\x01\x25\x01\x01\x00\x00\x00" + struct.pack("b",medium_air) +b"\x00"



class packet_io():
        
    start_token = b'\x7e'
    escape_token = b'\x7d'
    escape_mask = b'\x20'
    
    #
    # Message packet
    # 00   Start Token      7E
    # 01   Message ID       nn
    # 02   Sequence Number  01 - FF
    # 3-4  Checksum         0000-FFFF
    # 5-6  Payload Length   0000-FFFF
    # 7-8  Padding          xxxx
    
    def __init__(self, io):
        self.conn = io
        self.read_seq = 0
        self.write_seq = 1
        
    def unescaped_read(self):
        """Read a byte from the serial interface, removing any escaping if necessary.
        """
        byte = self.conn.read()
        
        if byte == self.start_token:
            print("UNEXPECTED MESSAGE START TOKEN")
        
        if byte == self.escape_token:
            byte = bytes([self.conn.read()[0] ^ self.escape_mask[0]])
        
        return byte

    def escape(self, byte):
        """Escape a single byte - not yet implemented / used
        """
        assert len(byte) == 1
        if byte == self.start_token or byte == self.escape_token:
            byte = self.escape_token + bytes([byte[0] ^ self.escape_mask[0]])
        return byte
            
            
        
    def read_packet(self):
        """Read a packet from the instrument. This will wait indefinitely.
        Returns a tuple in form (message_id, data, sequence_number, length, checksum).
        """
        # Wait for start character
        while(self.conn.read() != self.start_token): pass

        # Start receiving packet
        message_id = struct.unpack('B', self.unescaped_read())[0]
        
        sequence_number =  struct.unpack('B', self.unescaped_read())[0]
        
        #if sequence_number == self.read_seq:
        #    print("Error: Sequence number appeared twice")
            
        #if sequence_number != (self.read_seq % 255) + 1:
        #    print("Error: Sequence number delta not 1: " + str(sequence_number))
        
        self.read_seq = sequence_number
        
        checksum = struct.unpack('<H', self.unescaped_read() + self.unescaped_read())[0]
        
        length = struct.unpack('<H', self.unescaped_read() + self.unescaped_read())[0]
        
        # Discard padding - need it if implementing checksum
        padding = self.unescaped_read() + self.unescaped_read()
        
        data = b''
        
        for i in range (0, length) :
            data += (self.unescaped_read())
        
        return (message_id, data, sequence_number, length, checksum)
    
    def make_packet(self, message_id, payload):
        """Assemble a packet - Not yet implemented/completed
        """
        output = bytearray()
        output += self.start_token

        checkum = (message_id << 8) + self.write_seq 
        
        
        data = bytearray()

        data += message_id
        
    def handle_packet(self, pkt):
        """Handle converting a packet into the data we would actually want
        """
        
        # Measurement Data Response
        if(pkt[0] == 0x60):
            data = struct.unpack_from(tsMeasurementDataType[0], pkt[1], 8)
            for i in range(len(data)):
                print(tsMeasurementDataType[1][i] + ": " + str(data[i]))
            return data
        
        # Power. Not sure about format
        if(pkt[0] == 0x68):
            power = struct.unpack("<fh", pkt[1])[0]
            return power
        
        
        # Wavelength
        if(pkt[0] == 0x6B):
            wavelength = struct.unpack("<d", pkt[1])[0]
            #print("Wavelength: " + str(wavelength))
            return wavelength
        
        print("Message Unhandled...")
        print(lookup_message(pkt[0]))
        
        return pkt

class Wavemeter:
    
    def __init__ (self, port = '/dev/ttyUSB0'):
        self.con = serial.Serial(port, 921600, timeout=1)
        self.pio = packet_io(self.con)

    def set_wavelength_units(self, units='nm'):
        """Set the units of the wavelength reported. Valid options are 'nm', 'GHz', and 'cm-1' (wavenumbers in per cm).
        These are not case sensitive.        
        """
        units = units.lower()
        control_map = {
            'nm': set_wavelength_nm,
            'ghz': set_wavelength_ghz,
            'cm-1': set_wavelength_cm1,
        }
        if units not in control_map.keys():
            raise ValueError("Units must be one of "+str(control_map.keys()))
        
        self.con.write(control_map[units])
        
    def set_medium(self, med='vacuum'):
        """Set the medium for the reported wavelength. Can be set to report wavelength in 'vacuum' or 'air'.
        """
        med = med.lower()
        control_map = {
            'vacuum': set_medium_vacuum,
            'air': set_medium_air
        }
        if med not in control_map.keys():
            raise ValueError("Medium must be one of "+str(control_map.keys()))
        
        self.con.write(control_map[med])

    def get_power(self):
        """Get the current power measurement from the Wavemeter. This is reported in milliwatts (mW).
        """
        # Ask for power
        self.con.write(request_power)
        pkt1 = self.pio.read_packet()
        return self.pio.handle_packet(pkt1)

    def get_wavelength(self):
        """Get the current wavelength measurement from the wavemeter.
        This is reported in the units selected by ~set_wavelength_units~
        """
        # Ask for wavelength
        self.con.write(request_wavelength)
        pkt2 = self.pio.read_packet()
        return self.pio.handle_packet(pkt2)

    def close(self):
        self.con.close()
        delattr(self, 'con')

    def __del__(self):
        """Closes the connection to the instrument."""
        # We only want to close the serial port if it was successfully opened in
        # the first place
        if hasattr(self, 'con'):
                self.con.close()


if __name__ == '__main__':
    # A small example/test of the driver
    
    w = Wavemeter('/dev/ttyUSB0')
    w.set_wavelength_units('nm')
    w.set_medium('vacuum')
    
    print("Wavelength %f nm-vac" % w.get_wavelength())
    print("Power %f mW" % w.get_power())
