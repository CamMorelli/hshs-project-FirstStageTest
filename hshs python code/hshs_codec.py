"""Module for encoding and decoding data from MCU. Encoding is only for testing because encoding is normally handled by MCU"""
import copy
import numpy as np

NUM_CHANNELS = 4

class SerialFormatsNew:
    """Defines format elements for serial port data from MCU"""
    def __init__(self, value, n_bytes, offset, scaling):
        self.value = value
        self.n_bytes = n_bytes
        self.offset = offset
        self.scaling = scaling


class TextClass:
    """Defines sample for MCU text responses, which are not a fixed length"""
    def __init__(self):
        self.data_list = ['text']      #   This defines the order of data elements
        self.flag_code = 0xFD
        self.text =                 SerialFormatsNew(0, 1,  0, 1)
        self.size = None
class FitDataClass:
    """Defines sample for fit data package"""
    def __init__(self):
        self.data_list = ['transition_type', 'first_point', 'num_points', 'slopes', 'intercepts', 'debug_data', 'offset']      #   This defines the order of data elements
        self.flag_code = 0xFC
        self.type = 0   #   This is calculated from transition_type
        self.transition_type =      SerialFormatsNew(0,                     2,  0,          1)
        self.first_point =          SerialFormatsNew(0,                     4,  0,          1)
        self.num_points =           SerialFormatsNew(0,                     2,  0,          1)
        self.slopes =               SerialFormatsNew(0,   4,  110000000, 10000)    #   Array size is set here
        self.intercepts =           SerialFormatsNew(0,   4,  110000000, 10000)    #   Array size is set here
        self.debug_data =  SerialFormatsNew(0,   4,  0,          1000)    #   Array size is set here
        self.offset =               SerialFormatsNew(0,                     2,  0,          100)
        self.size = None

class HSHSDataClass:
    """Defines sample for crisp data package"""
    def __init__(self, num_channels):
        self.data_list = ['time', 'state_time', 'count', 'state_count', 'state', 'pd_1_value', 'pd_2_value', 'pd_diff_meas', 'pd_diff_set_point', \
                          'error_value', 'average_error', 'servo_value', 'crisp_led_value']      #   This defines the order of data elements
        self.flag_code = 0xFE
        self.time =                 SerialFormatsNew(0,                         4,  0,  1000)
        self.state_time =           SerialFormatsNew(0,                         4,  0,  1000)
        self.count =                SerialFormatsNew(0,                         4,  0,  1)
        self.state_count =          SerialFormatsNew(0,                         4,  0,  1)
        self.state =                SerialFormatsNew(0,                         2,  0,  1)
        self.pd_1_value =           SerialFormatsNew(0,                         2,  0,  10000)
        self.pd_2_value =           SerialFormatsNew(0,                         2,  0,  10000)
        self.pd_diff_meas =         SerialFormatsNew(0,                         4,  110000000,  100000)    #   Range -1000 to 1000
        self.pd_diff_set_point =    SerialFormatsNew(0,                         4,  110000000,  100000)    #   Range -1000 to 1000

        self.error_value =          SerialFormatsNew(0,                         4,  110000000,  50000)    #   Range -1000 to 1000
        self.average_error =         SerialFormatsNew(0,                         4,  110000000,  50000)    #   Range -1000 to 1000

        self.servo_value =          SerialFormatsNew(0,                         4,  0,  10000)  # Current range 0 to 1000
        self.crisp_led_value =      SerialFormatsNew(0,                         2,  0,  100)
        self.size = None

class HSHSDataClassNew:
    """Defines sample for crisp data package"""
    def __init__(self, num_channels):
        self.data_list = ['time', 'state_time', 'count', 'state_count', 'state', 'laser_pwm', 'tec_pwm', 'fan_pwm', 'laser_temperatures', 'temperature_set_points', \
                          'laser_current', 'tec_current']      #   This defines the order of data elements

        self.flag_code = 0xFE
        self.time =                 SerialFormatsNew(0,                         4,  0,  1000)
        self.state_time =           SerialFormatsNew(0,                         4,  0,  1000)
        self.count =                SerialFormatsNew(0,                         4,  0,  1)
        self.state_count =          SerialFormatsNew(0,                         4,  0,  1)
        self.state =                SerialFormatsNew(0,                         2,  0,  1)

        self.laser_pwm =            SerialFormatsNew(np.zeros(num_channels),    2,  0,  150)    #   Array size is set here
        self.tec_pwm =              SerialFormatsNew(np.zeros(num_channels),    2,  0,  150)    #   Array size is set here
        self.fan_pwm =              SerialFormatsNew(0,                         2,  0,  150)
        self.laser_temperatures =   SerialFormatsNew(np.zeros(num_channels),    2,  0,  150)    #   Array size is set here
        self.temperature_set_points =   SerialFormatsNew(np.zeros(num_channels),    2,  0,  150)    #   Array size is set here
        self.laser_current =        SerialFormatsNew(0,                         2,  0,  1500)
        self.tec_current =          SerialFormatsNew(0,                         2,  0,  1500)

        self.size = None


class Codec:
    """Attributes and methods for decoding MCU data as well as encoding MCU data for testing"""
    def __init__(self, num_channels):
        self.bytes_received = []
        self.pending_flag = -1
        self.expected_data_bytes = -1
        self.hex_obj_lookup = {}
        self.hex_size_lookup = {}
        self.crisp_data = HSHSDataClassNew(num_channels)
        self.fit_data = FitDataClass()
        self.text_data = TextClass()
        self.data_obj_list = [self.crisp_data, self.fit_data, self.text_data]
        for obj in self.data_obj_list:
            self.fill_object_from_loop(obj, 1, 2)
            self.calc_data_size(obj)
            self.hex_obj_lookup[hex(obj.flag_code)]=obj  #   hex() creates a hex string from an integer
            self.hex_size_lookup[hex(obj.flag_code)]=obj.size

    def get_frame(self, byte) -> tuple[bytes, bool]:
        """Handles single bytes from process_byte_stream and finds packages based on separation codes/flags with bit 8 set; returns complete frame and frame_detected boolean"""
        frame_detected = False
        frame_data = bytes([])
        if byte > 127:      #   Byte values > 127 reserved for delimiter/flag codes
            if self.pending_flag > 0:   #   Process prior frame if pending_flag is valid
                expected_data_bytes = self.hex_size_lookup[hex(self.pending_flag)] + 1 #   Add one extra byte because hex flag is part of frame
                if len(self.bytes_received) == expected_data_bytes or self.pending_flag == 0xFD:    #   Check for valid data chunk length prior to data flag or text type
                    frame_detected = True
                    frame_data = bytes(self.bytes_received)
                else:
                    print (f"WARNING: Invalid data package length for flag: {hex(self.pending_flag)} expected: {expected_data_bytes} received: {len(self.bytes_received)}")
            if hex(byte) in self.hex_size_lookup:     #   Check for new valid encoding flag
                self.pending_flag = byte
                self.bytes_received = [byte]  #   Place flag in first byte of frame. This allows testing encoding/decoding without buffering/get_frame
            else:                           #   Not valid encoding flag
                self.pending_flag = -1
                print (f"WARNING: Unrecognized data flag: {hex(byte)}")
                self.bytes_received = []    #   Reset payload buffer
        else:   #   Bytes values <= 127 are for data
            if self.pending_flag > 0:
                self.bytes_received.append(byte)
        return frame_data, frame_detected

    def pack_element(self, in_value, num_bytes) -> (bytes):
        """Pack data element from 8bits/byte to 7 bits/byte. This is only for simulation and testing as the 7-bit packing is performed by the MCU"""
        out_value = bytes()
        if in_value > 1 << (num_bytes * 7):
            print("WARNING: Warning, ", in_value, " > available bits ", num_bytes*7, "allow maximum of ", 1<<(num_bytes*7))
        for index in range(num_bytes - 1, -1, -1):
            next_value = (in_value >> (index * 7)) & 0x7F
            byte_val = bytes([next_value])  # An alternative is byte_val = struct.pack('B', next_value)
            out_value += byte_val
        return out_value

    def unpack_element(self, bytes_in, read_index, num_bytes) -> tuple[int, int]:
        """Unpack data element from 7 bits/byte (coming from MCU) to 8 bits/byte for handling in python"""
        new_value = 0
        for count, index in enumerate(range(num_bytes - 1, -1, -1)):
            new_value += (bytes_in[read_index+count] & 0x7F) << (index * 7)
        read_index += num_bytes
        return new_value, read_index

    def calc_data_size(self, obj):
        """Find data package size in bytes based on information in data class"""
        size = 0
        for key in obj.data_list:
            next_attribute = getattr(obj, key)
            if isinstance(next_attribute.value, (list, tuple, np.ndarray)):
                size += len(next_attribute.value)*next_attribute.n_bytes
            else:
                size += next_attribute.n_bytes
        obj.size = size

    def fill_object_from_loop(self, obj, offset, scaling) -> int:
        """Fill object containing data and format. This is used for testing only as data packages are produced by the MCU"""
        index = 0
        for key in obj.data_list:
            next_attribute = getattr(obj, key)
            if isinstance(next_attribute.value, (list, tuple, np.ndarray)):
                for index2 in range (0, len(next_attribute.value)):
                    next_attribute.value[index2] = index*scaling + offset
                    index += 1
            else:
                if obj.flag_code == 0xFD:
                    next_attribute.value = "test"
                else:
                    next_attribute.value = index*scaling + offset
                index += 1

    def fill_object_from_buffer(self, obj, offset, scaling):
        """Fill object containing data and format from existing buffer. This is used for testing only as data packages are produced by the MCU"""
        index = 0
        for key in obj.data_list:
            next_attribute = getattr(obj, key)
            if isinstance(next_attribute.value, (list, tuple, np.ndarray)):
                for index2 in range (0, len(next_attribute.value)):
                    next_attribute.value[index2] = index*scaling + offset
                    index += 1
            else:
                next_attribute.value = index*scaling + offset
                index += 1

    def dump_codec_obj(self, obj):
        """Prints contents of Class definition of serial port buffer"""
        for key in obj.data_list:
            next_attribute = getattr(obj, key)
            if isinstance(next_attribute.value, (list, tuple, np.ndarray)):
                print (key, [next_attribute.value[index] for index in range(0, len(next_attribute.value))])
            else:
                print (key, next_attribute.value)

    def compare_codec_obj(self, obj1, obj2):
        """Compares contents of two objects for Class definitions of serial port buffer"""
        diff_count = 0
        for key in obj1.data_list:
            next_attribute1 = getattr(obj1, key)
            next_attribute2 = getattr(obj2, key)
            if isinstance(next_attribute1.value, (list, tuple, np.ndarray)):
                for index in range(0, len(next_attribute1.value)):
                    if next_attribute1.value[index] != next_attribute2.value[index]:
                        diff_count += 1
                        print(f"WARNING: Values differ for attribute {key}, index {index}: First value: {next_attribute1.value[index]} Second value: \
                              {next_attribute2.value[index]}")
            else:
                if next_attribute1.value != next_attribute2.value:
                    print(f"WARNING: Values differ for attribute {key}: First value: {next_attribute1.value} Second value: {next_attribute2.value}")
                    diff_count += 1
        print(f"{diff_count} differences for {hex(obj1.flag_code)}")

    def encode_frame(self, obj) -> bytes:
        """Encodes complete buffer using Class definition of format; this is for testing as encoding is normally on MCU"""
        out_buffer = bytes([obj.flag_code])
        for key in obj.data_list:
            next_attribute = getattr(obj, key)
            offset_val = next_attribute.offset
            scaling_val = next_attribute.scaling
            num_bytes = next_attribute.n_bytes
            if isinstance(next_attribute.value, (list, tuple, np.ndarray)):
                array_size = len(next_attribute.value)
                for index in range(0, array_size):
                    next_value = next_attribute.value[index]
                    out_buffer += self.pack_element(int(next_value*scaling_val + offset_val), num_bytes)
            else:
                if obj.flag_code == 0xFD:    #   Incorporate entire text buffer
                    next_value = next_attribute.value
                    out_buffer += bytes(next_value, 'utf-8')
                else:
                    next_value = next_attribute.value
                    out_buffer += self.pack_element(int(next_value*scaling_val + offset_val), num_bytes)
        return out_buffer

    def decode_frame(self, buffer: bytes) -> any:
        """Decodes complete frame using Class definition of format; byte package validation is performed before reaching here. Returns appropriate new copy of object"""
        pending_flag = buffer[0]
        obj = copy.deepcopy(self.hex_obj_lookup[hex(pending_flag)])     #   Unless copy is made, all changes will be made to single original object
        if pending_flag == 0xFD:    #   Text data
            obj.text.value = str(buffer[1:],'UTF-8').replace('\r','').strip('\n')
        else:   #   crisp data or fit data
            read_index = 1
            for key in obj.data_list:
                next_attribute = getattr(obj, key)
                offset_val = next_attribute.offset
                scaling_val = next_attribute.scaling
                num_bytes = next_attribute.n_bytes
                if isinstance(next_attribute.value, (list, tuple, np.ndarray)):
                    array_size = len(next_attribute.value)
                    for index in range(0, array_size):
                        (unpacked_value, read_index) = self.unpack_element(buffer, read_index, num_bytes)
                        if scaling_val == 1:     #   Allow integer
                            next_attribute.value[index] = unpacked_value - offset_val
                        else:
                            next_attribute.value[index] = (unpacked_value - offset_val)/scaling_val
                else:
                    (unpacked_value, read_index) = self.unpack_element(buffer, read_index, num_bytes)
                    if scaling_val == 1:     #   Allow integer
                        next_attribute.value = unpacked_value - offset_val
                    else:
                        next_attribute.value = (unpacked_value - offset_val)/scaling_val
        return obj

    def process_byte_stream(self, buffer):
        """Process byte stream coming through serial port from MCU"""
        if len(buffer) > 0:
            for byte in buffer:
                (frame_data, frame_detected) = self.get_frame(byte)
                if frame_detected:
                    obj = self.decode_frame(frame_data)
                    self.compare_codec_obj(self.hex_obj_lookup[hex(frame_data[0])], obj)

def test():
    """Test codec function"""
    print("\n//////////////////////////////")
    print("Do basic test for single data class......")
    print("//////////////////////////////\n")
    codec_obj = Codec(NUM_CHANNELS)    #   Creates objects and fills attributes
    out_buffer = bytearray()
    out_buffer.extend(codec_obj.encode_frame(codec_obj.fit_data))
    codec_obj.dump_codec_obj(codec_obj.fit_data)   #   This dumps contents of object
    out_buffer.append(255)      #   Terminate last frame
    print(len(out_buffer), ''.join(['\\x%02x' % b for b in out_buffer]))    #   Print entire buffer, which is manageable for this small test case
    codec_obj.process_byte_stream(out_buffer)

    print("\n//////////////////////////////")
    print("Do basic test of all data classes......")
    print("//////////////////////////////\n")
    codec_obj = Codec(NUM_CHANNELS)    #   Creates objects and fills attributes
    out_buffer = bytearray()
    for obj in codec_obj.data_obj_list:
        out_buffer.extend(codec_obj.encode_frame(obj))
    out_buffer.append(255)      #   Terminate last frame
    codec_obj.process_byte_stream(out_buffer)

    print("\n//////////////////////////////")
    print("Now add variations to trigger warning messages......")
    print("//////////////////////////////\n")
    #   Test 7-bit UTF-8 characters [ chr(32) to chr(126)]
    codec_obj.text_data.text.value = ' !"#$%&\'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~'
    codec_obj.crisp_data.time.value = 23
    out_buffer.append(255)
    out_buffer.extend(b'12345')
    out_buffer.append(249)
    out_buffer.extend(b'12345')
    out_buffer.append(255)      #   Terminate last frame
    codec_obj.process_byte_stream(out_buffer)

    print("\n//////////////////////////////")
    print("Print examples of 7-bit text as character, hex, and decimal......")
    print("//////////////////////////////\n")
#   bytesarray with all values that are common characters, in integer order, and less than 127
    test_buffer = b' !"#$%&\'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~'  
    print(str(test_buffer, 'UTF-8'))
    print(len(test_buffer), ''.join(['\\x%02x' % b for b in test_buffer]))
    print(len(test_buffer), ''.join([' %03d' % b for b in test_buffer]))

    print("\n//////////////////////////////")
    print("Examples of different representations of 255......")
    print("//////////////////////////////\n")
    print("As bytes (all give same result for single byte as does struct.pack('B', 255); Note bytes(255) gives an empty array of 255 elements):")
    print (f"bytes([255]): {bytes([255])},  int(255).to_bytes(1, 'big'): {int(255).to_bytes(1, 'big')},  int(255).to_bytes(1, 'little'): {int(255).to_bytes(1, 'little')}")
    print("As hex string:")
    print (f"hex(255): {hex(255)}")
    print("As character:")
    print (f"chr(255): {chr(255)}")
    print("Mixed:")
    print(f"chr(255).encode: {chr(255).encode()}")
    print("")

if __name__ == '__main__':
#    print(f'Hello {2*3}')
    test()
