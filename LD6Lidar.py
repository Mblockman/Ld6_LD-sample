import serial
import struct
import time
import math

class LD6Lidar:
    SERIAL_BAUDRATE = 230400
    DEFAULT_TIMEOUT = 500

    def __init__(self):
        self._serial = None
        self.Length = 0x2c
        self.distanceList = [0] * 360
        self.lightList = [0] * 360
        self.temp = [0] * 360
        self.CrcTable = [ 0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 
                    0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 
                    0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 
                    0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 
                    0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 
                    0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 
                    0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 
                    0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 
                    0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 
                    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 
                    0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 
                    0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 
                    0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 
                    0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03,
                    0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 
                    0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 
                    0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 
                    0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 
                    0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 
                    0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 
                    0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 
                    0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 
                    0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 
                    0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 
                    0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 
                    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 
                    0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 
                    0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c,
                    0xd1, 0x7f, 0x32, 0xe5, 0xa8 ]

    def CalCRC8(self, data):
        crc = 0
        for byte in data:
            crc = self.CrcTable[(crc ^ byte) & 0xff]
        return crc

    def connect(self, port):
        self._serial = serial.Serial(port, self.SERIAL_BAUDRATE, timeout=1)
        return self._serial.is_open

    def isOpen(self):
        return self._serial.is_open if self._serial else False
    
    def scanData(self, timeout=DEFAULT_TIMEOUT):
        start_time = time.time()
        read_data = bytearray()

        while (time.time() - start_time) < timeout / 1000.0:
            if self._serial.in_waiting > 0:
                temp = self._serial.read(1)
                if temp == b"\x54" and len(read_data) == 0:
                    read_data += temp
                    continue
                if temp == b"\x2C" and len(read_data) == 1:
                    self.Length = temp[0]
                    read_data += temp
                    read_data += self._serial.read(int(temp[0])+1)
                    break
        return read_data    
    
    def process_lidar_sumCheck(self, data):
        #print(f"data: {' '.join(map(lambda x: f'{x:02x}', data))}")
        if len(data) < self.Length+2:
            return False
        self.CRC = data[self.Length+2]
        #print(f"CRC: {self.CRC:02x} {self.Length:02x}")
        crc_value = self.CalCRC8(bytearray(data[0:self.Length+2]))
        #print(f"crc_value: {crc_value:02x}")
        if self.CRC == crc_value:
            #print("CRC check OK")
            return True
        else:
            #print("CRC check NG")
            return False


    def process_lidar_data(self, data, flag=False):

        LSN = (data[1]-8) // 3
        FSA = (data[5] << 8 | data[4]) / 100
        LSA = (data[self.Length-1] << 8 | data[self.Length-2]) / 100
        #print(f"LSN: {LSN} FSA: {FSA} LSA: {LSA}")
        Distance = [0] * LSN
        Intensity = [0] * LSN
        Angle = [0] * LSN
        for i in range(0, 3 * LSN, 3):
            Distance[i // 3] = (data[6 + i + 1] << 8 | data[6 + i])

        Angle = [0] * LSN
        Angle_Diff = (LSA - FSA)
        X_value = [0] * LSN
        Y_value = [0] * LSN
        if Angle_Diff < 0:
            Angle_Diff += 360

        for i in range(LSN):
            try:
                Angle[i] = i * Angle_Diff / (LSN - 1) + FSA
                if Distance[i] > 0:
                    #AngCorrect = math.atan(21.8 * (155.3 - Distance[i]) / (155.3 * Distance[i]))
                    if flag:
                        #Angle[i] += AngCorrect * 180 / math.pi
                        if Angle[i] >= 360:
                            Angle[i] -= 360
                    else:
                        X_value[i] = Distance[i] * math.cos(math.radians(Angle[i]))
                        Y_value[i] = Distance[i] * math.sin(math.radians(Angle[i]))
                        
            except Exception as e:
                #print(f"Error: {e} {i}")
                continue
        if flag:
            #angle = [math.radians(a) for a in Angle]
            angle_distance_pairs = [(a, d) for a, d in zip(Angle, Distance) if d > 0]
            angle, Distance = zip(*angle_distance_pairs) if angle_distance_pairs else ([], [])
            return angle, Distance        
        else:
            return X_value, Y_value      

if __name__ == "__main__":
    lidar = LD6Lidar()
    if lidar.connect('COM5'):
        while True:
            scan_data = lidar.scanData()
            if scan_data is not None:
                if lidar.process_lidar_sumCheck(scan_data):
                    angle, distance = lidar.process_lidar_data(scan_data,True)
                    if angle is not None and distance is not None:
                        print(f"angle: {angle}, distance: {distance}")   
                else:
                    print("checksum error")     
    
