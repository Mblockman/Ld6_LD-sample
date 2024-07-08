import math
import struct

class Ld6ladar:
    def __init__(self, flag=False):
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
        self.Start_angle = 0.0
        self.End_angle = 0.0
        self.CRC = 0x00
        self.Speed = 0
        self.TimeStamp = 0.0
        self.CRC_flag = flag
        self.limitDistance = 100
        self.limitConfidence = 150
        self.dataLen = 90
        self.Data = []



    def CalCRC8(self, data):
        crc = 0
        for byte in data:
            crc = self.CrcTable[(crc ^ byte) & 0xff]
        return crc
    
    def Reset(self):
        self.Confidence_i = list()
        self.Angle_i = list()
        self.Distance_i = list()
    
    def setData(self, data):
        #print(" ".join(f"{byte:02x}" for byte in data))
        try:
            if data[0] == 0x54 and data[1] == 0x2C:
                data = data[2:]
            if data[0] == 0x2C:
                data = data[1:]
            #print("len(data): ", len(data),len(data[0:-3]),int(self.dataLen/2-1), hex(data[int(self.dataLen/2-1)]))
            self.CRC = data[int(self.dataLen/2-1)]
            #print(f"CRC: {self.CRC:02x} {int(90/2-1)}")
            #if len(data) > (self.dataLen/2-1):
                #print(f"{data[-4]:02x} {data[-3]:02x} {data[-2]:02x}  {data[-1]:02x}")
            data = data[:int(self.dataLen/2)]
        except:
            pass
        self.Data = data
        #print(" ".join(f"{byte:02x}" for byte in self.Data))
    
    def setLimitDistance(self, distance):
        self.limitDistance = distance

    def setLimitConfidence(self, confidence):
        self.limitConfidence = confidence

    def getLimitDistance(self):
        return self.limitDistance
    
    def getLimitConfidence(self):
        return self.limitConfidence
    
    def setCrc(self, data):
        self.CRC_flag = data

    def getConfidence(self):
        return self.Confidence_i
    
    def getAngle(self):
        return self.Angle_i
    
    def getDistance(self):
        return self.Distance_i

    def checkCrc(self):
        result = False
        #print("********************************************")
        if self.CRC_flag == True:
            crc_value = self.CalCRC8(bytearray([0x54, 0x2C]) + bytearray(self.Data[0:0x2c]))
            #print(f"crc_value: {crc_value:02x} {self.CRC:02x} {'TRUE' if self.CRC == crc_value else 'FALSE'}")
            if crc_value == self.CRC:
                result = True
        else:
            result = True
        return result
    
    def parseData(self, parseflag):
        str = "".join(f"{byte:02x}" for byte in self.Data)
        #print("********************************************")
        if len(str) != self.dataLen and self.CRC_flag == False:
            #print(f"str: {len(str)} {self.dataLen} {self.CRC_flag}")
            #print("********************************************")
            return [], [], False
        
        self.Speed = struct.unpack('<H', self.Data[0:2])[0] / 100
        self.Start_angle = struct.unpack('<H', self.Data[2:4])[0] / 100.0
        self.End_angle = struct.unpack('<H', self.Data[-5:-3])[0] / 100.0
        self.TimeStamp = struct.unpack('<H', self.Data[-3:-1])[0]
        self.CS = self.Data[-1]
        #print(f"{self.Speed} {self.Start_angle} {self.End_angle} {self.TimeStamp} {self.CS} {self.End_angle - self.Start_angle}")
        if(self.End_angle-self.Start_angle > 0):
            angleStep = float(self.End_angle-self.Start_angle)/(12)
        else:
            angleStep = float((self.End_angle+360)-self.Start_angle)/(12)
        
        points = []
        Distance_i = []
        Confidence_i = []
        Angle_i = []
        counter = 0
        circle = lambda deg : deg - 360 if deg >= 360 else deg
        for i in range(0,6*12,6): 
            distance = int(str[8+i+2:8+i+4] + str[8+i:8+i+2],16)/100
            confidence = int(str[8+i+4:8+i+6],16)
            angle = circle(angleStep*counter+self.Start_angle)*math.pi/180.0
            if distance > self.limitDistance: #거리 제한
                continue
            if confidence < self.limitConfidence: #인식율 제한
                continue

            counter += 1
            if parseflag == True:
                Distance_i.append(distance)
                Confidence_i.append(confidence)
                Angle_i.append(angle)            
            else:
                points.append((distance * math.cos(angle)*100, distance * math.sin(angle)*100))
        if parseflag == True:
            return Angle_i, Distance_i, True
        else:
            return points, [], True
            
       
    def calculate(self, parseflag):
        if self.checkCrc() != True:
            return list(), list(), False
        else:
            return self.parseData(parseflag)

    def calculate_data(self, data, parseflag):
        self.setData(data)
        if self.checkCrc() != True:
            return list(), list(), False
        else:
            return self.parseData(parseflag)        


