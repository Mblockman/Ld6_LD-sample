import serial
import binascii
import Ld6ladar
import matplotlib.pyplot as plt
import math

fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='polar')
ax.set_title('lidar (exit: Key Q)',fontsize=18)

plt.connect('key_press_event', lambda event: exit(1) if event.key == 'q' else None)

ser = serial.Serial(port='COM4',
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

#tmpString = ""
lines = list()
angles = list()
distances = list()

ladar = Ld6ladar.Ld6ladar()
ladar.setCrc(True) #crc 체크 처리
i = 0
while True:
    loopFlag = True
    flag2c = False

    if(i % 40 == 39):
        try:
            if('line' in locals()):
                line.remove()
            line = ax.scatter(angles, distances, c="blue", s=2)
        except:
            pass
        ax.set_theta_offset(math.pi / 2)
        plt.pause(0.01)
        angles.clear()
        distances.clear()
        i = 0
        
    temp_data = bytearray()
    while loopFlag:
        b = None
        b = ser.read()
        tmpInt = int.from_bytes(b, 'big')
        temp_data.append(tmpInt)

        if (tmpInt == 0x54):
            flag2c = True
            continue
        
        elif(tmpInt == 0x2c and flag2c):
            temp_angles, temp_distances, _ = ladar.calculate_data(temp_data, True) # 라디안 추출유무 처리
            angles.extend(temp_angles)
            distances.extend(temp_distances)
            tmpString = ""
            temp_data = bytearray()
            loopFlag = False

        flag2c = False
    
    i +=1

ser.close()
