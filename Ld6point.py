import serial
import binascii
import Ld6ladar
import matplotlib.pyplot as plt
import math

fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111)
ax.set_title('lidar (exit: Key Q)', fontsize=18)
ax.set_xlim(-2500, 2500)
ax.set_ylim(-2500, 2500)

fig.canvas.mpl_connect('key_press_event', lambda event: exit(1) if event.key == 'q' else None)

ser = serial.Serial(port='COM4',
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

points = list()

ladar = Ld6ladar.Ld6ladar()
ladar.setCrc(True) #crc 체크 처리
i = 0
while True:
    loopFlag = True
    flag2c = False

    if(i % 40 == 0):
        plt.clf()
        plt.xlim(-2500, 2500)  # x축 고정
        plt.ylim(-2500, 2500)  # y축 고정   
        if points:
            x, y = zip(*points)
            plt.scatter(x, y, c="blue", s=2)
        plt.draw()
        plt.pause(0.01)
        points.clear()        
        i = 1

    
        
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
            point,_,_ = ladar.calculate_data(temp_data, False) # 라디안 추출유무 처리
            points.extend(point)
            tmpString = ""
            temp_data = bytearray()
            loopFlag = False

        flag2c = False
    
    i +=1

ser.close()
