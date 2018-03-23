#!/usr/bin/env python

from time import sleep
from serial import Serial
import rospy
import numpy as np
from std_msgs.msg import Int16MultiArray

Serial_Con = Serial("/dev/ttyUSB0", baudrate=115200, timeout=0.001)
Serial_Con.setDTR(1)
dictErrors = {  1 : "Input Voltage",
                2 : "Angle Limit",
                4 : "Overheating",
                8 : "Range",
                16 : "Checksum",
                32 : "Overload",
                64 : "Instruction"
        }

AX_START=0x55
TX_DELAY_TIME = 0.00002

def moveServo(id,speed,position):
    #print("Moving Servo ID:"+str(id)+"----------------------")
    AX_REG_WRITE =1
    AX_GOAL_SP_LENGTH =7
    if(position < 0):
        position = 0
    if(position > 1000):
        position = 1000
    if(speed < 0):
        speed = 0
    if(speed > 30000):
        speed = 30000

    p = [position&0xff, position>>8]
    s = [speed&0xff, speed>>8]
    checksum = (~(id + AX_GOAL_SP_LENGTH + AX_REG_WRITE + p[0] + p[1] + s[0] + s[1]))&0xff
    outData = chr(AX_START)
    #print(hex(ord(chr(AX_START))))
    outData += chr(AX_START)
    #print(hex(ord(chr(AX_START))))
    outData += chr(id)
    #print(hex(ord(chr(id))))
    outData += chr(AX_GOAL_SP_LENGTH)
    #print(hex(ord(chr(AX_GOAL_SP_LENGTH))))
    outData += chr(AX_REG_WRITE)
    #print(hex(ord(chr(AX_REG_WRITE))))
    outData += chr(p[0])
    #print(hex(ord(chr(p[0]))))
    outData += chr(p[1])
    #print(hex(ord(chr(p[1]))))
    outData += chr(s[0])
    #print(hex(ord(chr(s[0]))))
    outData += chr(s[1])
    #print(hex(ord(chr(s[1]))))
    outData += chr(checksum)
    #print(hex(ord(chr(checksum))))
    Serial_Con.write(outData)
    sleep(TX_DELAY_TIME)
    return True
#print("Done -----------")

def ReadTemp(id):
    #print("Read Temp ID:"+str(id)+"-------------")
    AX_READ_DATA=3
    AX_TEMP_READ=26
    Serial_Con.flushInput()
    checksum = (~(id + AX_READ_DATA + AX_TEMP_READ))&0xff
    outData = chr(AX_START)
    #print(hex(ord(chr(AX_START))))
    outData += chr(AX_START)
    #print(hex(ord(chr(AX_START))))
    outData += chr(id)
    #print(hex(ord(chr(id))))
    outData += chr(AX_READ_DATA)
    #print(hex(ord(chr(AX_READ_DATA))))
    outData += chr(AX_TEMP_READ)
    #print(hex(ord(chr(AX_TEMP_READ))))
    outData += chr(checksum)
    #print(hex(ord(chr(checksum))))
    Serial_Con.write(outData)
    sleep(TX_DELAY_TIME)
    #print("Done -----------")
    count=0
    sleep(0.1)
    while count<200:
        reply=Serial_Con.read(1)
        if reply != '':
            for x in range(0, 7):
                if x == 5:
                    #print reply.encode("hex")
                    #print str(int(reply.encode("hex"),16))+"*C"
                    tempture=int(reply.encode("hex"),16)
                reply=Serial_Con.read(1)
            count=200
            #print("-----------")
            return tempture
        count+=1
        sleep(0.1)

def ReadVin(id):
    #print("Read Temp ID:"+str(id)+"-------------")
    AX_READ_DATA=3
    AX_TEMP_READ=27
    Serial_Con.flushInput()
    checksum = (~(id + AX_READ_DATA + AX_TEMP_READ))&0xff
    outData = chr(AX_START)
    #print(hex(ord(chr(AX_START))))
    outData += chr(AX_START)
    #print(hex(ord(chr(AX_START))))
    outData += chr(id)
    #print(hex(ord(chr(id))))
    outData += chr(AX_READ_DATA)
    #print(hex(ord(chr(AX_READ_DATA))))
    outData += chr(AX_TEMP_READ)
    #print(hex(ord(chr(AX_TEMP_READ))))
    outData += chr(checksum)
    #print(hex(ord(chr(checksum))))
    Serial_Con.write(outData)
    sleep(TX_DELAY_TIME)
    #print("Done -----------")
    count=0
    sleep(0.1)
    while count<200:
        reply=Serial_Con.read(1)
        if reply != '':
            for x in range(0, 7):
                #print reply.encode("hex"),
                #print str(int(reply.encode("hex"),16))+""
                if x == 5:
                    #print reply.encode("hex")
                    #print str(int(reply.encode("hex"),16))+"*C"
                    tempture=int(reply.encode("hex"),16)
                reply=Serial_Con.read(1)
            count=200
            #print("-----------")
            return tempture
        count+=1
        sleep(0.1)

def ReadPos(id):
    #print("Read Temp ID:"+str(id)+"-------------")
    AX_READ_DATA=3
    AX_TEMP_READ=28
    Serial_Con.flushInput()
    checksum = (~(id + AX_READ_DATA + AX_TEMP_READ))&0xff
    outData = chr(AX_START)
    #print(hex(ord(chr(AX_START))))
    outData += chr(AX_START)
    #print(hex(ord(chr(AX_START))))
    outData += chr(id)
    #print(hex(ord(chr(id))))
    outData += chr(AX_READ_DATA)
    #print(hex(ord(chr(AX_READ_DATA))))
    outData += chr(AX_TEMP_READ)
    #print(hex(ord(chr(AX_TEMP_READ))))
    outData += chr(checksum)
    #print(hex(ord(chr(checksum))))
    Serial_Con.write(outData)
    sleep(TX_DELAY_TIME)
    #print("Done -----------")
    count=0
    sleep(0.1)
    while count<200:
        reply=Serial_Con.read(1)
        if reply != '':
            for x in range(0, 8):
                #print reply.encode("hex"),
                #print str(int(reply.encode("hex"),16))+""
                if x == 5:
                    #print reply.encode("hex")
                    #print str(int(reply.encode("hex"),16))+"*C"
                    pos1=reply.encode("hex")
                if x == 6:
                    #print reply.encode("hex")
                    #print str(int(reply.encode("hex"),16))+"*C"
                    pos2=int(reply.encode("hex")+pos1,16)
                reply=Serial_Con.read(1)
            count=200
            #print("-----------")
            return pos2
        count+=1
        sleep(0.1)

def controller(data):
    error = data.data
    print error
    errX = error[0]
    errY = error[1]
    currentPan = ReadPos(1)
    currentTilt = ReadPos(2)
    panCommand = currentPan - errX
    tiltCommand = currentTilt - errY
    moveServo(1,0,panCommand)
    moveServo(2,0,tiltCommand)


def servoControl():
    rospy.init_node('servo_controller', anonymous=True)
    global center_err_sub
    center_err_sub = rospy.Subscriber('/center_error', Int16MultiArray, controller)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == '__main__':
    servoControl()
