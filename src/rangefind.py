#!/usr/bin/env python

import rospy
from serial import Serial
import numpy as np
from std_msgs.msg import Float32
from math import e



def rangecal():
    pass

    # anRead = ser.readline()
    # formanRead = float(anRead.strip('\0'))
    # print formanRead







def range_setup():
    global ser
    global rangePub
    global avg_arr
    avg_arr = [0,0,0,0,0]
    ser = Serial('/dev/ttyACM0', 9600, timeout = 0.01)
    rangePub = rospy.Publisher('z_dist', Float32, queue_size = 1)

    while(1):

        try:
            anRead = float(ser.readline())
            z_dist = 0.2287193 + 36490770*(e**(-13.07154*anRead))
            avg_arr = np.append(avg_arr, z_dist)
            if (len(avg_arr) > 5):
                avg_arr = np.delete(avg_arr, 0)
            dist_avg = np.mean(avg_arr)
            print dist_avg
        except ValueError:
            pass




if __name__ == '__main__':
    range_setup()
