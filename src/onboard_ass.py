#!/usr/bin/env python2
import serial
import pynmea2
import rospy
from urc.msg import FloatArray

#Plug the tx0 of the GPS into pin 4 on the uart header
'''
1. Altitude (m)
2. Temperature (C)
3. Absolute Pressure (mb)
4. Sea-level Pressure (mb)
5. Humidity (%) 
6. Sensor Voltage (mV)
7-9. Magnetometer X,Y,Z
'''

def main():
    ser = serial.Serial("/dev/ttyUSB0", 9600)
    pub = rospy.Publisher('ass_data', FloatArray, queue_size=10)
    rospy.init_node('ass', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        dat = ser.readline()
	print(dat)
	dat=dat.strip("\n")
	datArr=dat.split(",")
        for i in range(len(datArr)):
            if "INF" in datArr[i]:
                datArr[i] = 0
            else:
                datArr[i] = float(datArr[i])
        if len(datArr) == 9:
            pub.publish(datArr)
	else:
	    print("Error")

if __name__ == '__main__':
    main()
