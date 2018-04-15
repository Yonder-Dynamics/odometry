#!/usr/bin/env python2
import serial
import pynmea2
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

#Plug the tx0 of the GPS into pin 4 on the uart header

port = "/dev/ttyAMA0"

def main():
    ser = serial.Serial(port, 9600)
    pub_nfs = rospy.Publisher('gps_nfs', NavSatFix, queue_size=10)
    pub_odo = rospy.Publisher('gps_odo', Odometry, queue_size=10)
    rospy.init_node('internal_gps', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        gpsdat = ser.readline()
        if gpsdat[0:6] == "$GPGGA":
            msg = pynmea2.parse(gpsdat)
            r_msg = NavSatFix()
            r_msg.latitude = msg.latitude
            r_msg.longitude = msg.longitude
            r_msg.altitude = msg.altitude
            c = msg.gps_qual
            r_msg.position_covariance = \
                    [c,0,0, 0,c,0, 0,0,c]
            pub_nfs.publish(r_msg)
            o_msg = Odometry()
            o_msg.header.frame_id = "gps_frame"
            o_msg.pose.pose.position.x = msg.latitude
            o_msg.pose.pose.position.y = msg.longitude
            o_msg.pose.pose.position.z = msg.altitude
            o_msg.pose.pose.orientation.x = 1
            o_msg.pose.pose.orientation.y = 0
            o_msg.pose.pose.orientation.z = 0
            o_msg.pose.pose.orientation.w = 0
            h = 99999
            o_msg.pose.covariance = [c, 0, 0, 0, 0, 0,
                                     0, c, 0, 0, 0, 0,
                                     0, 0, c, 0, 0, 0,
                                     0, 0, 0, h, 0, 0,
                                     0, 0, 0, 0, h, 0,
                                     0, 0, 0, 0, 0, h]
            o_msg.twist.twist.linear.x = 0
            o_msg.twist.twist.linear.y = 0
            o_msg.twist.twist.linear.z = 0
            o_msg.twist.twist.angular.x = 0
            o_msg.twist.twist.angular.y = 0
            o_msg.twist.twist.angular.z = 0
            o_msg.twist.covariance = [h, 0, 0, 0, 0, 0,
                                      0, h, 0, 0, 0, 0,
                                      0, 0, h, 0, 0, 0,
                                      0, 0, 0, h, 0, 0,
                                      0, 0, 0, 0, h, 0,
                                      0, 0, 0, 0, 0, h]
            print(r_msg)
            pub_odo.publish(o_msg)
            

if __name__ == '__main__':
    main()
