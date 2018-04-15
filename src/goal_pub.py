#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import NavSatFix

def main():
    pub = rospy.Publisher("goal_coords", NavSatFix, queue_size=10)
    rospy.init_node("goal_pub", anonymous=False)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = NavSatFix()
        msg.header.frame_id="gps_frame"
        msg.longitude = 100
        msg.latitude = 100
        msg.position_covariance = [1,0,0,0,1,0,0,0,1]
        pub.publish(msg)
        r.sleep()
        print(msg)

if __name__ == '__main__':
    main()
