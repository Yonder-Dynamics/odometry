#!/usr/bin/env python

import RTIMU
import os.path
import time, math, operator, socket
import rospy
import sys
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from collections import deque
import pyquaternion
import numpy as np

IMU_FRAME_ID = "imu_link"

def main():
    SETTINGS_FILE = "/home/ubuntu/catkin_ws/src/urc/src/RTIMULib"

    s = RTIMU.Settings(SETTINGS_FILE)
    print(s)
    imu = RTIMU.RTIMU(s)

    if (not imu.IMUInit()):
        print("Failed to init IMU")
        sys.exit(1)

    imu.setGyroEnable(True)  
    imu.setAccelEnable(True)  
    imu.setCompassEnable(False)  

    poll_interval = imu.IMUGetPollInterval() 

    pubIMU = rospy.Publisher('imu/data', Imu, queue_size=10)
    #pubMag = rospy.Publisher('/imu/magnetic_field', MagneticField, queue_size=10)
    #pubGPS = rospy.Publisher('/arduino/gps', NavSatFix, queue_size=10)
    rospy.init_node('imu', anonymous=True)
    r = rospy.Rate(1000 / imu.IMUGetPollInterval())

    while not rospy.is_shutdown():
        if imu.IMURead():
            data = imu.getIMUData()
            #print(data)
            gyro = data["gyro"]
            # subtract gravity
            rot = pyquaternion.Quaternion(axis=[0,0,1], degrees=0)
            q = pyquaternion.Quaternion(data["fusionQPose"][2], data["fusionQPose"][1], data["fusionQPose"][0], data["fusionQPose"][3])
            q1 = pyquaternion.Quaternion(data["fusionQPose"][0], data["fusionQPose"][1], data["fusionQPose"][2], data["fusionQPose"][3])
            q = q * rot
            grav = q.rotate([0,0,-1])
            #print(np.multiply(grav, 9.80665))
            accel = data["accel"]
            #accel = np.add(accel, grav)

            acc = Vector3()
            acc.x = accel[0] * 9.80665
            acc.y = accel[1] * -9.80665
            acc.z = accel[2] * 9.80665

            gyro = Vector3()
            gyro.x = -data["gyro"][0]
            gyro.y = data["gyro"][1]
            gyro.z = data["gyro"][2]

            msg = MagneticField()
            msg.header.frame_id = IMU_FRAME_ID
            mag = Vector3()
            mag.x = data["compass"][0] / 1000000
            mag.y = data["compass"][1] / 1000000
            mag.z = data["compass"][2] / 1000000
            msg.magnetic_field = mag

            quat = Quaternion()
            """
            quat.x = data["fusionQPose"][0]
            quat.y = data["fusionQPose"][1]
            quat.z = data["fusionQPose"][2]
            quat.w = data["fusionQPose"][3]
            """
            quat.x = q[0]
            quat.y = q[1]
            quat.z = q[2]
            quat.w = q[3]

            imu_dat = Imu()
            imu_dat.header.frame_id = IMU_FRAME_ID
            imu_dat.header.stamp = rospy.Time.now()
            imu_dat.linear_acceleration = acc
            imu_dat.orientation_covariance = [0.001, 0, 0,
                                              0, 0.001, 0,
                                              0, 0, 0.001]
            imu_dat.angular_velocity = gyro
            imu_dat.angular_velocity_covariance = [0.001, 0, 0,
                                                   0, 0.001, 0,
                                                   0, 0, 0.001]
            imu_dat.orientation = quat
            imu_dat.linear_acceleration_covariance = [0.001, 0, 0,
                                                      0, 0.001, 0,
                                                      0, 0, 0.001]
            pubIMU.publish(imu_dat)
            print(imu_dat)
            r.sleep()

if __name__ == '__main__':
    main()
