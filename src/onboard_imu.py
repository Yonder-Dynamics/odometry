#!/usr/bin/env python

import RTIMU
import os.path
import rospkg
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
    path = rospkg.RosPack().get_path("odometry")
    print(path)
    SETTINGS_FILE = path+"/src/RTIMULib"
    if not os.path.isfile(SETTINGS_FILE+".ini"):
        print("Settings file does not exist")
        print(SETTINGS_FILE)
        return

    s = RTIMU.Settings(SETTINGS_FILE)
    #print(s.MPU9250GyroAccelSampleRate)
    imu = RTIMU.RTIMU(s)

    if (not imu.IMUInit()):
        print("Failed to init IMU")
        sys.exit(1)

    imu.setGyroEnable(True)  
    imu.setAccelEnable(True)  
    imu.setCompassEnable(False)  

    poll_interval = imu.IMUGetPollInterval() 
    print(poll_interval)
    hz = 1./poll_interval*1000.0
    print(hz)

    pubIMU = rospy.Publisher('imu/data', Imu, queue_size=10)
    #pubMag = rospy.Publisher('/imu/magnetic_field', MagneticField, queue_size=10)
    #pubGPS = rospy.Publisher('/arduino/gps', NavSatFix, queue_size=10)
    rospy.init_node('imu', anonymous=True)
    r = rospy.Rate(hz)
    start = time.time()
    record = [0]*5

    while not rospy.is_shutdown():
        if imu.IMURead():
            record.pop(0)
            record.append(time.time() - start)
            start = time.time()
            print(1/sum(record)*len(record))
            data = imu.getIMUData()
            #print(data)
            gyro = data["gyro"]
            # subtract gravity
<<<<<<< HEAD
            rot = pyquaternion.Quaternion(axis=[0,0,1], degrees=0)
            q = pyquaternion.Quaternion(data["fusionQPose"][2], data["fusionQPose"][1], data["fusionQPose"][0], data["fusionQPose"][3])
            q = q * rot
            grav = q.rotate([0,0,-1])
=======
            q = pyquaternion.Quaternion(data["fusionQPose"][0], data["fusionQPose"][1], data["fusionQPose"][2], data["fusionQPose"][3])
            axis = [1, 0, 0]
            angle = math.pi
            rot180 = pyquaternion.Quaternion(math.cos(angle/2), math.sin(angle/2)*axis[0], math.sin(angle/2)*axis[1], math.sin(angle/2)*axis[2])
            q = rot180*q*rot180.inverse
            # quaternion is upside down
>>>>>>> 0f5e648eae4dfa2aa21e725f872b762e627f8bbd
            #print(np.multiply(grav, 9.80665))
            accel = data["accel"]
            #accel = np.add(accel, grav)

            acc = Vector3()
            acc.x = accel[0] * 9.80665
            acc.y = accel[1] * 9.80665
            acc.z = accel[2] * 9.80665

            gyro = Vector3()
            gyro.x = -data["gyro"][0]
            gyro.y = data["gyro"][1]
            gyro.z = -data["gyro"][2]

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
            quat.x = q[1]
            quat.y = q[2]
            quat.z = q[3]
            quat.w = q[0]

            imu_dat = Imu()
            imu_dat.header.frame_id = IMU_FRAME_ID
            imu_dat.header.stamp = rospy.Time.now()
            imu_dat.linear_acceleration = acc
            imu_dat.linear_acceleration_covariance = [0.04, 0, 0,
                                                      0, 0.04, 0,
                                                      0, 0, 0.04]
            imu_dat.orientation = quat
            imu_dat.orientation_covariance = [0.0025, 0, 0,
                                              0, 0.0025, 0,
                                              0, 0, 0.0025]
            imu_dat.angular_velocity = gyro
            imu_dat.angular_velocity_covariance = [0.02, 0, 0,
                                                   0, 0.02, 0,
                                                   0, 0, 0.02]
            pubIMU.publish(imu_dat)
            #print(imu_dat)
            r.sleep()

if __name__ == '__main__':
    main()
