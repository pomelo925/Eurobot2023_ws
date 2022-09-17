import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, MagneticField
import math

import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

G = 9.80665
MagFieldConversion_uT_T = 0.000001
mag_pub_count = 0
imu_pub_rate = 200
mag_pub_rate = 100

imu_msg = Imu()
mag_msg = MagneticField()

mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_250,
    afs=AFS_2G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ)

def updateImuData():    
    # print("mpu9250 raw data")
    # create imu msg
    q0 = 1.0 #W
    q1 = 0.0 #X
    q2 = 0.0 #Y
    q3 = 0.0 #Z

    #Fill imu message
    imu_msg.header.stamp = rospy.get_rostime()
    imu_msg.header.frame_id = 'mpu9250_frame'

    imu_msg.orientation.x = q0
    imu_msg.orientation.y = q1
    imu_msg.orientation.z = q2
    imu_msg.orientation.w = q3
    imu_msg.orientation_covariance[0] = 0.01
    imu_msg.orientation_covariance[4] = 0.01
    imu_msg.orientation_covariance[8] = 0.01


    gx, gy, gz = mpu.readGyroscopeMaster()
    imu_msg.angular_velocity.x = math.radians(gx)
    imu_msg.angular_velocity.y = math.radians(gy)
    imu_msg.angular_velocity.z = math.radians(gz)
    imu_msg.angular_velocity_covariance[0] = 0.01
    imu_msg.angular_velocity_covariance[4] = 0.01
    imu_msg.angular_velocity_covariance[8] = 0.01

    ax, ay, az = mpu.readAccelerometerMaster()
    imu_msg.linear_acceleration.x = ax*G
    imu_msg.linear_acceleration.y = ay*G
    imu_msg.linear_acceleration.z = az*G
    imu_msg.linear_acceleration_covariance[0] = 10
    imu_msg.linear_acceleration_covariance[4] = 10
    imu_msg.linear_acceleration_covariance[8] = 10

def updateMagData():    
    mx, my, mz = mpu.readMagnetometerMaster()

    mag_msg.header.stamp = rospy.get_rostime()
    mag_msg.header.frame_id = 'mpu9250_frame'

    mag_msg.magnetic_field.x = mx*MagFieldConversion_uT_T
    mag_msg.magnetic_field.y = my*MagFieldConversion_uT_T
    mag_msg.magnetic_field.z = mz*MagFieldConversion_uT_T
    mag_msg.magnetic_field_covariance[0] = 0.01
    mag_msg.magnetic_field_covariance[4] = 0.01
    mag_msg.magnetic_field_covariance[8] = 0.01    

def imuTimerCallback(event):
    updateImuData()
    imu_pub.publish(imu_msg)
    
# def magTimerCallback(event):
#     updateMagData()
    # mag_pub.publish(mag_msg)

if __name__ == '__main__':
    
    # write mpu9250 register
    mpu.configure()
    # mpu.calibrate()
    # mpu.configure()

    print("Calibration done! Start publishing data!") 
    
    # initial node
    rospy.init_node('mpu9250', anonymous=True)
    6
    imu_pub = rospy.Publisher('imu_raw', Imu, queue_size=10)
    # mag_pub = rospy.Publisher('mag_raw', MagneticField, queue_size=10)
    
    rospy.Timer(rospy.Duration(1/imu_pub_rate), imuTimerCallback)
    # rospy.Timer(rospy.Duration(1/mag_pub_rate), magTimerCallback)
    rospy.spin()
    # try:
    #     mpu9250_init()
    # except rospy.ROSInterruptException:
    #     pass
