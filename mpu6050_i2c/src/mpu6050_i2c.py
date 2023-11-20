#!/usr/bin/python

import smbus
import math
import time
import rospy
import tf
import geometry_msgs
from sensor_msgs.msg import Imu
import os
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_byte(adr):
    return bus.read_i2c_block_data(address, adr,1)[0]

def read_word(adr):
    high = bus.read_i2c_block_data(address, adr,1)[0]
    low = bus.read_i2c_block_data(address, adr + 1,1)[0]
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a, b):
    return math.sqrt((a * a) + (b * b))


def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)

def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)

def read_acc_gyro_data():

    # print "Gyroscope data"
    # print "--------------"
    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)
    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)


    gyro_xout_scaled=gyro_xout/131.0
    gyro_yout_scaled=gyro_yout/131.0
    gyro_zout_scaled=gyro_zout/131.0
    
    # print "{}\t{}\t{}\t{}".format ("X out: ", gyro_xout, "scaled: ", gyro_xout_scaled)
    # print "{}\t{}\t{}\t{}".format ("Y out: ", gyro_yout, " scaled: ", gyro_yout_scaled)
    # print "{}\t{}\t{}\t{}".format ("Z out: ", gyro_zout, " scaled: ", gyro_zout_scaled)

    
    # print
    # print "Accelerometer data"
    # print "------------------"

    

    accel_xout_scaled = accel_xout / 16384.0
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0
    # print "{}\t{}\t{}\t{}".format ("X out: ", accel_xout, " scaled: ", accel_xout_scaled)
    # print "{}\t{}\t{}\t{}".format ("Y out: ", accel_yout, " scaled: ", accel_yout_scaled)
    # print "{}\t{}\t{}\t{}".format ("Z out: ", accel_zout, " scaled: ", accel_zout_scaled)
    
    # print

    # print "X rotation: ", get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    # print "Y rotation: ", get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    
    

    # print("{} {} {} {} {} {}".format(gyro_xout_scaled,gyro_yout_scaled,gyro_zout_scaled,accel_xout_scaled,accel_yout_scaled,accel_zout_scaled))
    return gyro_xout_scaled,gyro_yout_scaled,gyro_zout_scaled,accel_xout_scaled,accel_yout_scaled,accel_zout_scaled
if __name__ == "__main__":
    rospy.init_node('mpu6050')
    topic=rospy.get_param('~topic', '/imu/data_raw')
    frame_id = rospy.get_param('~frame_id', 'imu')
    data =[name for name in os.listdir("/sys/bus/pci/devices/0000:00:16.1/i2c_designware.1") ] # for up 4000
    #data =[name for name in os.listdir("/sys/bus/pci/devices/0000:00:19.0/i2c_designware.3") ] #for up xtreme

    matching = [s for s in data if "i2c" in s]
    i2c_port = int(matching[0][-1])

    bus = smbus.SMBus(i2c_port)
    address = 0x68 #default=0x68
    bus.write_byte_data(address, power_mgmt_1, 0)            

    pub = rospy.Publisher(topic, Imu, queue_size=10)

    r = rospy.Rate(10)
    imu_data=Imu()
    imu_data.header.frame_id=frame_id

    while not rospy.is_shutdown() :
        imu_data.header.stamp=rospy.Time.now()

        gyro_xout_scaled,gyro_yout_scaled,gyro_zout_scaled,accel_xout_scaled,accel_yout_scaled,accel_zout_scaled=read_acc_gyro_data()
   
    
        imu_data.angular_velocity.x=gyro_xout_scaled*3.14/180.0
        imu_data.angular_velocity.y=gyro_yout_scaled*3.14/180.0
        imu_data.angular_velocity.z=gyro_zout_scaled*3.14/180.0

        imu_data.linear_acceleration.x=accel_xout_scaled*9.8
        imu_data.linear_acceleration.y=accel_yout_scaled*9.8
        imu_data.linear_acceleration.z=accel_zout_scaled*9.8

        
        pub.publish(imu_data)
        r.sleep()