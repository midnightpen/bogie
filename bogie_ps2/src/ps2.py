#! /usr/bin/env python
import serial
import os
import rospy
from geometry_msgs.msg import Twist

max_linear=0.2
max_angular=0.2

def com_port():
    flag = False
    while flag != True: 
        try :
            ser = serial.Serial("/dev/ttyUSB1", 57600)
            flag = ser.is_open
            print("Successfuly connect port")

            return ser 
        except:
            print("Fail to connect port")

def map_range(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def publish_cmd_vel(ser):
    rospy.init_node('bogie_ps2', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(20)  # Adjust the publishing rate as needed
    twist_msg = Twist()

    while not rospy.is_shutdown(): 
        data=ser.readline()
        data=data.strip()
        dataMessage = data.split(' ')

        if dataMessage[0] != 'x' and dataMessage[-1] != 'x':
            pass
        else:
            x=int(dataMessage[1])
            y=int(dataMessage[-2])

            if x > 140 or x < 114:

                twist_msg.linear.x = round(map_range(x,0,255,max_linear*-1,max_linear),3)
            else :

                twist_msg.linear.x = 0

            if y > 140 or y < 114:

                twist_msg.angular.z = round(map_range(y,0,255,max_linear*-1,max_linear),3)
            else :

                twist_msg.angular.z = 0


            #print(twist_msg)
            pub.publish(twist_msg)
            #rate.sleep()

if __name__ == '__main__':
    try:
        ser=com_port()
        publish_cmd_vel(ser)
    except KeyboardInterrupt:
        print('Interrupted')
        exit()
    