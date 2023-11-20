#! /usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32,Int16
from geometry_msgs.msg import Twist

port = serial.Serial("/dev/bogie", baudrate=115200, timeout=3.0)

MAX_LIN_VEL = 0.5
MAX_ANG_VEL = 0.3
dead_zone = 50

LIN_data = 2
ANG_data = 4

def robot_Depth_CB(msg):
    global Depth
    Depth = msg.data  

if __name__ == "__main__":
    pub_rc_command = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.init_node('rc_direct_control')
    r = rospy.Rate(10)
    twist = Twist()
    while not rospy.is_shutdown():
        rcv = port.readline()
        
        dataMessage = rcv.split(' ')
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        #print(dataMessage)
        if (dataMessage[0] == 'x') and (dataMessage[11])[:-2] == 'x':
            linear = 0.0
            if abs(int(dataMessage[LIN_data])) > dead_zone:
                if int(dataMessage[LIN_data]) > 0 :
                    linear = ((float(dataMessage[LIN_data])-dead_zone) / (400-dead_zone) ) * MAX_LIN_VEL
                elif int(dataMessage[LIN_data]) < 0:
                    linear = ((float(dataMessage[LIN_data])+dead_zone) / (400-dead_zone) ) * MAX_LIN_VEL
                else:
                    linear = 0
            else:
                linear = 0

            if abs(int(dataMessage[ANG_data])) > dead_zone:
                if int(dataMessage[LIN_data]) > (-1 * (dead_zone)):
                    if int(dataMessage[ANG_data]) > 0 :
                        angular = - ((float(dataMessage[ANG_data])-dead_zone) / (400-dead_zone) ) * MAX_ANG_VEL
                    elif int(dataMessage[ANG_data]) < 0:
                        angular = - ((float(dataMessage[ANG_data])+dead_zone) / (400-dead_zone) ) * MAX_ANG_VEL
                    else:
                        angular = 0
                else:
                    if int(dataMessage[ANG_data]) > 0 :
                        angular = ((float(dataMessage[ANG_data])-dead_zone) / (400-dead_zone) ) * MAX_ANG_VEL
                    elif int(dataMessage[ANG_data]) < 0:
                        angular = ((float(dataMessage[ANG_data])+dead_zone) / (400-dead_zone) ) * MAX_ANG_VEL
                    else:
                        angular = 0    
            else:
                angular = 0.0 

            twist.linear.x = linear
            twist.angular.z = angular   
            print(linear,angular)

        pub_rc_command.publish(twist)
        #r.sleep()
