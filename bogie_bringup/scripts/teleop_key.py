#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

MAX_LIN_VEL = 0.5
MAX_ANG_VEL = 0.10

#MAX_LIN_VEL = rospy.get_param('~linear_speed_limit', 1.0)
#MAX_ANG_VEL = rospy.get_param('~angular_speed_limit', 5.0)

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.05

msg = """
Control Your Robot
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit
"""

e = """
Communications Failed
"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input


def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    return vel


def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
    return vel


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_key')
    MAX_LIN_VEL = rospy.get_param('~linear_vel_max', 0.5)
    MAX_ANG_VEL = rospy.get_param('~angular_vel_max', 0.8)
    LIN_VEL_STEP_SIZE = rospy.get_param('~linear_vel_step', 0.01)
    ANG_VEL_STEP_SIZE = rospy.get_param('~angular_vel_step', 0.05)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        print msg
        while(1):
            key = getKey()
            if key == 'w':
                target_linear_vel = checkLinearLimitVelocity(
                    target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel, target_angular_vel)
            elif key == 'x':
                target_linear_vel = checkLinearLimitVelocity(
                    target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel, target_angular_vel)
            elif key == 'a':
                target_angular_vel = checkAngularLimitVelocity(
                    target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel, target_angular_vel)
            elif key == 'd':
                target_angular_vel = checkAngularLimitVelocity(
                    target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel, target_angular_vel)
            elif key == ' ' or key == 's':
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print vels(target_linear_vel, target_angular_vel)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print msg
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(
                control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/4.0))
            twist.linear.x = control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(
                control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/4.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
