#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def pose_with_covariance_callback(msg):
    # Create a new Odometry message
    odometry_msg = Odometry()

    # Copy the header from the PoseWithCovarianceStamped message
    odometry_msg.header = msg.header

    # Copy the pose from the PoseWithCovarianceStamped message
    odometry_msg.pose.pose = msg.pose.pose

    # Publish the Odometry message
    odometry_publisher.publish(odometry_msg)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('pose_to_odometry_converter')

    # Create a subscriber for the PoseWithCovarianceStamped message
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, pose_with_covariance_callback)

    # Create a publisher for the Odometry message
    odometry_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)

    # Spin ROS
    rospy.spin()