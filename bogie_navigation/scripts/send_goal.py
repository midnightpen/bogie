#!/usr/bin/env python

import rospy
import tf
import geometry_msgs
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

loop_check=False

def send_waypoints():
    # Initialize the ROS node
    rospy.init_node('send_waypoints_node', anonymous=True)

    # Set the header information
    header = rospy.Header(stamp=rospy.Time.now(), frame_id='map')

    # Retrieve the waypoints parameter from the launch file
    waypoints_param = rospy.get_param('~waypoints', '[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]')
    loop = rospy.get_param('~loop', "false")

    rospy.loginfo("loop %s" , loop) 

    waypoints = eval(waypoints_param)  # Convert string representation to list
    rospy.loginfo(waypoints) 

    # Define the waypoints

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    
    pose_stamped_waypoints = []
    for waypoint in waypoints:
        
        x, y, yaw = waypoint

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

        pose_stamped = MoveBaseGoal()
        pose_stamped.target_pose.header=header
        pose_stamped.target_pose.pose.position.x=x
        pose_stamped.target_pose.pose.position.y=y
        pose_stamped.target_pose.pose.orientation.z= quaternion[2]
        pose_stamped.target_pose.pose.orientation.w= quaternion[3]



        pose_stamped_waypoints.append(pose_stamped)
    
    # Publish each waypoint
    while True:
        for waypoint in pose_stamped_waypoints:
            quaternion = (
                waypoint.target_pose.pose.orientation.x,
                waypoint.target_pose.pose.orientation.y,
                waypoint.target_pose.pose.orientation.z,
                waypoint.target_pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            rospy.loginfo("Waypoint sent: x=%.2f, y=%.2f, yaw=%.2f", waypoint.target_pose.pose.position.x, waypoint.target_pose.pose.position.y, euler[2])
            client.send_goal(waypoint)

            wait = client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                if client.get_result():
                    rospy.loginfo("Goal execution done!")
        if loop is not True:
            rospy.loginfo("All waypoints sent")
            break

if __name__ == '__main__':
    try:
        send_waypoints()
    except rospy.ROSInterruptException:
        pass
