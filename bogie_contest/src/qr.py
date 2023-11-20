#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from pyzbar import pyzbar
from std_msgs.msg import String



def image_callback(msg):
    bridge = CvBridge()
    data = ""
    try:
        # Convert the ROS image message to OpenCV image
        publisher_qr = rospy.Publisher('/usb_cam/qr', Image, queue_size=10)
        publisher_qr_data = rospy.Publisher('/qr_data', String, queue_size=10)


        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert the OpenCV image to a ROS image message
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect QR codes in the image
        qr_codes = pyzbar.decode(gray)


        # Process each detected QR code
        for qr_code in qr_codes:
            # Extract the QR code's data
            data = qr_code.data.decode("utf-8")
            print(data)
            

            points = qr_code.polygon
            if len(points) > 4 :
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else :
                hull = points
            #print(hull[0][0])
            # Number of points in the convex hull
            n = len(hull)
        
            # Draw the convext hull
            for j in range(0,n):
                cv2.line(cv_image, hull[j], hull[ (j+1) % n], (255,0,0), 3)
            
            cv2.putText(cv_image,'data',(hull[0][0],hull[0][1]),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255), 2,cv2.LINE_AA, False)

            
            #rospy.loginfo("QR code detected: %s", data)
        
        
        # Publish the ROS image message to a topic if needed
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        publisher_qr.publish(ros_image)
        publisher_qr_data.publish(data)
    except CvBridgeError as e:
        print(e)
        return
    

def image_listener():
    rospy.init_node('image_listener')
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        image_listener()
    except rospy.ROSInterruptException:
        pass
