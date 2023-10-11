#!/usr/bin/env python3
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

if __name__ == '__main__':
    rospy.init_node("image_publisher",anonymous=True)
    pub = rospy.Publisher('image_topic',Image,queue_size=10)
    rospy.loginfo("Started Publishing video")
    rate = rospy.Rate(10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        ret,frame= cap.read()
        pub.publish(bridge.cv2_to_imgmsg(frame))
        rospy.loginfo("Publishing video")
        #cv2.imshow('frame',frame)
        #if cv2.waitKey(1) == ord('q'):
        #    break
        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()   
