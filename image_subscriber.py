#!/usr/bin/env python3
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def Image_callback(msg : Image) :
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    edge = cv2.Canny(gray,100,200)
    new_image = np.zeros(cv_image.shape,np.uint8)
    new_image = cv2.resize(new_image,(0,0),fx=2,fy=1)
    width = new_image.shape[1]
    new_image[:,:width//2]= cv_image
    new_image[:,width//2:]= cv2.cvtColor(edge,cv2.COLOR_GRAY2RGB)
    cv2.imshow("Img",new_image)
    cv2.waitKey(1) 
    rospy.loginfo("Subscribing video")
if __name__ == '__main__':

    rospy.init_node("image_subscriber",anonymous=True)
    sub = rospy.Subscriber("image_topic",Image,callback=Image_callback)
    rospy.loginfo("Started subscribing video")
    rospy.spin()
    cv2.destroyAllWindows()
    
        
