import rospy
import numpy as np
import cv2
import time
from vision.srv import message,messageResponse
from vision.msg import image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def aruco_display(corners, ids, rejected, image):
	if len(corners) > 0:
		
		ids = ids.flatten()
		
		
		
		for (markerCorner, markerID) in zip(corners, ids):
			
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))
			
			coordinates = list(topRight,bottomRight,bottomLeft,topLeft )
			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
				
			#print("[Inference] ArUco marker ID: {}".format(markerID))
	
			
	return image



def callback(request):
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(request)

	ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
	}

	aruco_type = "DICT_4x4_100"

	arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])

	arucoParams = cv2.aruco.DetectorParameters_create()
	

	corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)
	
	detected_markers = aruco_display(corners, ids, rejected, cv_image)
	cv2.imshow("Image", detected_markers)
	cv2.waitKey(1)
	return messageResponse(corners,ids)


		   
def server():
	rospy.init_node("server_node")
	service = rospy.Service("aruco",message,callback)
	
	rospy.spin()



if __name__ == '__main__':

	
	server()


	
