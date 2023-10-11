#!/usr/bin/env python3
import rospy
import rospy
import numpy as np
import cv2
import time
import math
from vision.srv import message,messageResponse
from vision.srv import route,routeResponse
from vision.msg import image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs import Pose

class Server:

	def __init__(self):
		rospy.init_node("server_node")
		service = rospy.Service("aruco",message,self.callback)
		service2=rospy.Service("director",route,self.callback2)
		self.bridge = CvBridge()
		rospy.spin()
		

	def aruco_display(self,corners, ids, rejected, image):
		
		if len(corners) > 0 :
		#if ids is not None:	
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
	
	def callback2(self,request):
		initial_coordinate_x,initial_coordinate_y = request.initial_pose.position.x,request.initial_pose.position.y
		index=self.ros_ids.index(request.id)
		coordinates=list(self.corners[index][0],self.corners[index][1],self.corners[index][2],self.corners[index][3])
		final_coordinate_x1,final_coordinate_y1=coordinates[0] 
		final_coordinate_x2,final_coordinate_y2=coordinates[1]
		final_coordinate_x3,final_coordinate_y3=coordinates[2]
		final_coordinate_x4,final_coordinate_y4=coordinates[3]

		final_coordinate_x=(final_coordinate_x1+final_coordinate_x2+final_coordinate_x3+final_coordinate_x4)/4
		final_coordinate_y=(final_coordinate_y1+final_coordinate_y2+final_coordinate_y3+final_coordinate_y4)/4
		final_angle= np.arctan(final_coordinate_y-initial_coordinate_y)/(final_coordinate_x-initial_coordinate_x)
		initial_angle = 2*(math.asin(request.orientation.initial_pose.w))
		angle=final_angle-initial_angle
		distance = pow(pow((initial_coordinate_x-final_coordinate_x),2)+pow((initial_coordinate_y-final_coordinate_y),2),0.5)

		response = messageResponse(distance=distance,angle=angle)

		return response


	def callback(self,request):
		
		
		cv_image = self.bridge.imgmsg_to_cv2(request.aruco)

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

			#arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
		arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

		arucoParams = cv2.aruco.DetectorParameters()

		self.corners,ids, rejected = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)
		
		if ids is None:
			ids = []
		ids=np.array(ids, dtype=np.float32)
		ids = tuple(ids.flatten().tolist())

		corners_flattened=np.array(self.corners)
		corners_flattened=corners_flattened.flatten()
		corners_flattened=corners_flattened.tolist()
		print(corners_flattened)
		#print(ids)
		# print(rejected)
		dim=[MultiArrayDimension() for i in range(3)]
		dim[0].size = len(ids)
		dim[1].size = 4
		dim[2].size = 2
		x = MultiArrayLayout(dim=dim,data_offset=0)
		ros_corners = Float32MultiArray(data=corners_flattened, layout=x)
		dim2=[MultiArrayDimension() for i in range(1)]
		dim2[0].size=1
		y=MultiArrayLayout(dim=dim2,data_offset=0)
		#print(type(ids))
		
		#ids=np.array(ids)
		#print(ids)
		#ids = tuple(ids.flatten().tolist())
		print(ids)
		#print(type(ids))

		# ids = []
		#response = messageResponse(corners = ros_corners, ids = ids)
		self.ros_ids=Float32MultiArray(data=ids,layout=y)
		response = messageResponse(corners = ros_corners, ids=self.ros_ids)
		print(response)
			#detected_markers = aruco_display(corners, ids, rejected, cv_image)
		#detected_markers = aruco_display(corners, ids, rejected, cv_image)
		#cv2.imshow("Image", detected_markers)
		#cv2.waitKey(1)
		return response


		   
	# def server():
	# 	rospy.init_node("server_node")
	# 	service = rospy.Service("aruco",message,callback)
		
	# 	rospy.spin()



if __name__ == '__main__':

	
	server = Server()


	
