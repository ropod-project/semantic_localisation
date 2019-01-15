#!/usr/bin/env python

import roslib
import sys
import freenect
import rospy
import cv2,math
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist,Pose
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf


#markerLength = 100
marker_size = 24.6#15.8 #cm
id_to_find=2
#-- Font for the text in the image
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0
font = cv2.FONT_HERSHEY_PLAIN
camera_size=[640,480]
#markerSeparation = 8
class camera(object):



	def __init__(self):
		self.bridge_object = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
		self.image_pub=rospy.Publisher("/ArUco_Marker_Id", Pose, queue_size=1)

		self.pos=Pose()


	def camera_callback(self,data):

		try:
			# We select bgr8 because its the OpneCV encoding by default
			#br = tf.TransformBroadcaster()


			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

			ret=True
		except CvBridgeError as e:
			print(e)

				#hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		self.pos=Pose()
		
		#camera_matrix=np.loadtxt("/home/shrijan00/camera_matrix.txt",delimiter=',')
		camera_matrix=np.matrix('529.9725194672609 0 312.6165646707006; 0 540.7277466388451 265.3383268607133; 0 0 1')
		#camera_distortion=np.loadtxt("/home/shrijan00/distortion_matrix.txt",delimiter=',')
		camera_distortion=np.matrix('0.2087167506355359 -0.3669474642386585 -0.009072893445966169 -0.004643909287421034 0')
		#Conver the image into gray scale	
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		#Define the dictionary of ArUco markers
		
		aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_250)
		#aruco_dict = aruco.Dictionary_get( aruco.DICT_5X5_250)   aruco.DICT_ARUCO_ORIGINAL
		#board = aruco.GridBoard_create(5, 7, markerLength, markerSeparation, aruco_dict)
		parameters = aruco.DetectorParameters_create()
		#print("Parameters of the ArUco Markers",parameters)
		#lists of ids and the corners beloning to each id
		corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters, cameraMatrix=camera_matrix, distCoeff=camera_distortion)

		#aruco.refineDetectedMarkers(gray, board, corners, ids, rejected)


		#print("Success")
		if np.all(ids != None):
		#if ids != None and ids[0] == id_to_find:
			cv_image =  aruco.drawDetectedMarkers(cv_image, corners, ids, (0,255,0))

			
			#print("Welc")
			#rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, camera_distortion)
			#rvecs, tvecs, _ = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, camera_distortion)
			#retval, rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, camera_distortion)
			ret=aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
			rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
			cv_image=aruco.drawAxis(cv_image, camera_matrix, camera_distortion, rvec, tvec, 10)

			#-- Print the tag position in camera frame
			str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
			cv2.putText(cv_image, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
			self.pos.position.x=tvec[0]/100
			self.pos.position.y=tvec[1]/100
			self.pos.position.z=tvec[2]/100
			#-- Obtain the rotation matrix tag->camera
			R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
			R_tc    = R_ct.T

			#roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
			R=R_flip*R_tc
			sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
			singular = sy < 1e-6
			if not singular:
				x = math.atan2(R[2, 1], R[2, 2])
				y = math.atan2(-R[2, 0], sy)
				z = math.atan2(R[1, 0], R[0, 0])
			else:
				x = math.atan2(-R[1, 2], R[1, 1])
				y = math.atan2(-R[2, 0], sy)
				z=0
			roll=x
			pitch=y
			yaw=z
			
			(q0,q1,q2,q3)=quaternion_from_euler(roll,pitch,yaw)
			self.pos.orientation.x=q0
			self.pos.orientation.y=q1
			self.pos.orientation.z=q2
			self.pos.orientation.w=q3
			self.image_pub.publish(self.pos)

			str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll),math.degrees(pitch),math.degrees(yaw))
			cv2.putText(cv_image, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

			#Rotation defined for the flipped frame

			#rotation=rotationMatrixToEulerAngles(R_tc*R)


			#for i in range(len(ids)):
				#id=int(ids[i][0])
				
				#translation=tvec[i][0]
				#rotation = (180/math.pi)*rvec[i][0]
				#camera_distance = math.sqrt(translation[0]*translation[0] +

				#euler_rotation=np.array([x, y,z])
				#print("Euler Angles are",rotation)

				


		  # if there is at least one marker detected
			#print("Halleuja",retval)
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(1)
			 	



def main():
	rospy.init_node('intel_realsense', anonymous=True)
	rs_object=camera()
	
	rate=rospy.Rate(1)
	#print("Hello")
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shut Down Time")
	cv2.destroyAllWindows()



if __name__ == '__main__':

	main()
	


