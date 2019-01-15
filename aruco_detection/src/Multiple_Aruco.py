#!/usr/bin/env python

import roslib
import sys
import rospy
#import freenect
import cv2,math
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from aruco_detection.msg import Aruco
from geometry_msgs.msg import Twist,Pose
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

marker_size =25#24.6#16.4#22.5#15.8 #22.6#15.8 #cm
#id_to_find=2
#-- Font for the text in the image
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0
font = cv2.FONT_HERSHEY_PLAIN
camera_size=[640,480]

class camera(object):
	def __init__(self):
		self.bridge_object = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
		self.image_pub=rospy.Publisher("/ArUco_Marker", Aruco, queue_size=10)
		self.pos=Aruco()

	def camera_callback(self,data):
		try:
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
			ret=True
		except CvBridgeError as e:
			print(e)

		self.pos=Aruco()

		#Use the calibrated camera matrix and Distortion matrix from the program Calibration.py and Snapshot.py
		#camera_matrix=np.loadtxt("/home/shrijan00/D415 RS/cameraMatrix.txt",delimiter=',')
		camera_matrix=np.matrix('1064.316652 0.000000 959.561358; 0.000000 1069.207142 536.476647; 0.0 0.0 1.0')

		#camera_distortion=np.loadtxt("/home/shrijan00/D415 RS/distortion.txt",delimiter=',')
		#camera_distortion=np.matrix('-0.000051 -0.000465 0.000576 0.000168 0.0000001')
		camera_distortion=np.matrix('0.00000001 0.00000001 0.00000001 0.00000001 0.00000001')
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000) #DICT_ARUCO_ORIGINAL)#(aruco.DICT_7X7_250)

		parameters = aruco.DetectorParameters_create()

		self.corners, self.ids, self.rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters, cameraMatrix=camera_matrix, distCoeff=camera_distortion)
		#self.pos.marker_detected=np.array(self.ids)
		#print("Corner position are",self.corners)
		if np.all(self.ids != None):
			cv_image =  aruco.drawDetectedMarkers(cv_image, self.corners, self.ids, (0,255,0))
			estimate=aruco.estimatePoseSingleMarkers(self.corners, marker_size, camera_matrix, camera_distortion)
			
			self.rvecs = estimate[0]
			self.tvecs = estimate[1]
			#print('rotation vector' ,self.rvecs)
			pt =np.array([[-23, -20, 0]], dtype=np.float)
			self.pos.header.stamp=rospy.Time.now()
			self.pos.header.frame_id="camera_link"
			self.pos.child_frame_id="ArUco_Marker"
			i=0
			#for rvec, tvec in zip(self.rvecs, self.tvecs):
			for i in range(0, self.ids.size):
				temp_pose = Pose()
				#print("Rotation vector is ",self.rvecs[i])
				cv_image=aruco.drawAxis(cv_image, camera_matrix, camera_distortion, self.rvecs[i], self.tvecs[i], 10)
				#str_position = "trans"%((self.tvecs[i][0]))
				imgpts,_ = cv2.projectPoints(pt, self.rvecs[i], self.tvecs[i], camera_matrix, camera_distortion)
				#print("Image points",imgpts[0][0])

				#str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(self.tvecs[i][0][0], self.tvecs[i][0][1], self.tvecs[i][0][2])
				str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(self.tvecs[i][0][2], self.tvecs[i][0][0], self.tvecs[i][0][1])
				cv2.putText(cv_image, str_position, (int(imgpts[0][0][0]),int(imgpts[0][0][1])), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
				#cv2.putText(cv_image, "{:.2f}".format(str_position), (0,100), font, 4,(255,255,255), 2, cv2.LINE_AA)
				self.pos.marker_id.append( self.ids[i] )
				#self.pos.pose.position.x=self.tvecs[i][0][0]/100
				#self.pos.pose.position.y=self.tvecs[i][0][1]/100
				#self.pos.pose.position.z=self.tvecs[i][0][2]/100
				
				#self.pos.pose.position.x=self.tvecs[i][0][2]/100
				#self.pos.pose.position.y=-self.tvecs[i][0][0]/100
				#self.pos.pose.position.z=-self.tvecs[i][0][1]/100
				temp_pose.position.x=self.tvecs[i][0][2]/100
				temp_pose.position.y=-self.tvecs[i][0][0]/100
				temp_pose.position.z=-self.tvecs[i][0][1]/100

				#self.rotation=np.array([self.rvecs[i][0][0], self.rvecs[i][0][1], self.rvecs[i][0][2]])
				self.rotation=np.array([self.rvecs[i][0][2], self.rvecs[i][0][0], self.rvecs[i][0][1]])
				#-- Obtain the rotation matrix tag->camera
				#print ("orientation is ",orientation)
                		R_ct  = np.matrix(cv2.Rodrigues(self.rotation)[0])
                		R_tc=R_ct.T
                		roll, pitch, yaw = self.rotationMatrixToEulerAngles(R_flip*R_tc)
                		(q0,q1,q2,q3)=quaternion_from_euler(roll,pitch,yaw)

                		#self.pos.pose.orientation.x=q0
                		#self.pos.pose.orientation.y=q1
                		#self.pos.pose.orientation.z=q2
                		#self.pos.pose.orientation.w=q3
                		temp_pose.orientation.x=q0
                		temp_pose.orientation.y=q1
                		temp_pose.orientation.z=q2
                		temp_pose.orientation.w=q3
                		self.pos.poses.append( temp_pose )

			#print(i)
		cv2.imshow("Image window", cv_image)
		self.image_pub.publish(self.pos)
		cv2.waitKey(1)



#Refer to this https://www.learnopencv.com/rotation-matrix-to-euler-angles/
	def rotationMatrixToEulerAngles(self,R) :
		def isRotationMatrix(self,R):
			Rt = np.transpose(R)
			shouldBeIdentity = np.dot(Rt, R)
			I = np.identity(3, dtype=R.dtype)
			self.n = np.linalg.norm(I - shouldBeIdentity)
			return self.n < 1e-6


		assert (isRotationMatrix(self,R))
		sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
		singular = sy < 1e-6
		if not singular :
			x = math.atan2(R[2, 1], R[2, 2])
			y = math.atan2(-R[2, 0], sy)
			z = math.atan2(R[1, 0], R[0, 0])
		else:
			x = math.atan2(-R[1, 2], R[1, 1])
			y = math.atan2(-R[2, 0], sy)
			z=0
		return np.array([x, y, z])




def main():
	rospy.init_node('Kinect', anonymous=True)
	rs_object=camera()
	
	#rate=rospy.Rate(5)
	#print("Hello")
	try:
    		rospy.sleep(0)
		rospy.spin()
	except KeyboardInterrupt:
		print("Shut Down Time")
	cv2.destroyAllWindows()



if __name__ == '__main__':

	main()
	
