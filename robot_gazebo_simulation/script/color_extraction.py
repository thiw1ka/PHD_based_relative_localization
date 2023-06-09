#!/usr/bin/env python
# Python code for Multiple Color Detection


import numpy as np
import cv2
# from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

class image_converter :    
	def __init__(self):
		self.image_pub = rospy.Publisher("/Robot1/camera_targets",PoseArray,queue_size = 5)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/Robot1/iris/camera/image_raw",Image,self.callback)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		

		hsvFrame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

		# Set range for red color and
		# define mask
		# red_lower = np.array([136, 87, 111], np.uint8)
		# red_upper = np.array([180, 255, 255], np.uint8)
		# red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

		# # Set range for green color and
		# # define mask
		# green_lower = np.array([25, 52, 72], np.uint8)
		# green_upper = np.array([102, 255, 255], np.uint8)
		# green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

		# Set range for blue color and
		# define mask
		blue_lower = np.array([94, 80, 2], np.uint8)
		blue_upper = np.array([120, 255, 255], np.uint8)
		blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

		# Morphological Transform, Dilation
		# for each color and bitwise_and operator
		# between imageFrame and mask determines
		# to detect only that particular color
		kernel = np.ones((5, 5), "uint8")

		# # For red color
		# red_mask = cv2.dilate(red_mask, kernel)
		# res_red = cv2.bitwise_and(cv_image, cv_image,
		# 						mask = red_mask)

		# # For green color
		# green_mask = cv2.dilate(green_mask, kernel)
		# res_green = cv2.bitwise_and(cv_image, cv_image,
		# 							mask = green_mask)

		# For blue color
		blue_mask = cv2.dilate(blue_mask, kernel)
		res_blue = cv2.bitwise_and(cv_image, cv_image,
								mask = blue_mask)

		# # Creating contour to track red color
		# contours, hierarchy = cv2.findContours(red_mask,
		# 									cv2.RETR_TREE,
		# 									cv2.CHAIN_APPROX_SIMPLE)

		# for pic, contour in enumerate(contours):
		# 	area = cv2.contourArea(contour)
		# 	if(area > 300):
		# 		x, y, w, h = cv2.boundingRect(contour)
		# 		cv_image = cv2.rectangle(cv_image, (x, y),
		# 								(x + w, y + h),
		# 								(0, 0, 255), 2)
				
		# 		cv2.putText(cv_image, "Red Colour", (x, y),
		# 					cv2.FONT_HERSHEY_SIMPLEX, 1.0,
		# 					(0, 0, 255))	

		# # Creating contour to track green color
		# contours, hierarchy = cv2.findContours(green_mask,
		# 									cv2.RETR_TREE,
		# 									cv2.CHAIN_APPROX_SIMPLE)

		# for pic, contour in enumerate(contours):
		# 	area = cv2.contourArea(contour)
		# 	if(area > 300):
		# 		x, y, w, h = cv2.boundingRect(contour)
		# 		cv_image = cv2.rectangle(cv_image, (x, y),
		# 								(x + w, y + h),
		# 								(0, 255, 0), 2)
				
		# 		cv2.putText(cv_image, "Green Colour", (x, y),
		# 					cv2.FONT_HERSHEY_SIMPLEX,
		# 					1.0, (0, 255, 0))

		new_poses = PoseArray()
		new_poses.header.frame_id = "Robot1/camera_link";
		new_poses.header.stamp = rospy.Time.now()
		# Creating contour to track blue color
		contours, hierarchy = cv2.findContours(blue_mask,
											cv2.RETR_TREE,
											cv2.CHAIN_APPROX_NONE)[-2:]
		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area > 30):
				x, y, w, h = cv2.boundingRect(contour)
				cv_image = cv2.rectangle(cv_image, (x, y),
										(x + w, y + h),
										(255, 0, 0), 2)
				
				cv2.putText(cv_image, "Blue Colour", (x, y),
							cv2.FONT_HERSHEY_SIMPLEX,
							1.0, (255, 0, 0))
				msg = Pose()
				msg.position.x = x + w/2
				msg.position.y = y + h/2
				# print ("center ",msg)

				new_poses.poses.append(msg)
		# Program Termination
		# cv2.imshow("Multiple Color Detection in Real-TIme", cv_image)
		# if cv2.waitKey(10) & 0xFF == ord('q'):
		# 	cap.release()
		# 	cv2.destroyAllWindows()
		# 	break

		# (rows,cols,channels) = cv_image.shape
		# if cols > 60 and rows > 60 :
		# 	cv2.circle(cv_image, (50,50), 10, 255)

		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)

		try:
			# print("publish")
			self.image_pub.publish(new_poses)
		except CvBridgeError as e:
			print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)