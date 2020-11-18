#!/usr/bin/python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from matplotlib import pyplot as plt

bridge = CvBridge()

def show_image(img):
	cv2.imshow("Image Window", img)
	cv2.waitKey(3)

def callback(img_msg):
	rospy.loginfo(img_msg.header)
	print("This was called")
	try:
		cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
	except CvBridgeError, e:
		rospy.logerr("CvBridge Error: {0}".format(e))

	print("The issue is with show_image")
	show_image(cv_image)

if __name__ == '__main__':
	rospy.init_node('image_disp_feed', anonymous=True)
	sub_image = rospy.Subscriber("/ur10/camera1/image_raw", Image, callback)
	while not rospy.is_shutdown():
		rospy.spin()
