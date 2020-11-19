#!/usr/bin/python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def find_poster(im):
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) 
  
    # Find Canny edges 
    edged = cv2.Canny(gray,150,200) 
    cv2.waitKey(0) 
    _, contours, hierarchy = cv2.findContours(edged,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
    
    # cv2.imshow('Canny Edges After Contouring', edged) # This line is useful for tuning the Canny edges. Shows the edges that are being detected
    
    xy_min = np.amin(contours[-1], axis=0) # Finds the minimum x and y combination in contours[-1] which is the poster edge for the star poster.
    xy_max = np.amax(contours[-1], axis=0) # Finds the maximum x and y combination in contours[-1] "

    # print xy_min, xy_max

    im_cropped = im[xy_min[0][1]:xy_max[0][1], xy_min[0][0]:xy_max[0][0]] # This crops the image
    
    # cv2.imshow('Cropped', im_cropped) # Shows image after cropping
    # cv2.waitKey(0) # Wait until enter is pressed in window

    return im_cropped

def draw_lines(im):
    points = []

    # im = cv2.imread("car.jpg")
    gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, bin = cv2.threshold(gray,100,255,1) # inverted threshold (light obj on dark bg)
    bin = cv2.dilate(bin, None)  # fill some holes
    bin = cv2.dilate(bin, None)
    bin = cv2.erode(bin, None)   # dilate made our shape larger, revert that
    bin = cv2.erode(bin, None)
    cv2.imshow("grey", bin)
    bin, contours, hierarchy = cv2.findContours(bin,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    for i in range(len(contours)):
        # print contours[i]
        M = cv2.moments(contours[i])
        if M["m00"]:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # cv2.drawContours(im, contours[i], -1, (0, 255, 0), 2)
            # cv2.circle(im, (cX, cY), 7, (200, 0, 0), -1)
        points.append((cX,cY))
    print len(points)
    for k in range(len(points)):
        if (k<len(points)-1):
            cv2.line(im, points[k], points[k+1], (0,0,255), 3)
        print("Point {}: {}".format(k, points[k]))
        # print(cX[i],cY[i])

    return im

bridge = CvBridge()

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

class ros_to_cv2:
    rx_flag = False
    rx_buffer = None
    def callback(self, img_msg):
        # rospy.loginfo(img_msg.header)
        try:
            # print("DATA RECEIVED")
            self.rx_flag = True
            self.rx_buffer = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == '__main__':
    rospy.init_node('image_disp_feed', anonymous=True, disable_signals=True)
    rate = rospy.Rate(1) # Check for new data at 1 Hz
    processor = ros_to_cv2()
    sub_image = rospy.Subscriber("/ur10/camera1/image_raw", Image, processor.callback)
    display_only = False
    final_img = None
    while not rospy.is_shutdown():
        # print("LOOPING")
        if processor.rx_flag and not display_only:
            wall_pic = processor.rx_buffer
            # sub_image.shutdown()
            crop = find_poster(wall_pic)
            final_img = draw_lines(crop)
            display_only = True
        if display_only:
            # show_image(crop)
            show_image(final_img)

        rate.sleep()