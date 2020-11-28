#!/usr/bin/python
from __future__ import division

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from kinematics import *


def find_poster(im):
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) 
  
    # Find Canny edges 
    edged = cv2.Canny(gray,150,200) 
    # cv2.waitKey(0) 
    _, contours, hierarchy = cv2.findContours(edged,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
    
    # cv2.imshow('Canny Edges After Contouring', edged) # This line is useful for tuning the Canny edges. Shows the edges that are being detected
    
    xy_min = np.amin(contours[-1], axis=0) # Finds the minimum x and y combination in contours[-1] which is the poster edge for the star poster.
    xy_max = np.amax(contours[-1], axis=0) # Finds the maximum x and y combination in contours[-1] "

    # print xy_min, xy_max

    im_cropped = im[xy_min[0][1]:xy_max[0][1], xy_min[0][0]:xy_max[0][0]] # This crops the image [y_min:y_max, x_min:x_max]

    # Flip image
    im_flipped = cv2.flip(im_cropped, 1)
    
    cv2.imshow('Cropped', im_cropped) # Shows image after cropping
    cv2.waitKey(0) # Wait until enter is pressed in window
    return im_flipped, xy_min, xy_max

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
    # print len(points)
    for k in range(len(points)):
        if (k<len(points)-1):
            cv2.line(im, points[k], points[k+1], (0,0,255), 3)
        # print("Point {}: {}".format(k, points[k]))
        # print(cX[i],cY[i])

    return im, points

bridge = CvBridge()

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def convert_point(points, xy_min, xy_max):
    x_points = []
    y_points = []

    poster_width = 1 # Poster width in meters, change accordingly
    poster_height = 1 # Poster height in meters, change accordingly
    
    x_pixels = xy_max[0][0] - xy_min[0][0]
    y_pixels = xy_max[0][1] - xy_min[0][1]

    x_length = poster_width / x_pixels # This will convert the # of pixels to # of pixels per meter
    y_length = poster_height / y_pixels # change the value of '1' to match the 

    x_center = x_pixels / 2 # find horizontal center point of poster 
    y_center = y_pixels / 2 # find vertical center of poster
    
    # print points
    for k in range(len(points)):
        if (k<len(points)):
            x_points.append((x_center - points[k][0]) * x_length) # X pixels are left to right, treat center as "0" and find distance relative to "0", left being negative right being positive
            y_points.append((y_center - points[k][1]) * y_length) # Y pixels are top to bottom, treat center as "0" and find distance relative to "0", down being negative up being positive
        # print("Point {}: {}".format(k, x_points[k]))
    return x_points, y_points

def move_arm(x_points, y_points):
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=50)
    rate = rospy.Rate(1)
    
    Tmsg = JointTrajectory()
    Tmsg.joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint', 'wrist_3_joint']
    Tmsg.header.stamp = rospy.Time.now()

    Tpointmsg = JointTrajectoryPoint()
    Tpointmsg.velocities = [0,0,0,0,0,0]
    Tpointmsg.time_from_start = rospy.Duration(0.5)
    print("NUMBER OF POINTS: ", len(points))

    for k in range(len(points)):
        if (k<len(points)):
            # desired_solution = [np.pi,-np.pi/2,-np.pi/2,-np.pi/2, 0, 0]
            desired_solution = [0,0,0,-np.pi/2, 0, 0]
            target_pose = [x_points[k],0.5, y_points[k]+0.8, -np.pi/2, 0, 0]
            # target_pose = [0, 0.5, 0.8, -np.pi/2, 0, 0]
            Tmsg.header.stamp = rospy.Time.now()
            print("TARGET POSE", target_pose)
            Tpointmsg.positions = inv_kin(target_pose, desired_solution)
            # print inv_kin(target_pose, desired_solution)
            for i in range(len(Tpointmsg.positions)):
                # print Tpointmsg.positions[i]
                while (abs(Tpointmsg.positions[i]) >= np.pi):
                    # print Tpointmsg.positions[i]
                    if Tpointmsg.positions[i] < -np.pi:
                        # print "<"
                        Tpointmsg.positions[i] = np.pi
                    elif Tpointmsg.positions[i] > np.pi:
                        # print ">"
                        Tpointmsg.positions[i] =- np.pi
                    elif abs(Tpointmsg.positions[i]) == np.pi:
                        # print "="
                        Tpointmsg.positions[i] = 0
            # print Tpointmsg.positions
            Tpointmsg.positions = [round(num, 1) for num in Tpointmsg.positions]
            Tmsg.points = [Tpointmsg]
            pub.publish(Tmsg)

            # print "publishing"
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joint_states.position = [round(num, 2) for num in joint_states.position]
            joint_states.position[0], joint_states.position[2] = joint_states.position[2], joint_states.position[0]
            while (joint_states.position != Tpointmsg.positions):
                # print joint_states.position
                # print Tpointmsg.positions
                Tmsg.header.stamp = rospy.Time.now()
                joint_states = rospy.wait_for_message("joint_states", JointState)
                joint_states.position = [round(num, 2) for num in joint_states.position]
                joint_states.position[0], joint_states.position[2] = joint_states.position[2], joint_states.position[0]
            rospy.sleep(0.5)

def go_home():
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(25)
    
    Tmsg = JointTrajectory()
    # Tmsg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint', 'wrist_3_joint']
    Tmsg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint', 'wrist_3_joint']
    Tmsg.header.stamp = rospy.Time.now()

    Tpointmsg = JointTrajectoryPoint()
    Tpointmsg.velocities = [0,0,0,0,0,0]
    Tpointmsg.time_from_start = rospy.Duration(0.5)

    Tpointmsg.positions = [-np.pi/2,-np.pi/2,np.pi/2,0,0,0]
    Tmsg.points = [Tpointmsg]
    # print "publishing"
    joint_states = rospy.wait_for_message("joint_states", JointState)
    joint_states.position = [round(num, 3) for num in joint_states.position]
    while (joint_states.position[0:3] != [round(np.pi/2,3),round(-np.pi/2,3),round(-np.pi/2,3)]):
        # print joint_states.position[0:3]
        pub.publish(Tmsg)
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joint_states.position = [round(num, 3) for num in joint_states.position]
    rospy.sleep(1)

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
    rospy.init_node('cd_node', anonymous=True, disable_signals=True)
    rate = rospy.Rate(1) # Check for new data at 1 Hz
    processor = ros_to_cv2()
    sub_image = rospy.Subscriber("/ur10/camera1/image_raw", Image, processor.callback)
    display_only = False
    final_img = None
    
    go_home()

    while not rospy.is_shutdown():
        # print("LOOPING")
        if processor.rx_flag and not display_only:
            wall_pic = processor.rx_buffer
            # sub_image.shutdown()
            crop, xy_min, xy_max = find_poster(wall_pic)
            final_img, points = draw_lines(crop)
            x_points, y_points = convert_point(points, xy_min, xy_max)
            move_arm(x_points, y_points)
            display_only = True
        if display_only:
            # show_image(crop)
            show_image(final_img)

        rate.sleep()