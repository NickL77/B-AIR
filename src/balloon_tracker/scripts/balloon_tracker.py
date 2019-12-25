#!/usr/bin/env python
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import rospy
import math

class BalloonTracker:

    def __init__(self):
       
        # Initialize Realsense Subscribers
        # image_raw: raw bgr image to detect green balloon
        # image_rect_raw: depth image to get distance of balloon
        # camera_info: used to get image dimensions for calculations
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        
        self.pub = rospy.Publisher('/balloon_tracker/location', Point, queue_size=10)
        
        # CvBridge is used to convert between ROS images and openCV frames
        self.bridge = CvBridge()

        # Upper and lower bounds for RGB filtering
        self.lower_green = np.array([0, 100, 0])
        self.upper_green = np.array([85, 220, 85])

        # Initialize variables, mainly for trig calculations to get
        # X and Y baloon offsets
        self.balloon_pos = (-1, -1)
        self.width = -1
        self.height = -1
        self.focal_length = 1.93
        self.h_fov = math.radians(91.2)
        self.v_fov = math.radians(65.5)
        self.h_image_plane = 2 * self.focal_length * math.tan(self.h_fov / 2.0)
        self.v_image_plane = 2 * self.focal_length * math.tan(self.v_fov / 2.0)

    # Runs RGB contour filtering on color image. Sets self.balloon_pos to the X, Y
    # position of the center of the balloon in the image
    def color_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        blur = cv2.GaussianBlur(cv_image, (5, 5), 0)
        mask = cv2.inRange(blur, self.lower_green, self.upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Check if contours found and if largest contour passes size threshold
        if len(contours) > 0 and max([cv2.contourArea(c) for c in contours]) > 100:
            
            c = max(contours, key=cv2.contourArea)
            
            M = cv2.moments(c)
            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])
            self.balloon_pos = (cX, cY)

            # Drawing contour and center of contour for visualizations
            cv2.circle(cv_image, (cX, cY), 7, (255, 0, 0), -1)
            cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)

        else:

            self.balloon_pos = (-1, -1)

        cv2.imshow('frame', cv_image)
        cv2.waitKey(1)

    # Gets euclidian distance between camera and the point in the center of the
    # balloon, then uses trigonometry and pinhole camera model to calculate angle
    # offsets and then global X, Y offsets. Then publishes offsets
    def depth_callback(self, data):
        cv_depth = self.bridge.imgmsg_to_cv2(data)

        if self.balloon_pos[0] > -1 and self.balloon_pos[1] > -1 and self.width > -1 and self.height > -1:
            blur = cv2.GaussianBlur(cv_depth, (15, 15), 0)
            j = self.balloon_pos[0]
            hyp = cv_depth[self.balloon_pos[1]][self.balloon_pos[0]]
            # remove noisy measurements, 2000+
            # also cap error measurement so PID controller is limited
            hyp = min(600, hyp) 

            h_pos = self.balloon_pos[0] * self.h_image_plane / self.width
            v_pos = self.balloon_pos[1] * self.v_image_plane / self.height

            sensor_h_pos = (self.h_image_plane / 2) - h_pos
            sensor_v_pos = (self.v_image_plane / 2) - v_pos

            x_angle = math.atan(sensor_h_pos / self.focal_length)
            y_angle = math.atan(sensor_v_pos / self.focal_length)

            x_offset = math.sin(x_angle) * hyp
            y_offset = math.sin(y_angle) * hyp

            p = Point()
            p.x = x_offset
            p.y = y_offset
            p.z = z_offset = -0.69

            self.pub.publish(p)

    def camera_info_callback(self, data):
        self.width = data.width
        self.height = data.height

if __name__ == '__main__':
    balloon_tracker = BalloonTracker()
    rospy.init_node('balloon_tracker')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down')
