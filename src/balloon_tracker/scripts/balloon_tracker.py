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
        
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)

        self.pub = rospy.Publisher('/balloon_tracker/location', Point, queue_size=10)

        self.bridge = CvBridge()

        self.lower_green = np.array([0, 70, 0])
        self.upper_green = np.array([70, 255, 70])

        self.balloon_pos = (-1, -1)
        self.width = -1
        self.height = -1
        self.focal_length = 1.93
        self.h_fov = math.radians(91.2)
        self.v_fov = math.radians(65.5)
        self.h_image_plane = 2 * self.focal_length * math.tan(self.h_fov / 2.0)
        self.v_image_plane = 2 * self.focal_length * math.tan(self.v_fov / 2.0)

    def depth_callback(self, data):
        cv_depth = self.bridge.imgmsg_to_cv2(data)
        color = cv2.cvtColor(cv_depth, cv2.COLOR_GRAY2RGB)

        if self.balloon_pos[0] > -1 and self.balloon_pos[1] > -1 and self.width > -1 and self.height > -1:
            
            hyp = cv_depth[self.balloon_pos[1]][self.balloon_pos[0]]

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

            print(x_offset, y_offset)
            
            #print(len(data.data), len(data.data[0]), cv_depth[self.balloon_pos[1]][self.balloon_pos[0]])
            #cv2.circle(color, (self.balloon_pos[0], self.balloon_pos[1]), 30, (0, 255, 0), -1)
        
        # cv2.imshow('depth', color)
        # cv2.waitKey(1)


    def color_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        blur = cv2.GaussianBlur(cv_image, (5, 5), 0)
        
        mask = cv2.inRange(blur, self.lower_green, self.upper_green)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0 and max([cv2.contourArea(c) for c in contours]) > 100:
            
            c = max(contours, key=cv2.contourArea)
            
            M = cv2.moments(c)
            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])
            self.balloon_pos = (cX, cY)

            cv2.circle(cv_image, (cX, cY), 7, (255, 0, 0), -1)
            cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)

        else:

            self.balloon_pos = (-1, -1)

        cv2.imshow('frame', cv_image)
        cv2.waitKey(1)

    def camera_info_callback(self, data):
        self.width = data.width
        self.height = data.height

if __name__ == '__main__':
    balloon_tracker = BalloonTracker()
    rospy.init_node('balloon_tracker', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down')
