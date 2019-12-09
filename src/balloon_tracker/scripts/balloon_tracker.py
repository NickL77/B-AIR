#!/usr/bin/env python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import rospy

class BalloonTracker:

    def __init__(self):
        
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        
        self.bridge = CvBridge()

        self.lower_green = np.array([0, 70, 0])
        self.upper_green = np.array([70, 255, 70])

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        blur = cv2.GaussianBlur(cv_image, (5, 5), 0)
        
        mask = cv2.inRange(blur, self.lower_green, self.upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0 and max([cv2.contourArea(c) for c in contours]) > 100:
            
            c = max(contours, key=cv2.contourArea)
            
            M = cv2.moments(c)
            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])

            cv2.circle(cv_image, (cX, cY), 7, (255, 0, 0), -1)
            cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)

        cv2.imshow('mask', mask)
        cv2.imshow('frame', cv_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    balloon_tracker = BalloonTracker()
    rospy.init_node('balloon_tracker', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down')
