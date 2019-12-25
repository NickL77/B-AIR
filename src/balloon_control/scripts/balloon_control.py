#!/usr/bin/env python 
import rospy 
import tf2_ros 
import sys
import math
import numpy as np

from collections import deque
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Point 
from geometry_msgs.msg import Vector3


class Controller():

    def __init__(self):
        rospy.init_node('balloon_pos')
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/balloon_tracker/location', Point, self.callback)
        
        # Robot parameters
        self.l = 50 # robot to fan 
        self.x_deadzone = 175 # deadzone where we stop
        self.y_deadzone = 100

        # PID parameters
        self.x_kp = -2.0 / 100 / 6
        self.y_kp = 2.0 / 2
        self.x_kd = 1.5
        self.y_kd = 0.2

        # Ring_buffer data structure to average previous values for smoothing
        self.prev_x = -1
        self.prev_y = -1
        self.ring_buffer_y = deque([], 5);
        self.ring_buffer_x = deque([], 5);

        self.found = False

    def callback(self, pos):
  
        try:
    
            # translate the position of balloon in camera frame to position of the balloon in fan frame
            x = pos.x - self.l
            y = pos.y
           
            # calculate deltas needed to move
            cdx = x - self.prev_x
            cdy = y - self.prev_y
            
            # Calculations for D controller			
            if self.prev_x > 0 and self.prev_y > 0:
                    dx = np.mean(self.ring_buffer_x) 
                    dy = np.mean(self.ring_buffer_y)
            else:
                    dx = 0
                    dy = 0
            self.prev_x = x
            self.prev_y = y
            self.ring_buffer_x.append(cdx)
            self.ring_buffer_y.append(cdy)

            # calculate linear and angular velocity to go in the right direction
            # if balloon is far out (not in deadzone)
            if abs(x) < self.x_deadzone and abs(y) < self.y_deadzone:
                    # print('deadzone')
                    v = 0
                    th = 0
            else:			
                    v = x * self.x_kp + dx * self.x_kd
                    th = (y / self.l) * self.y_kp + dy * self.y_kd

            twist = Twist()
            twist.linear.x = v; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
            self.pub.publish(twist)
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            pass

if __name__ == '__main__':
	controller = Controller()
  	rospy.spin()

