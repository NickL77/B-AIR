#!/usr/bin/env python 
#The line above tells Linux that this file is a Python script, and that the OS should 
#use the Python interpreter in /usr/bin/env to run it. Don't forget to use "chmod +x [filename]" to make 
#this script executable.

#Import the rospy package. For an import to work, it must be specified in both the package manifest AND 
#the Python file in which it is used.
import rospy 
import tf2_ros 
import sys
import math

from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Point 
from geometry_msgs.msg import Vector3

#robot parameters
speed = 10 # speed 
turn = 50 # turn 
l = 10 # robot to fan 
deadzone = 5# deadzone where we stop

control_speed = 0 
control_turn = 0

#Define the method which contains the main functionality of the node.
def controller(pos):
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - target_frame: the tf frame of the target AR tag
  """
  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
  # TODO: check this ^
  




  # Create a timer object that will sleep long enough to result in a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  try:
    # Process trans to get your state error Generate a control command to send to the robot
    
    twist = Twist()
    
    # c = some constant velocity l = distance from center of robot to fan x_balloon = goal_offset[0] # x 
    # postion of balloon in camera frame y_balloon = goal_offset[1] # y position of balloon in camera 
    # frame
    # # translate the position of balloon in camera frame to position of the balloon in fan frame
    # total = x_balloon + y_balloon v = x_balloon / total w = y_balloon / total / l
    
  
    x = pos.x + l
    y = pos.y
    #if (x**2 + y**2 < deadzone):
    #  v = 0
    #  th = 0
    # else:
    v = -1 * x / 100
    th = (y / l) / 2
    print(x)
    print(y)

    if abs(x) < 15 and abs(y) < 15:
	v = 0
	th = 0


    # print(v) print(th)

    target_speed = v * speed
    target_turn = th * turn

    twist.linear.x = v; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
    pub.publish(twist)
    
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    print(e)
    pass
  # Use our rate object to sleep until it is time to publish again

      
# This is Python's sytax for a main() method, which is run by default when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down If not, run the talker method

  #Run this program as a new node in the ROS computation graph called /turtlebot_controller.
  rospy.init_node('balloon_pos')

  # SUBSCRIBER FOR BALLOON MSG
  sub = rospy.Subscriber('/balloon_tracker/location', Point, controller)
  rospy.spin()
  # while True:
    # goal_offset = raw_input("linx, rotz") print(goal_offset) values_array = [float(i) for i in 
    # goal_offset.split(" ")] print(values_array) try:
    #   controller(values_array) except rospy.ROSInterruptException: pass

