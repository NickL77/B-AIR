#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

#Define the method which contains the main functionality of the node.
def controller(goal_offset):
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - target_frame: the tf frame of the target AR tag
  """
  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      # Process trans to get your state error
      # Generate a control command to send to the robot
      
      linear = Vector3()
      rotation = Vector3()
      
      # c = some constant velocity
      # l = distance from center of robot to fan
      # x_balloon = goal_offset[0] # x postion of balloon in camera frame
      # y_balloon = goal_offset[1] # y position of balloon in camera frame
      # # translate the position of balloon in camera frame to position of the balloon in fan frame
      # total = x_balloon + y_balloon
      # v = x_balloon / total
      # w = y_balloon / total / l


      linear.x = 1
      linear.y = 0
      linear.z = 0
      rotation.x = 0
      rotation.y = 0
      rotation.z = 1

      control_command = Twist()
      control_command.linear = linear
      control_command.angular = rotation
      #################################### end your code ###############

      pub.publish(control_command)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      print(e)
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_controller', anonymous=True)

  try:
    controller((1, -2))
  except rospy.ROSInterruptException:
    pass
