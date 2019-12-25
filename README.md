# B-AIR
For project description visit: https://sites.google.com/berkeley.edu/b-air/

### To Run:
On Turtlebot:
1. `roslaunch turtlebot_bringup minimal.launch`

On Remote PC:
1. `roslaunch realsense2_camera rs_camera.launch`
2. `rosrun balloon_tracker balloon_tracker.py`
2. `rosrun balloon_control balloon_control.py`

Demo
    <p align="center">
      ![Gif](images/robot_demo.gif)
      ![Gif](images/detection.gif)
    </p>
