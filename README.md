# UR_Controller
In order to run this controller you need to clone the Universal Robot Ros Driver repository.
Follow the instructions here: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master
Once you have copied Ros Driver into your workspace, run the following commands:
  1-. roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=<IP>
  2-. roslaunch ur_robot_driver example_rviz.launch
Then you can run the python script UR_CONTROLLER_MAIN.py
At the moment, this program can only execute routines in joint angles notation 
