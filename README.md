turtlebot_navigation
====================
To run the program the following steps in strict order.

1) Run turtlebot simulation
    
  roslaunch turtlebot_gazebo turtlebot_playground.launch

2) Run slam map generation

  roslaunch turtlebot_gazebo gmapping_demo.launch

3) (optional) Run map visualiser
  
  roslaunch turtlebot_rviz_launchers view_navigation.launch

4) (optional) Run keyboard control of the robot

  roslaunch turtlebot_teleop keyboard_teleop.launch

5) Build and run the program

  catkin_make --pkg turtlebot_program
  rosrun turtlebot_program turtlebot_program cmd_vel:=base_controller/command
