open robot at rViz : roslaunch interbotix_descriptions description.launch robot_name:="px100" jnt_pub_gui:=True
open rViz with the real robot : roslaunch interbotix_sdk arm_run.launch robot_name:="px100"
on/off torq on motors : rosservice call /px100/torque_joints_off


open rviz and move it 
link : https://github.com/Interbotix/interbotix_ros_arms/tree/master/interbotix_examples/interbotix_moveit_interface

roslaunch interbotix_moveit_interface moveit_interface.launch robot_name:=px100 use_python_interface:=True use_fake:=True 

roslaunch interbotix_moveit_interface moveit_interface.launch robot_name:=px100 use_python_interface:=True use_actual:=True 



roslaunch arm_move arm_box.launch robot_name:=px100 use_python_interface:=True use_actual:=True 
roslaunch arm_move arm_box.launch use_python_interface:=True use_fake:=True 
roslaunch arm_move arm_box.launch use_python_interface:=True use_actual:=True 

open at gazebo :
roslaunch interbotix_gazebo gazebo.launch robot_name:=px100 
