# homework02-Dimitrios Chamzas 10/2020

# Brief overview
This package(diff_drive) is part of Embeded systems in Robotics homework.
The purpose is to create a differectial robot from sratch using xacro files.
Import the model at gazebo simulator and control it using the package libgazebo_ros_diff_drive

# Nodes
2 Different nodes exist for controling the robot.
1. Node : follow_rect makes the robot follow a series of points reading them from `<config/pointSet.yaml>` in this example the robot executes a rectangle trajectory. Fill free to exeriment by changing the points.
2. Node : flipper makes the robot exucute a flip by going back and forward on a line withought turning.

# Launch files 
1. If you wanna see the rebot in rViz execute `<roslaunch diff_drive see_robot.launch >`
2. In order to open the gazebo simulator and upload the robot exucute `<roslaunch diff_drive myGazebo.launch >`
3. If you have your gazebo already open or want to upload the robot again for some reason `<roslaunch diff_drive spawn.launch >` will do the job.
4. The main launch file is **ddrive.launch**,
    1. open Gazebo
    2. upload robot
    3. open rViz
    4. run Node



The simulator Is on pause mode so in order to get things moving unpause.


# Runing examples

![](https://github.com/ME495-EmbeddedSystems/homework-3-jimas95/blob/main/gifs/flip.gif)
![](https://github.com/ME495-EmbeddedSystems/homework-3-jimas95/blob/main/gifs/rectangle.gif)