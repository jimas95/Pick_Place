# homework02-Dimitrios Chamzas 10/2020

# Brief overview
This package(diff_drive) is part of Embeded systems in Robotics homework.
The purpose is to create a differectial robot from sratch using xacro files.
Import the model at gazebo simulator and control it using the package libgazebo_ros_diff_drive.

# Nodes
2 Different nodes exist for controling the robot.
1. **follow_rect** makes the robot follow a series of points reading them from `<config/pointSet.yaml>` in this example the robot executes a rectangle trajectory. Fill free to exeriment by changing the points.
2. **flipper** makes the robot exucute a flip by going back and forward on a line withought turning.

# Launch files 
1. If you wanna see the robot in rViz execute `<roslaunch diff_drive see_robot.launch >`
2. In order to open the gazebo simulator and upload the robot exucute `<roslaunch diff_drive myGazebo.launch >`
3. If you have your gazebo already open or want to upload the robot again for some reason `<roslaunch diff_drive spawn.launch >` will do the job.
4. The main launch file of package is **ddrive.launch**,
    1. opens Gazebo
    2. uploads robot
    3. opens rViz
    4. run Nodes

## Arguments 
1. pause*(boolean)*, initial state of Gazebo. Default: True
2. rViz*(boolean)*, opens rViz. Default: True
3. follow*(boolean)*, runs follow_rect node. Default: False
4. flip*(boolean)*, runs flipper node. Default: False
5. x,y,z*(float)* , set the initial position in the gazebo world. Default: 0,0,0.5 
6. world*(string path)*, path to uploading world.  Default: `<$(find diff_drive)/worlds/ddrive.world>`

The simulator is on pause mode so in order to get things moving unpause.
It is recomended to open only gazebo and execute each node from a terminal.
Do **NOT** set both flip,follow args to True, will cause unexpected output.


# Runing examples
## for flipping the robot run `<roslaunch diff_drive ddrive.launch flip:=True>`

![](https://github.com/ME495-EmbeddedSystems/homework-3-jimas95/blob/main/gifs/flip.gif)

## for following points the robot run `<roslaunch diff_drive ddrive.launch follow:=True>`

![](https://github.com/ME495-EmbeddedSystems/homework-3-jimas95/blob/main/gifs/rectangle.gif)