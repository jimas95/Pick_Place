#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerResponse,TriggerRequest,Empty,EmptyRequest,EmptyResponse
from moveit_commander.conversions import pose_to_list
import tf2_ros

class RobotPX():
    def __init__(self):
        super(RobotPX, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('RobotPX',anonymous=True,log_level=rospy.DEBUG)
        robot_name = "px100"
        self.robot_name = robot_name
        self.dof = 4

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "interbotix_arm"
        # group_name = "interbotix_gripper"
        group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher("move_group/display_planned_path",
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        ## Getting Basic Information
        planning_frame = group.get_planning_frame()
        print( "============ Reference frame: %s" % planning_frame)

        eef_link = group.get_end_effector_link()
        print( "============ End effector: %s" % eef_link)

        group_names = robot.get_group_names()
        print( "============ Robot Groups:", robot.get_group_names())

        print( "============ print(ing robot state")
        print( robot.get_current_state())
        print( "")


        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        #create services 
        self.srv_joint_state  = rospy.Service('get_joint_states', Trigger, self.srvf_joint_state)
        self.srv_eef_position = rospy.Service('get_eef_pose', Trigger, self.srvf_eef_position)
        self.srv_pathPlan = rospy.Service('pathPlan', Empty, self.srvf_pathPlan)
        self.srv_execute_plan = rospy.Service('execute_plan', Empty, self.srvf_execute_plan)
        self.srv_reset = rospy.Service('reset', Empty, self.srvf_reset)

        #create my scene 
        self.create_my_scene()
        self.attach_box()

    def update(self):
        rate = rospy.Rate(1) # publish freacuancy (DO NOT Change)
        while not rospy.is_shutdown():
            rospy.logdebug("Hello!")
            self.print_joint_state()
            self.get_eef_pose(True)
            temp = self.scene.get_known_object_names()
            rospy.logdebug(temp)
            rate.sleep()

    def get_joint_state(self):
        return self.group.get_current_joint_values()
    
    def print_joint_state(self):
        joint_state = self.group.get_current_joint_values()
        i=0
        for joint in joint_state: 
            rospy.logdebug("joint "+str(i)+ ": " +str(joint*180/pi))
            i+=1
        rospy.logdebug("------")

    def srvf_joint_state(self,TriggerRequest):
        joint_state = self.get_joint_state()
        data = ""
        i=0
        for joint in joint_state: 
            data+= "joint " + str(i) + ": " + str(joint*180/pi) + "\n"
            i+=1
        msg = TriggerResponse()
        msg.message = str(joint_state)
        msg.success = True
        return msg


    def get_eef_pose(self,bool_print):
        current_pose = self.group.get_current_pose().pose
        if(bool_print): rospy.logdebug("eef position %s",current_pose)
        return current_pose

    def srvf_eef_position(self,TriggerRequest):
        msg = TriggerResponse()
        current_pose = self.group.get_current_pose().pose
        msg.message = "position: " + str(current_pose.position) + " orientation: " + str(current_pose.orientation)
        msg.success = True
        return msg

    def create_my_scene(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot_name + "/base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.6
        self.box_name = "graspObject"
        self.scene.add_box(self.box_name, box_pose, size=(0.025, 0.025, 0.05))

        if(not self.wait_for_state_update(objName = self.box_name, box_is_known=True, timeout=10)):
            rospy.logerr("ERROR ADDING OBJECT --> "+ self.box_name)


        
        tableHeight = 0.05
        tableSize = 0.5
        table1_position = [-0.1,0,0]
        self.add_table(width=0.25,height=0.6,position=table1_position,name="table1")
        leg_size = 0.025
        self.add_leg([ -0.2,-0.3 +leg_size/2 ,-tableHeight+table1_position[2]],0.25,"leg5")
        self.add_leg([ -0.2,0.3 -leg_size/2,-tableHeight+table1_position[2]],0.25,"leg6")
        

        
        tableHeight = 0.05
        tableSize = 0.5
        table2_position = [0,0,-0.25]
        self.add_table(width=0.5,height=1,position=table2_position,name="table2")
        leg_size = 0.025
        self.add_leg([-tableSize/2 +leg_size/2,-tableSize +leg_size/2 ,-tableHeight+table2_position[2]],0.5,"leg1")
        self.add_leg([-tableSize/2 +leg_size/2, tableSize -leg_size/2 ,-tableHeight+table2_position[2]],0.5,"leg2")
        self.add_leg([ tableSize/2 -leg_size/2,-tableSize +leg_size/2 ,-tableHeight+table2_position[2]],0.5,"leg3")
        self.add_leg([ tableSize/2 -leg_size/2, tableSize -leg_size/2 ,-tableHeight+table2_position[2]],0.5,"leg4")


              
    def add_table(self,width,height,position,name):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot_name + "/base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2] - 0.05/2
        self.scene.add_box(name, box_pose, size=(width, height, 0.05))

        rospy.logdebug("ADDING OBJECT --> "+ name)
        if(not self.wait_for_state_update(objName = name, box_is_known=True, timeout=10)):
            rospy.logerr("ERROR ADDING OBJECT --> "+ name)

    def add_leg(self,position,length,name):
        leg_size = 0.025
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot_name + "/base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2] - length/2
        self.scene.add_box(name, box_pose, size=(leg_size, leg_size, length))

        rospy.logdebug("ADDING OBJECT --> "+ name)
        if(not self.wait_for_state_update(objName = name, box_is_known=True, timeout=10)):
            rospy.logerr("ERROR ADDING OBJECT --> "+ name)


    def wait_for_state_update(self,objName, box_is_known=False, box_is_attached=False, timeout=5):
        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([objName])
            is_attached = len(attached_objects.keys()) > 0
            # rospy.logerr("still trying to get the object "+ objName)
            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = objName in self.scene.get_known_object_names()
            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def attach_box(self):
        grasping_group = 'interbotix_gripper'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, "graspObject", touch_links=touch_links)
        return self.wait_for_state_update(objName="graspObject",box_is_attached=True, box_is_known=False, timeout=4)

    def detach_box(self,obj_name):
        self.scene.remove_attached_object(self.eef_link, name=obj_name)
        return self.wait_for_state_update(obj_name,box_is_known=True, box_is_attached=False, timeout=4)

    def remove_box(self,obj_name):
        self.scene.remove_world_object(obj_name)
        return self.wait_for_state_update(obj_name,box_is_attached=False, box_is_known=False, timeout=4)

    def object_exists(self,obj_name):
        is_known = obj_name in self.scene.get_known_object_names()
        return is_known

    def plan_cartesian_path(self, scale=1):
        waypoints = []
        wpose = self.group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x =  0.12
        wpose.position.y =  0.0
        wpose.position.z = -0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x =  0.16
        wpose.position.y =  0.0
        wpose.position.z =  0.044
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x =  0.19
        wpose.position.y =  0.0
        wpose.position.z =  0.15
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def srvf_pathPlan(self,EmptyRequest):
        rospy.logdebug("Finding path")
        self.plan, fraction = self.plan_cartesian_path()
        rospy.logdebug("path FOUND")
        # self.display_trajectory(plan)
        return EmptyResponse()

    def srvf_execute_plan(self,EmptyRequest):
        rospy.logdebug("Executing path")
        self.group.execute(self.plan, wait=True)
        rospy.logdebug("path DONE")
        return EmptyResponse()

    def srvf_reset(self,EmptyRequest):

        return EmptyResponse()


if __name__ == '__main__':
    try:
        robot = RobotPX()
        robot.update()
    except rospy.ROSInterruptException:
        pass