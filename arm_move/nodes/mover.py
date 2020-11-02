#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerResponse,TriggerRequest
from moveit_commander.conversions import pose_to_list

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
        group_name = "interbotix_gripper"
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

        #create my scene 
        self.create_my_scene()


    def update(self):
        rate = rospy.Rate(1) # publish freacuancy (DO NOT Change)
        while not rospy.is_shutdown():
            rospy.logdebug("Hello!")
            self.print_joint_state()
            # self.get_eef_pose(True)
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
        self.box_name = "graspObject"
        self.scene.add_box(self.box_name, box_pose, size=(0.025, 0.025, 0.05))

        if(self.wait_for_state_update(objName = self.box_name, box_is_known=True, timeout=10)):
            rospy.logerr("ERROR ADDING OBJECT --> "+ self.box_name)


        objectName = "table"
        tableHeight = 0.05
        tableSize = 0.5
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = -0.025
        self.scene.add_box(objectName, box_pose, size=(2*tableSize, 2*tableSize, tableHeight))

        if(self.wait_for_state_update(objName = objectName, box_is_known=True, timeout=4)):
            rospy.logerr("ERROR ADDING OBJECT -->" + self.box_name)
        
        objectName = "leg1"
        box_pose.pose.position.x = tableSize
        box_pose.pose.position.y = tableSize
        box_pose.pose.position.z = - tableHeight -0.25
        self.scene.add_box(objectName, box_pose, size=(0.025, 0.025, 0.5))

        if(self.wait_for_state_update(objName = objectName, box_is_known=True, timeout=4)):
            rospy.logerr("ERROR ADDING OBJECT -->" + self.box_name)

        objectName = "leg2"
        box_pose.pose.position.x = -tableSize
        box_pose.pose.position.y = tableSize
        box_pose.pose.position.z = - tableHeight-0.25
        self.scene.add_box(objectName, box_pose, size=(0.025, 0.025, 0.5))

        if(self.wait_for_state_update(objName = objectName, box_is_known=True, timeout=4)):
            rospy.logerr("ERROR ADDING OBJECT -->" + self.box_name)

        objectName = "leg3"
        box_pose.pose.position.x = tableSize
        box_pose.pose.position.y = -tableSize
        box_pose.pose.position.z = - tableHeight-0.25
        self.scene.add_box(objectName, box_pose, size=(0.025, 0.025, 0.5))

        if(self.wait_for_state_update(objName = objectName, box_is_known=True, timeout=4)):
            rospy.logerr("ERROR ADDING OBJECT -->" + self.box_name)

        objectName = "leg4"
        box_pose.pose.position.x = -tableSize
        box_pose.pose.position.y = -tableSize
        box_pose.pose.position.z = - tableHeight-0.25
        self.scene.add_box(objectName, box_pose, size=(0.025, 0.025, 0.5))

        if(self.wait_for_state_update(objName = objectName, box_is_known=True, timeout=4)):
            rospy.logerr("ERROR ADDING OBJECT -->" + self.box_name)

              



    def wait_for_state_update(self,objName, box_is_known=False, box_is_attached=False, timeout=5):
        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([objName])
            is_attached = len(attached_objects.keys()) > 0
            rospy.logerr("still trying to get the object "+ objName)
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




if __name__ == '__main__':
    try:
        robot = RobotPX()
        robot.update()
    except rospy.ROSInterruptException:
        pass