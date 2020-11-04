#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import unittest
from time import sleep
import rostest
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse
from arm_move.srv import step_srv,step_srvResponse,step_srvRequest
from geometry_msgs.msg import Pose

class TestCollision(unittest.TestCase):

    def test_step_UnvalidPath(self):
        #init node 
        sleep(6)
        rospy.init_node("test")

        # create service caller
        srv = rospy.ServiceProxy("/px100/step", step_srv)

        #create position point 
        point = Pose()
        point.position.x = 0.0
        point.position.y = 0.2
        point.position.z = -0.10
        point.orientation.x = 0.0
        point.orientation.y = 0.0 
        point.orientation.z = 0.7 
        point.orientation.w = 0.7

        #input msg
        msg = step_srvRequest()
        msg.gripper = False
        msg.pose = point

        responce = srv.call(msg)
   
        result = False
        if(responce.ErrorCode.val==-1):
            result = True
        self.assertTrue(result)


    def test_step_validPath(self):
        #init node 
        rospy.init_node("test")
        # create service caller
        srv = rospy.ServiceProxy("/px100/step", step_srv)

        #create position point 
        point = Pose()
        point.position.x = 0.15
        point.position.y = 0.0
        point.position.z = 0.10
        point.orientation.x = 0.0
        point.orientation.y = 0.0 
        point.orientation.z = 0.0 
        point.orientation.w = 1.0

        #input msg
        msg = step_srvRequest()
        msg.gripper = False
        msg.pose = point

        responce = srv.call(msg)
        result = False
        if(responce.ErrorCode.val==1):
            result = True
        self.assertTrue(result)

if __name__=='__main__':
    rostest.rosrun("arm_move","test.py",TestCollision)