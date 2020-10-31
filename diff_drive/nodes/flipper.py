#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Vector3
import math
import tf2_ros

"""
Node Flip 
this node is an open loop that makes the robot flip back ang forth on the same line 
---- might be sencitive to different machines frecuancy has to be 20 Hz ----
Publicher : --> geometry_msgs.Twist --> at Topic cmd_vel

"""

class robotControl:

    def __init__(self):
        self.name = "Robot"
        self.velocity = Twist(Vector3(0,0,0),Vector3(0,0,0)) 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.counter = 0 

# set correct velocitys in order to flip the robot
    def update(self,trans):

        self.counter = self.counter + 1
        if(self.counter>70):#stop
            self.velocity.linear = Vector3(0,0,0)
            if(self.counter<75):
                self.velocity.linear = Vector3(-1.5,0,0)


        elif(self.counter>50):#flip
            self.velocity.linear = Vector3(-3,0,0)

        elif(self.counter>20):#stop
            self.velocity.linear = Vector3(0,0,0)
            if(self.counter<25):
                self.velocity.linear = Vector3(1.5,0,0)


        elif(self.counter>0):#flip
            self.velocity.linear = Vector3(3,0,0)

        if(self.counter>100): # RESET COUNTER 
            self.counter = 0 

        self.velocity.angular = Vector3(0,0,0)
        self.pub.publish(self.velocity) # PUBLISH VELOCITY
        rospy.loginfo("Debug")
        rospy.loginfo("counter : " + str(self.counter))
        rospy.loginfo("Velocity : "+ str(self.velocity.linear))
        
        


# main Loop
def talker(): 
    rospy.init_node('Flip', anonymous=False)
    rate = rospy.Rate(20) # publish freacuancy (DO NOT Change)
    controler  = robotControl()

    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():

        try:
            trans = tfBuffer.lookup_transform("odom", 'link_chassis', rospy.Time())
            controler.update(trans.transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()




if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass