#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Vector3
# from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
# from tf2_msgs.msg import TFMessage
import math
import tf2_ros
from geometry_msgs.msg import Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class robotControl:

    def __init__(self):

        #load set of points 
        try:
            data = rospy.get_param("/points")
            temp_list = list(data.items())
            self.pointSet = [item[1] for item in temp_list]
            self.points_len = len(self.pointSet)
        except :    
            rospy.logerr("could not get param pointsSet... did you load them ?")

        self.point_target = self.pointSet[0]
        self.point_id = 0
        self.distance = 10000
        self.robotPosition = Transform()
        self.name = "Robot"
        self.velocity = Twist(Vector3(0,0,0),Vector3(0,0,0)) 
        self.thresh = 0.1
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.counter = 0 
        self.direction = -1




    def go_forward(self):
        self.velocity.linear = Vector3(0.5, 0, 0)
        rospy.loginfo("state go_forward")


    def go_backward(self):
        self.velocity.linear = Vector3(-0.5, 0, 0)
        rospy.loginfo("state go_backward")

    def flip_f(self):
        self.velocity.linear = Vector3(100, 0, 0)
        rospy.loginfo("state flip forward")

    def flip_b(self):
        self.velocity.linear = Vector3(100, 0, 0)
        rospy.loginfo("state flip backward")



    def update(self,trans):
        self.robotPosition = trans
        self.counter = self.counter + 1
        rospy.loginfo("counter : " + str(self.counter))

        if(self.counter>75):
            self.go_backward()
            if(self.counter>130):
                self.flip_b()
        else:
            self.go_forward()
            if(self.counter>50):
                self.flip_f()
        
        if(self.counter>150):
            self.counter = 0


        rospy.loginfo("Debug")
        # rospy.loginfo(self.velocity)
        self.pub.publish(self.velocity)
        



def talker():
    rospy.init_node('Velocity', anonymous=False)
    rate = rospy.Rate(20) # 10hz
    controler  = robotControl()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():


        try:
            trans = tfBuffer.lookup_transform("odom", 'link_chassis', rospy.Time())
            rospy.loginfo(trans.transform.translation)
            controler.update(trans.transform)

            # rospy.loginfo(trans.transform.rotation)
            # rospy.loginfo(tfBuffer.all_frames_as_string())


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # if(count<15):
            # message = Twist(Vector3(1,0,0),Vector3(0,0,0)) 
        
        # rospy.loginfo("Hello!")
        # pub.publish(message)
        rate.sleep()




if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass