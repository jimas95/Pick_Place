#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Vector3
import math
import tf2_ros
from geometry_msgs.msg import Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
Node Follow Points
Publishers : Topic /cmd_vel, type Twist
Subscribers: Topic tf,       type Transform
parameters : /point

this node gets the position of the robot from the tf tree 
looking at transformation odom --> link_chassis
calculates the propere linear and angular velocity 
for driving the robot in a series of points (in this case a rectungle)
"""

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


    """calculates the vector from turtle position --> target point"""
    def calVector(self):

        #calculate distance from target point 
        vector_t_pt =  [self.point_target[0] - self.robotPosition.translation.x,
                        self.point_target[1] - self.robotPosition.translation.y]
        self.distance = math.sqrt(math.pow(vector_t_pt[0],2) + math.pow(vector_t_pt[1],2))
        rospy.loginfo(self.name + " distance from point" + str(self.point_target) + " is ---> " + str(self.distance))

        #calculate direction angle 
        self.theta_dir = math.atan2(vector_t_pt[1], vector_t_pt[0])
        rospy.loginfo(self.name + " directional angle is ---> " + str(self.theta_dir*(180.0/math.pi)))



    """ calculate the proper linear and angular velocity based on current position,orientetion anf goal point"""
    def calVelocity(self):   # this is my control....
        
        #convert orientation
        orientation_q = self.robotPosition.rotation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        angle_speed = self.theta_dir - yaw

        if(angle_speed <-math.pi):
            angle_speed = 2*math.pi + angle_speed

        if(angle_speed>math.pi):
            angle_speed = -(2*math.pi - angle_speed)

        rospy.loginfo(self.name + " current angle is ---> " + str(yaw*(180.0/math.pi)))
        rospy.loginfo(self.name + " velocity angle is ---> " + str(angle_speed*(180.0/math.pi)))
        rospy.loginfo(self.name + " hunting point is ---> " + str(self.point_target))

        #set speed 
        self.velocity.linear = Vector3(0.6,0,0)
        self.velocity.angular = Vector3(0,0,-angle_speed)

        # go "full speed"
        if(self.distance>1.0 and abs(angle_speed)<0.25):
            self.velocity.linear = Vector3(2,0,0)

            


    """checks if we reached last point"""
    def set_next_point(self):

        self.point_id += 1
        if(self.point_id==self.points_len): #check if we reached last point
            self.point_id = 0
        
        self.point_target = self.pointSet[self.point_id] #set next target point

    """
    main update function
    calculates direction vector 
    calculates appropriate speed
    publish velocity
    """
    def update(self,trans):
        rospy.loginfo("Debug")
        
        self.robotPosition = trans
        self.calVector()   
        self.calVelocity()      

        rospy.loginfo("distance : "+str(self.distance))
        rospy.loginfo(self.velocity)

        if(self.distance <  self.thresh):
            self.set_next_point()

        self.pub.publish(self.velocity)
        


"""main Loop: starts node, and iterates"""
def talker(): 
    rospy.init_node('Follow_Points', anonymous=False)
    rate = rospy.Rate(20) # 20HZ
    controler  = robotControl()

    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():


        try:
            trans = tfBuffer.lookup_transform("odom", 'link_chassis', rospy.Time())
            rospy.loginfo(trans.transform.translation)
            controler.update(trans.transform)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue


        rate.sleep()


"""
Start everyting
"""
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass