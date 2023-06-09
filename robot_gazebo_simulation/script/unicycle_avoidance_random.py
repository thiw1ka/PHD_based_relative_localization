#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


linvel = 0.05
mindist = 0.6

# this is a global variable: all functions and methods in this file have access to it
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


# this is our callback function: it is executed any time a message on the specified topic
# is received. In our case, the specified topic is /drone/gt_pose.
# The message received will be available in the function through the variable msg  
def scanCallback(msg):
		
    # to use a global variable in a function, you must explicitly
    # redefine with the global attribute
    global cmd_pub
    global linvel
    samples = len(msg.ranges)
    min_left = 10.0
    min_right = 10.0
    for i in range(samples):
      angle = msg.angle_min+i*msg.angle_increment
      if angle > -1.57 and angle < 0:
        if min_right > msg.ranges[i]:
          min_right = msg.ranges[i]
        #print(i,' r ',msg.ranges[i],' ',angle)
      if angle > 0 and angle < 1.57:
        if min_left > msg.ranges[i]:
          min_left = msg.ranges[i]
        #print(i,' l ',msg.ranges[i],' ',angle)
      
    #print(' l ',min_left,' r ',min_right)
		
    lin_vel = linvel
    omega = 2.0*linvel*(min_left - min_right)
    if omega > 5.0*linvel:
      omega = 5.0*linvel
    if min_left < mindist or min_right < mindist:
      lin_vel = 0
      omega = 5.0*linvel
    


    # here I create a new Twist message (=linear velocity and angular velocity), I write
    # the value I computed for the linear velocity on z to achieve a flight height of 2m
    # and I publish it on the appropriate topic
    cmdmsg = Twist()
    cmdmsg.linear.x = lin_vel
    cmdmsg.angular.z = omega

		
    cmd_pub.publish(cmdmsg)
    




def my_first_controller():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global linvel
    global mindist

    rospy.init_node('escaper', anonymous=False)
    
    
    nodename = rospy.get_name()
    linvel = rospy.get_param(nodename+"/linvel", 0.05);
    mindist = rospy.get_param(nodename+"/mindist", 0.6);


    rospy.Subscriber("/scan", LaserScan, scanCallback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# this is the main function: the program starts here
if __name__ == '__main__':
    my_first_controller()



