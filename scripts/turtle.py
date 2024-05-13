#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("my_node", anonymous=False)
pub=rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

msg=Twist()
msg.linear.x=3.0
msg.linear.y=0.0
msg.linear.z=0.0
msg.angular.x=0.0
msg.angular.y=0.0
msg.angular.z=0.0



rate=rospy.Rate(1)
