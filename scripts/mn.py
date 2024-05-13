#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("my_node", anonymous=False)
pub=rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

i = 0

msg=Twist()
msg.linear.x=2.0
msg.angular.z=1.57

rate=rospy.Rate(1)


# while not rospy.is_shutdown():
#     t1 = rospy.Time.now()
#     print(t1)
#     if i<4:
# 		pub.publish(msg)
# 		rate.sleep()
		
# 	elif i==4:
# 		pub.publish(msg)
# 		msg.linear.x=0.0    
# 		msg.angular.z=0.0
# 		rate.sleep()	
		
# 	elif 4<i<9:
# 		pub.publish(msg)
# 		rate.sleep()
# 		msg.linear.x=2.0		
# 		msg.angular.z=-1.57
		
# 	else:
# 		pub.publish(msg)
# 		msg.linear.x=0.0
# 		msg.angular.z=0.0
# 		rate.sleep()

#     i=i+1
#     t2 = rospy.Time.now()
#     print(t2)

#     if (t2-t1).to_sec==174601573027207:
#         msg.linear.x = 0.0
#         msg.angular.z = 0.0
start_time = rospy.Time.now()
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    pub.publish(msg)
    if (current_time - start_time).to_sec() > 4.15:
        msg.linear.x = 2.0
        msg.angular.z = -1.57
        pub.publish(msg)

    if (current_time - start_time).to_sec() > 8.15:
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        pub.publish(msg)
        rate.sleep()
    
		

# #!/usr/bin/env python

# import rospy
# from geometry_msgs.msg import Twist

# rospy.init_node("my_node", anonymous=True)
# pub=rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

# i = 0

# msg=Twist()
# msg.linear.x=2.0
# msg.angular.z=1.57



# rate=rospy.Rate(1)



# while not rospy.is_shutdown():
# 	if i<4:
# 		pub.publish(msg)
# 		rate.sleep()
# 	elif i==4:
# 		pub.publish(msg)
# 		msg.linear.x=0.0    
# 		msg.angular.z=0.0
# 		rate.sleep()	
		
# 	elif 4<i<9:
# 		pub.publish(msg)
# 		rate.sleep()
# 		msg.linear.x=2.0		
# 		msg.angular.z=-1.57
		
# 	else:
# 		pub.publish(msg)
# 		msg.linear.x=0.0
# 		msg.angular.z=0.0
# 		rate.sleep()
# 	i=i+1
        






# #!/usr/bin/env python

# import rospy
# from geometry_msgs.msg import Twist

# rospy.init_node("my_node", anonymous=True)
# pub=rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

# i = 0

# msg=Twist()
# msg.linear.x= 3.5
# msg.angular.z= 6.1123



# rate=rospy.Rate(1)

# while not rospy.is_shutdown():
# 	while i % 4 == 0:
# 		pub.publish(msg)
# 		msg.angular.z *= -1
# 		rate.sleep()
# 	i=i+1
# 	break