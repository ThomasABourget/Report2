#!/usr/bin/env python3
import math
import rospy

from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist 
import tf2_geometry_msgs
from robot_vision_lectures.msg import XYZarray, SphereParams




if __name__ == '__main__':
	# initialize the node
	rospy.init_node('report2_planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	# try getting the most update transformation between the tool frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "fk_tooltip", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available!!!')
			loop_rate.sleep()
			continue
	
	pt_in_tool = tf2_geometry_msgs.PointStamped()
	pt_in_tool.header.frame_id = 'fk_tooltip'
	pt_in_tool.header.stamp = rospy.get_rostime()
	pt_in_tool.point.z= 0.1 # 10 cm away from flange
	

	sphere_params_msg = SphereParams()


	# define a plan variable
	plan = Plan()
	plan_point1 = Twist()
	# just a quick solution to send two target points
	# define a point close to the initial position
	plan_point1.linear.x = -0.7
	plan_point1.linear.y = -0.23
	plan_point1.linear.z = 0.363
	plan_point1.angular.x = 1.57
	plan_point1.angular.y = 0.0
	plan_point1.angular.z = 0.0
	# add this point to the plan
	plan.points.append(plan_point1)
	
	plan_point2 = Twist()
	# define a point away from the initial position
	plan_point2.linear.x = sphere_params_msg.xc
	plan_point2.linear.y = sphere_params_msg.yc
	plan_point2.linear.z = sphere_params_msg.zc
	plan_point2.angular.x = 1.57
	plan_point2.angular.y = 0.0
	plan_point2.angular.z = 0.0
	# add this point to the plan
	plan.points.append(plan_point2)

	plan_point3 = Twist()
	# define a point away from the initial position
	plan_point3.linear.x = -0.7
	plan_point3.linear.y = -0.23
	plan_point3.linear.z = 0.363
	plan_point3.angular.x = 1.57
	plan_point3.angular.y = 0.0
	plan_point3.angular.z = 0.0
	# add this point to the plan
	plan.points.append(plan_point3)

	plan_point4 = Twist()
	# define a point away from the initial position
	plan_point4.linear.x = -0.6367
	plan_point4.linear.y = -0.4039
	plan_point4.linear.z = 0.095
	plan_point4.angular.x = 1.57
	plan_point4.angular.y = 0.0
	plan_point4.angular.z = 0.0
	# add this point to the plan
	plan.points.append(plan_point4)

	
	while not rospy.is_shutdown():
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
