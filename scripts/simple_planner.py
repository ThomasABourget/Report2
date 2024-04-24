#!/usr/bin/env python3
import math
import rospy
import tf2_ros

from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist 
import tf2_geometry_msgs
from robot_vision_lectures.msg import XYZarray, SphereParams



if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)


def sphere_params_callback(data):
    # Callback function to update the point coordinates
    pt_in_tool.point.x = data.xc
    pt_in_tool.point.y = data.yc
    pt_in_tool.point.z = data.zc
    print(pt_in_tool.point.x, pt_in_tool.point.y, pt_in_tool.point.z)	
    return pt_in_tool.point.x, pt_in_tool.point.y, pt_in_tool.point.z




if __name__ == '__main__':
    # initialize the node
    rospy.init_node('simple_planner', anonymous=True)
    
    # add a publisher for sending joint position commands
    plan_pub = rospy.Publisher('/plan', Plan, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # set a 10Hz frequency for this loop
    loop_rate = rospy.Rate(10)
    
    try:
    	trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    	print('Frames not available!!!')
    	loop_rate.sleep()
    	
    rospy.Subscriber("/sphere_params", SphereParams, sphere_params_callback)
    pt_in_tool = tf2_geometry_msgs.PointStamped()
    pt_in_tool.header.frame_id = 'camera_color_optical_frame'
    pt_in_tool.header.stamp = rospy.get_rostime()
    pt_in_tool.point.x = -0.013910026289522648
    pt_in_tool.point.y = -0.017049305140972137
    pt_in_tool.point.z = 0.477964848279953

    
    pt_in_base = tfBuffer.transform(pt_in_tool,'base', rospy.Duration(1.0))
    print('Test point in the TOOL frame:  x= ', format(pt_in_tool.point.x, '.3f'), '(m), y= ', format(pt_in_tool.point.y, '.3f'), '(m), z= ', format(pt_in_tool.point.z, '.3f'),'(m)')
    print('Transformed point in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
    print('-------------------------------------------------')
    
    plan = Plan()
    plan_point1 = Twist()
    plan_point1.linear.x = -0.558
    plan_point1.linear.y = -0.131
    plan_point1.linear.z = 0.2247
    plan_point1.angular.x = -3.00
    plan_point1.angular.y = 0.0
    plan_point1.angular.z = -1.57
    #ADD points
    plan.points.append(plan_point1)
    
    plan_point2 = Twist()
    plan_point2.linear.x = pt_in_base.point.x
    plan_point2.linear.y = pt_in_base.point.y
    plan_point2.linear.z = pt_in_base.point.z
    plan_point2.angular.x = -3.00
    plan_point2.angular.y = 0.0
    plan_point2.angular.z = -1.57
    # add this point to the plan
    
    plan.points.append(plan_point2)
    
    plan_point3 = Twist()
    plan_point3.linear.x = pt_in_base.point.x
    plan_point3.linear.y = pt_in_base.point.y
    plan_point3.linear.z = pt_in_base.point.z + .3
    plan_point3.angular.x = -3.00
    plan_point3.angular.y = 0.0
    plan_point3.angular.z = -1.57
    # add this point to the plan
    
    plan.points.append(plan_point3)
    
    
    plan_point4 = Twist()
    plan_point4.linear.x = pt_in_base.point.x
    plan_point4.linear.y = pt_in_base.point.y
    plan_point4.linear.z = pt_in_base.point.z
    plan_point4.angular.x = -3.00
    plan_point4.angular.y = 0.0
    plan_point4.angular.z = -1.57
    # add this point to the plan
    plan.points.append(plan_point4)
    
    plan_point5 = Twist()
    plan_point5.linear.x = -0.558
    plan_point5.linear.y = -0.131
    plan_point5.linear.z = 0.2247
    plan_point5.angular.x = -3.00
    plan_point5.angular.y = 0.0
    plan_point5.angular.z = -1.57
    #ADD points
    plan.points.append(plan_point5)
    
    
    
    while not rospy.is_shutdown():
    	plan_pub.publish(plan)
    	# wait for 0.1 seconds until the next loop and repeat
    	loop_rate.sleep()
