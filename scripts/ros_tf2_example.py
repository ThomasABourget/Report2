#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs
from robot_vision_lectures.msg import SphereParams

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('ros_tf_example', anonymous = True)
	# add a ros transform listener
	 # Add a subscriber to read the estimated ball position
	# Global variables to store the estimated ball position
	ball_x = 0.0
	ball_y = 0.0
	ball_z = 0.0

	# Callback function to update the estimated ball position
	def ball_callback(data):
		global ball_x, ball_y, ball_z
		ball_x = data.xc
		ball_y = data.yc
		ball_z = data.zc
		q_rot = Quaternion()	
		
	rospy.Subscriber('/sphere_params', SphereParams, ball_callback)
	
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	
	
	
	while not rospy.is_shutdown():

		# try getting the most update transformation between the tool frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "fk_tooltip", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available!!!')
			loop_rate.sleep()
			continue
		# extract the xyz coordinates
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# extract the quaternion and converto RPY
		q_rot = trans.transform.rotation
		roll, pitch, yaw, = euler_from_quaternion([q_rot.x, q_rot.y, q_rot.z, q_rot.w])
		# a quick check of the readings
		print('Tool frame position and orientation w.r.t base: x= ', format(x, '.3f'), '(m),  y= ', format(y, '.3f'), '(m), z= ', format(z, '.3f'),'(m)')
		print('roll= ', format(roll, '.2f'), '(rad), pitch= ', format(pitch, '.2f'), '(rad), yaw: ', format(yaw, '.2f'),'(rad)') 
		# define a testpoint in the tool frame (let's say 10 cm away from flange)
		pt_in_tool = tf2_geometry_msgs.PointStamped()
		pt_in_tool.header.frame_id = 'fk_tooltip'
		pt_in_tool.header.stamp = rospy.get_rostime()
		sphere_params_msg = SphereParams()
		# Adjust the point in the tool frame to be at the center of the ball
		pt_in_tool.point.x = sphere_params_msg.xc
		pt_in_tool.point.y = sphere_params_msg.yc
		pt_in_tool.point.z = sphere_params_msg.zc

		# convert the 3D point to the base frame coordinates
		pt_in_base = tfBuffer.transform(pt_in_tool,'base', rospy.Duration(1.0))
		print('Test point in the TOOL frame:  x= ', format(pt_in_tool.point.x, '.3f'), '(m), y= ', format(pt_in_tool.point.y, '.3f'), '(m), z= ', format(pt_in_tool.point.z, '.3f'),'(m)')
		print('Transformed point in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
		print('-------------------------------------------------')
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()

