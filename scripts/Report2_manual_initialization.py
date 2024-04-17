#!/usr/bin/env python3

import rospy
import math
# import the messages for reading the joint positions and sending joint commands
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
import tf2_geometry_msgs
from robot_vision_lectures.msg import XYZarray, SphereParams


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('manual_initialization', anonymous = True)
	# add a publisher for sending joint position commands
	pos_pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size = 10)
	# set a 10Hz frequency for this loop
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	loop_rate = rospy.Rate(10)
	# try getting the most update transformation between the tool frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available!!!')
			loop_rate.sleep()
			continue
	
	sphere_params_msg = SphereParams()
	pt_in_tool = tf2_geometry_msgs.PointStamped()
	pt_in_tool.header.frame_id = 'camera_color_optical_frame'
	pt_in_tool.header.stamp = rospy.get_rostime()
	pt_in_tool.point.z = sphere_params_msg.zc # 10 cm away from flange
	pt_in_tool.point.x = sphere_params_msg.xc
	pt_in_tool.point.y = sphere_params_msg.zc

	pt_in_base = tfBuffer.transform(pt_in_tool,'base', rospy.Duration(1.0))
		print('Test point in the TOOL frame:  x= ', format(pt_in_tool.point.x, '.3f'), '(m), y= ', format(pt_in_tool.point.y, '.3f'), '(m), z= ', format(pt_in_tool.point.z, '.3f'),'(m)')
		print('Transformed point in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
		print('-------------------------------------------------')
	
	# define a joint trajectory variable for sending the control commands
	pos_cmd = JointTrajectory()
	pos_cmd_point = JointTrajectoryPoint()
	# just a quick solution to complete the message template
	pos_cmd.joint_names.append('elbow_joint')
	pos_cmd.joint_names.append('shoulder_lift_joint')
	pos_cmd.joint_names.append('shoulder_pan_joint')
	pos_cmd.joint_names.append('wrist_1_joint')
	pos_cmd.joint_names.append('wrist_2_joint')
	pos_cmd.joint_names.append('wrist_3_joint')
	
	# initialize the position command to zero
	for joint_no in range(6):
		pos_cmd_point.positions.append(0.0)
	# set the ideal time to destination
	pos_cmd_point.time_from_start = rospy.Duration(1.0) # here one second 
	# just change the value of the command for the second joint
	pos_cmd_point.positions[1] = -math.pi/4
	# just change the value of the command for the elbow joint
	pos_cmd_point.positions[0] = math.pi/4
	# add the trajectory point to the command
	pos_cmd.points.append(pos_cmd_point)
	# define a message header	
	header = Header()
	
	while not rospy.is_shutdown():
		# update the header with the most recent time stamp
		header.stamp = rospy.Time.now()
		# use the most recent header in the position command message
		pos_cmd.header = header
		# publish the message
		pos_pub.publish(pos_cmd)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
