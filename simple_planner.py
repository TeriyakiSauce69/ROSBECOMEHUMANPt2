#!/usr/bin/env python3

import rospy
import math

import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs

from robot_vision_lectures.msg import XYZarray 
from robot_vision_lectures.msg import SphereParams 


# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist

filterered_data_points  = SphereParams() 

# Set values to SphereParam after doing filtering 
def get_sphere_params(data): 
    global filterered_data_points 
    
    filterered_data_points.xc = data.xc
    filterered_data_points.yc = data.yc
    filterered_data_points.zc = data.zc
    filterered_data_points.radius = data.radius
    
def set_new_params(new_x, new_y, new_z):
	plan_point = Twist()
	# just a quick solution to send two target points
	# define a point close to the initial position
	plan_point.linear.x = new_x
	plan_point.linear.y = new_y
	plan_point.linear.z = new_z
	plan_point.angular.x = 3.09
	plan_point.angular.y = 0.059
	plan_point.angular.z = -1.57
	# add this point to the plan
	#plan.points.append(plan_point)
	return plan_point
	
	



if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)

	#Get sphere params
	rospy.Subscriber('/sphere_params', SphereParams, get_sphere_params) 
	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	# define a plan variable
	plan = Plan()

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	plan.points.append(set_new_params(-0.598, -0.127, 0.36)) 
	
	start_x = -0.598 
	start_y = -0.127
	start_z = 0.36
	
	flag = False
	while not rospy.is_shutdown():

		# try getting the most update transformation between the tool frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
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
			
		# define a testpoint in the tool frame (let's say 10 cm away from flange)
		pt_in_tool = tf2_geometry_msgs.PointStamped()
		pt_in_tool.header.frame_id = 'camera_color_optical_frame'
		pt_in_tool.header.stamp = rospy.get_rostime()
	
		pt_in_tool.point.x = filterered_data_points.xc
		pt_in_tool.point.y = filterered_data_points.yc
		pt_in_tool.point.z = filterered_data_points.zc	
	
		# convert the 3D point to the base frame coordinates
		pt_in_base = tfBuffer.transform(pt_in_tool,'base', rospy.Duration(1.0))
		
		print('Test point in the TOOL frame:  x= ', format(pt_in_tool.point.x, '.3f'), '(m), y= ', format(pt_in_tool.point.y, '.3f'), '(m), z= ', format(pt_in_tool.point.z, '.3f'),'(m)')
		print('Transformed point in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
		
		
		if flag  == False:
			#the_answers = the_magic()
			
			plan.points.append(set_new_params(pt_in_base.point.x, pt_in_base.point.y, start_z))	
			plan.points.append(set_new_params(pt_in_base.point.x, pt_in_base.point.y, pt_in_base.point.z))
			plan.points.append(set_new_params(pt_in_base.point.x, pt_in_base.point.y, start_z))
			plan.points.append(set_new_params(start_x, start_y, start_z))			
			plan.points.append(set_new_params(start_x, start_y, pt_in_base.point.z))
			
			flag = True
		# publish the plan
		plan_pub.publish(plan)
		
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
