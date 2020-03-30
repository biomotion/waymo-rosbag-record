#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf.msg import tfMessage
import math
from tf.transformations import quaternion_from_euler
import tf

br = tf.TransformBroadcaster()
#tf_pub = rospy.Publisher("Transform", tfMessage, queue_size = 100)

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def isRotationMatrix(R) :
	Rt = np.transpose(R)
	shouldBeIdentity = np.dot(Rt, R)
	I = np.identity(3, dtype = R.dtype)
	n = np.linalg.norm(I - shouldBeIdentity)
	return n < 1e-6
def rotationMatrixToEulerAngles(R) :

	assert(isRotationMatrix(R))
	
	sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
	
	singular = sy < 1e-6
	
	if  not singular :
		x = math.atan2(R[2,1] , R[2,2])
		y = math.atan2(-R[2,0], sy)
		z = math.atan2(R[1,0], R[0,0])
	else :
		x = math.atan2(-R[1,2], R[1,1])
		y = math.atan2(-R[2,0], sy)
		z = 0
	
	return np.array([x, y, z])


def talker():
	rospy.init_node('transform', anonymous=True)    
	rospy.Subscriber("Transform", tfMessage, vehicle_info_callback)
	rate = rospy.Rate(1) # 10hz
	while not rospy.is_shutdown():

		"""
		if get_camera_image is True:
			camera_image_visualization()

		if get_lidars is True:
			lidar_pointcloud_visualization()
			if len(lidars.lidars[0].labels) is not 0:
				lidar_label_visualization()

		if get_vehicle is True:
			transform = TransformStamped()
			transform.header = vehicle_info.header
			rotation_matrix = np.array([(vehicle_info.pose[0], vehicle_info.pose[1], vehicle_info.pose[2]),(vehicle_info.pose[04], vehicle_info.pose[5], vehicle_info.pose[6]),(vehicle_info.pose[8], vehicle_info.pose[9], vehicle_info.pose[10])])
			r,p,y = rotationMatrixToEulerAngles(rotation_matrix)
			quaternion = quaternion_from_euler(r, p, y)
			transform.transform.rotation.x = quaternion[0]
			transform.transform.rotation.y = quaternion[1]
			transform.transform.rotation.z = quaternion[2]
			transform.transform.rotation.w = quaternion[3]
			transform.transform.translation.x = vehicle_info.pose[3]
			transform.transform.translation.x = vehicle_info.pose[7]
			transform.transform.translation.x = vehicle_info.pose[11]
			transform.child_frame_id = "global"
			global transforms
			transforms.transforms.append(transform)

		tf_pub.publish(transforms)
		"""
		
		rate.sleep()

def vehicle_info_callback(msg):
	print("get vehicle info")
	
	for transfer in msg.transforms:
		if transfer.child_frame_id == "global":
			print(transfer)
			br.sendTransform((transfer.transform.translation.x, transfer.transform.translation.y, transfer.transform.translation.z),
				(transfer.transform.rotation.x, transfer.transform.rotation.y, transfer.transform.rotation.z, transfer.transform.rotation.w),
				transfer.header.stamp,
				transfer.child_frame_id,
				transfer.header.frame_id )


if __name__ == '__main__': 
	try:
		talker()
	except  rospy.ROSInterruptException:
		pass

