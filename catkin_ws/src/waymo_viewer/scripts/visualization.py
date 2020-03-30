#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
from waymo_viewer.msg import CameraImageArray
from waymo_viewer.msg import LidarArray
from waymo_viewer.msg import VehicleInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from tf.msg import tfMessage
import math
from tf.transformations import quaternion_from_euler
import tf


camera_images = CameraImageArray()
lidars = LidarArray()
br = CvBridge()
transforms = tfMessage()

get_camera_image = False
get_lidars = False
get_vehicle = False

vehicle = (0, 0, 255)
pedestrian = (0, 255, 0)
cyclist = (255, 0, 0)
sign = (0, 255, 255)

#camera_front_labelimage_pub = rospy.Publisher("camera_front_labelimage", Image, queue_size = 100)
#camera_frontleft_labelimage_pub = rospy.Publisher("camera_frontleft_labelimage", Image, queue_size = 100)
#camera_frontright_labelimage_pub = rospy.Publisher("camera_frontright_labelimage", Image, queue_size = 100)
#camera_sideleft_labelimage_pub = rospy.Publisher("camera_sideleft_labelimage", Image, queue_size = 100)
#camera_sideright_labelimage_pub = rospy.Publisher("camera_sideright_labelimage", Image, queue_size = 100)
camera_front_image_pub = rospy.Publisher("camera_front_image", Image, queue_size = 100)
camera_frontleft_image_pub = rospy.Publisher("camera_frontleft_image", Image, queue_size = 100)
camera_frontright_image_pub = rospy.Publisher("camera_frontright_image", Image, queue_size = 100)
camera_sideleft_image_pub = rospy.Publisher("camera_sideleft_image", Image, queue_size = 100)
camera_sideright_image_pub = rospy.Publisher("camera_sideright_image", Image, queue_size = 100)
camera_front_label_pub = rospy.Publisher("camera_front_label", MarkerArray, queue_size = 100)
camera_frontleft_label_pub = rospy.Publisher("camera_frontleft_label", MarkerArray, queue_size = 100)
camera_frontright_label_pub = rospy.Publisher("camera_frontright_label", MarkerArray, queue_size = 100)
camera_sideleft_label_pub = rospy.Publisher("camera_sideleft_label", MarkerArray, queue_size = 100)
camera_sideright_label_pub = rospy.Publisher("camera_sideright_label", MarkerArray, queue_size = 100)

lidar_top_pub = rospy.Publisher("lidar_top_pointcloud", PointCloud, queue_size = 100)
lidar_front_pub = rospy.Publisher("lidar_front_pointcloud", PointCloud, queue_size = 100)
lidar_left_pub = rospy.Publisher("lidar_left_pointcloud", PointCloud, queue_size = 100)
lidar_right_pub = rospy.Publisher("lidar_right_pointcloud", PointCloud, queue_size = 100)
lidar_rear_pub = rospy.Publisher("lidar_rear_pointcloud", PointCloud, queue_size = 100)
lidar_label_pub = rospy.Publisher("lidar_label", MarkerArray, queue_size = 100)

tf_pub = rospy.Publisher("Transform", tfMessage, queue_size = 100)

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

def camera_image_visualization():
	
	for camera_image in camera_images.camera_images:
		
		transform = TransformStamped()
		transform.header = camera_image.header
		rotation_matrix = np.array([(camera_image.extrinsic[0], camera_image.extrinsic[1], camera_image.extrinsic[2]),(camera_image.extrinsic[4], camera_image.extrinsic[5], camera_image.extrinsic[6]),(camera_image.extrinsic[8], camera_image.extrinsic[9], camera_image.extrinsic[10])])
		r,p,y = rotationMatrixToEulerAngles(rotation_matrix)
		quaternion = quaternion_from_euler(r, p, y)
		transform.transform.rotation.x = quaternion[0]
		transform.transform.rotation.y = quaternion[1]
		transform.transform.rotation.z = quaternion[2]
		transform.transform.rotation.w = quaternion[3]
		transform.transform.translation.x = camera_image.extrinsic[3]
		transform.transform.translation.y = camera_image.extrinsic[7]
		transform.transform.translation.z = camera_image.extrinsic[11]

		if camera_image.name == 1:
			#camera_image_front = br.imgmsg_to_cv2(camera_image.image, "bgr8")
			front_labels = MarkerArray()
			
			for index, label in enumerate(camera_image.labels):				
				label_marker = Marker()
				label_marker.header = label.header
				label_marker.type = Marker.CUBE
				label_marker.action = Marker.ADD
				label_marker.id = index
				label_marker.pose.position.x = label.box.center_x
				label_marker.pose.position.y = label.box.center_y
				label_marker.scale.x = label.box.length
				label_marker.scale.y = label.box.width

				if label.type == 1:
					#cv2.rectangle(camera_image_front, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), vehicle, 2)
					label_marker.ns = "Vehicle"
				elif label.type == 2:
					#cv2.rectangle(camera_image_front, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), pedestrian, 2)
					label_marker.ns = "Pedestrian"
				elif label.type == 3:
					#cv2.rectangle(camera_image_front, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), sign, 2)
					label_marker.ns = "Sign"
				elif label.type == 4:
					#cv2.rectangle(camera_image_front, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), cyclist, 2)
					label_marker.ns = "Cyclist"
				
				front_labels.markers.append(label_marker)

			camera_front_label_pub.publish(front_labels)
			camera_front_image_pub.publish(camera_image.image)
			transform.child_frame_id = "camera_front"

		elif camera_image.name == 2:			
			#camera_image_frontleft = br.imgmsg_to_cv2(camera_image.image, "bgr8")
			frontleft_labels = MarkerArray()
			
			for index, label in enumerate(camera_image.labels):
				label_marker = Marker()
				label_marker.header = label.header
				label_marker.type = Marker.CUBE
				label_marker.action = Marker.ADD
				label_marker.id = index
				label_marker.pose.position.x = label.box.center_x
				label_marker.pose.position.y = label.box.center_y
				label_marker.scale.x = label.box.length
				label_marker.scale.y = label.box.width

				if label.type == 1:
					#cv2.rectangle(camera_image_frontleft, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), vehicle, 2)
					label_marker.ns = "Vehicle"
				elif label.type == 2:
					#cv2.rectangle(camera_image_frontleft, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), pedestrian, 2)
					label_marker.ns = "Pedestrian"
				elif label.type == 3:
					#cv2.rectangle(camera_image_frontleft, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), sign, 2)
					label_marker.ns = "Sign"
				elif label.type == 4:
					#cv2.rectangle(camera_image_frontleft, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), cyclist, 2)
					label_marker.ns = "Cyclist"
			
				frontleft_labels.markers.append(label_marker)

			camera_frontleft_label_pub.publish(frontleft_labels)
			camera_frontleft_image_pub.publish(camera_image.image)
			transform.child_frame_id = "camera_frontleft"

		elif camera_image.name == 3:
			#camera_image_frontright = br.imgmsg_to_cv2(camera_image.image, "bgr8")
			frontright_labels = MarkerArray()
			
			for index, label in enumerate(camera_image.labels):
				label_marker = Marker()
				label_marker.header = label.header
				label_marker.type = Marker.CUBE
				label_marker.action = Marker.ADD
				label_marker.id = index
				label_marker.pose.position.x = label.box.center_x
				label_marker.pose.position.y = label.box.center_y
				label_marker.scale.x = label.box.length
				label_marker.scale.y = label.box.width

				if label.type == 1:
					#cv2.rectangle(camera_image_frontright, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), vehicle, 2)
					label_marker.ns = "Vehicle"
				elif label.type == 2:
					#cv2.rectangle(camera_image_frontright, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), pedestrian, 2)
					label_marker.ns = "Pedestrian"
				elif label.type == 3:
					#cv2.rectangle(camera_image_frontright, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), sign, 2)
					label_marker.ns = "Sign"
				elif label.type == 4:
					#cv2.rectangle(camera_image_frontright, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), cyclist, 2)
					label_marker.ns = "Cyclist"
			
				frontright_labels.markers.append(label_marker)

			camera_frontright_label_pub.publish(frontright_labels)
			camera_frontright_image_pub.publish(camera_image.image)
			transform.child_frame_id = "camera_frontright"

		elif camera_image.name == 4:
			#camera_image_sideleft = br.imgmsg_to_cv2(camera_image.image, "bgr8")
			sideleft_labels = MarkerArray()
			
			for index, label in enumerate(camera_image.labels):
				label_marker = Marker()
				label_marker.header = label.header
				label_marker.type = Marker.CUBE
				label_marker.action = Marker.ADD
				label_marker.id = index
				label_marker.pose.position.x = label.box.center_x
				label_marker.pose.position.y = label.box.center_y
				label_marker.scale.x = label.box.length
				label_marker.scale.y = label.box.width

				if label.type == 1:
					#cv2.rectangle(camera_image_sideleft, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), vehicle, 2)
					label_marker.ns = "Vehicle"
				elif label.type == 2:
					#cv2.rectangle(camera_image_sideleft, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), pedestrian, 2)
					label_marker.ns = "Pedestrian"
				elif label.type == 3:
					#cv2.rectangle(camera_image_sideleft, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), sign, 2)
					label_marker.ns = "Sign"
				elif label.type == 4:
					#cv2.rectangle(camera_image_sideleft, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), cyclist, 2)	
					label_marker.ns = "Cyclist"
			
				sideleft_labels.markers.append(label_marker)

			camera_sideleft_label_pub.publish(sideleft_labels)
			camera_sideleft_image_pub.publish(camera_image.image)
			transform.child_frame_id = "camera_sideleft"
		
		elif camera_image.name == 5:
			#camera_image_sideright = br.imgmsg_to_cv2(camera_image.image, "bgr8")
			sideright_labels = MarkerArray()
			
			for index, label in enumerate(camera_image.labels):
				label_marker = Marker()
				label_marker.header = label.header
				label_marker.type = Marker.CUBE
				label_marker.action = Marker.ADD
				label_marker.id = index
				label_marker.pose.position.x = label.box.center_x
				label_marker.pose.position.y = label.box.center_y
				label_marker.scale.x = label.box.length
				label_marker.scale.y = label.box.width

				if label.type == 1:
					#cv2.rectangle(camera_image_sideright, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), vehicle, 2)
					label_marker.ns = "Vehicle"
				elif label.type == 2:
					#cv2.rectangle(camera_image_sideright, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), pedestrian, 2)
					label_marker.ns = "Pedestrian"
				elif label.type == 3:
					#cv2.rectangle(camera_image_sideright, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), sign, 2)
					label_marker.ns = "Sign"
				elif label.type == 4:
					#cv2.rectangle(camera_image_sideright, (int(label.box.center_x-label.box.length/2), int(label.box.center_y-label.box.width/2)), (int(label.box.center_x+label.box.length/2), int(label.box.center_y+label.box.width/2)), cyclist, 2)	
					label_marker.ns = "Cyclist"
			
				sideright_labels.markers.append(label_marker)

			camera_sideright_label_pub.publish(sideright_labels)
			camera_sideright_image_pub.publish(camera_image.image)
			transform.child_frame_id = "camera_sideright"

		global transforms
		transforms.transforms.append(transform)

	#img_front_image = br.cv2_to_imgmsg(camera_image_front)
	#img_frontleft_image = br.cv2_to_imgmsg(camera_image_frontleft)
	#img_frontright_image = br.cv2_to_imgmsg(camera_image_frontright)
	#img_sideleft_image = br.cv2_to_imgmsg(camera_image_sideleft)
	#img_sideright_image = br.cv2_to_imgmsg(camera_image_sideright)

	#camera_front_labelimage_pub.publish(img_front_image)
	#camera_frontleft_labelimage_pub.publish(img_frontleft_image)
	#camera_frontright_labelimage_pub.publish(img_frontright_image)
	#camera_sideleft_labelimage_pub.publish(img_sideleft_image)
	#camera_sideright_labelimage_pub.publish(img_sideright_image)



def lidar_pointcloud_visualization():

	lidar_top_pub.publish(lidars.lidars[0].pointcloud_ri1)
	lidar_front_pub.publish(lidars.lidars[1].pointcloud_ri1)
	lidar_left_pub.publish(lidars.lidars[2].pointcloud_ri1)
	lidar_right_pub.publish(lidars.lidars[3].pointcloud_ri1)
	lidar_rear_pub.publish(lidars.lidars[4].pointcloud_ri1)

	for lidar in lidars.lidars:
		
		transform = TransformStamped()
		transform.header = lidar.header
		transform.child_frame_id = "vehicle"
		rotation_matrix = np.array([(lidar.extrinsic[0], lidar.extrinsic[1], lidar.extrinsic[2]),(lidar.extrinsic[4], lidar.extrinsic[5], lidar.extrinsic[6]),(lidar.extrinsic[8], lidar.extrinsic[9], lidar.extrinsic[10])])
		r,p,y = rotationMatrixToEulerAngles(rotation_matrix)
		quaternion = quaternion_from_euler(r, p, y)
		transform.transform.rotation.x = quaternion[0]
		transform.transform.rotation.y = quaternion[1]
		transform.transform.rotation.z = quaternion[2]
		transform.transform.rotation.w = quaternion[3]
		transform.transform.translation.x = lidar.extrinsic[3]
		transform.transform.translation.y = lidar.extrinsic[7]
		transform.transform.translation.z = lidar.extrinsic[11]
		if lidar.name == 1:
			transform.header.frame_id = "lidar_top"
		if lidar.name == 2:
			transform.header.frame_id = "lidar_front"
		if lidar.name == 3:
			transform.header.frame_id = "lidar_left"
		if lidar.name == 4:
			transform.header.frame_id = "lidar_right"
		if lidar.name == 5:
			transform.header.frame_id = "lidar_rear"
		global transforms
		transforms.transforms.append(transform)

def lidar_label_visualization():
	### lidar label visualization     
	lidar_label = MarkerArray()

	for index, label in enumerate(lidars.lidars[0].labels):
		label_marker = Marker()
		label_marker.header = label.header
		label_marker.type = Marker.CUBE
		label_marker.action = Marker.ADD
		label_marker.id = index
		label_marker.lifetime = rospy.Duration(0.1)
		label_marker.pose.position.x = label.box.center_x
		label_marker.pose.position.y = label.box.center_y
		label_marker.pose.position.z = label.box.center_z
		quaternion = quaternion_from_euler(0, 0, label.box.heading)
		label_marker.pose.orientation.x = quaternion[0]
		label_marker.pose.orientation.y = quaternion[1]
		label_marker.pose.orientation.z = quaternion[2]
		label_marker.pose.orientation.w = quaternion[3]
		label_marker.scale.x = label.box.length
		label_marker.scale.y = label.box.width
		label_marker.scale.z = label.box.height
		label_marker.color.a = 0.5
		
		if label.type == 1 :
			label_marker.color.b = vehicle[0]
			label_marker.color.g = vehicle[1]
			label_marker.color.r = vehicle[2]
			label_marker.ns = "Vehicle"

		elif label.type == 2 :
			label_marker.color.b = pedestrian[0]
			label_marker.color.g = pedestrian[1]
			label_marker.color.r = pedestrian[2]
			label_marker.ns = "Pedestrian"

		elif label.type == 3 :
			label_marker.color.b = sign[0]
			label_marker.color.g = sign[1]
			label_marker.color.r = sign[2]
			label_marker.ns = "Sign"

		elif label.type == 4 :
			label_marker.color.b = cyclist[0]
			label_marker.color.g = cyclist[1]
			label_marker.color.r = cyclist[2]
			label_marker.ns = "Cyclist"

		lidar_label.markers.append(label_marker)
	lidar_label_pub.publish(lidar_label)

def talker():
	rospy.init_node('camera_label_visualization', anonymous=True)    
	rospy.Subscriber("camera_image", CameraImageArray, camera_image_callback)
	rospy.Subscriber("lidar_all", LidarArray, lidar_all_callback)
	rospy.Subscriber("vehicle_info",VehicleInfo, vehicle_info_callback)
	rate = rospy.Rate(1) # 10hz
	while not rospy.is_shutdown():
		
		rate.sleep()


def camera_image_callback(msg):
	print("get cameras")
	global get_camera_image
	get_camera_image = True
	global camera_images
	camera_images = msg
	camera_image_visualization()

def lidar_all_callback(msg):
	print("get lidars")
	global get_lidars
	get_lidars = True
	global lidars
	lidars = msg
	lidar_pointcloud_visualization()
	#if len(lidars.lidars[0].labels) is not 0:
	lidar_label_visualization()

def vehicle_info_callback(msg):
	print("get vehicle info")
	global get_vehicle
	get_vehicle = True
	global vehicle_info
	vehicle_info = msg

	transform = TransformStamped()
	transform.header = vehicle_info.header
	rotation_matrix = np.array([(vehicle_info.pose[0], vehicle_info.pose[1], vehicle_info.pose[2]),(vehicle_info.pose[4], vehicle_info.pose[5], vehicle_info.pose[6]),(vehicle_info.pose[8], vehicle_info.pose[9], vehicle_info.pose[10])])
	r,p,y = rotationMatrixToEulerAngles(rotation_matrix)
	quaternion = quaternion_from_euler(r, p, y)
	transform.transform.rotation.x = quaternion[0]
	transform.transform.rotation.y = quaternion[1]
	transform.transform.rotation.z = quaternion[2]
	transform.transform.rotation.w = quaternion[3]
	transform.transform.translation.x = vehicle_info.pose[3]
	transform.transform.translation.y = vehicle_info.pose[7]
	transform.transform.translation.z = vehicle_info.pose[11]
	transform.child_frame_id = "global"
	global transforms
	transforms.transforms.append(transform)
	tf_pub.publish(transforms)

if __name__ == '__main__': 
	try:
		talker()
	except  rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()

