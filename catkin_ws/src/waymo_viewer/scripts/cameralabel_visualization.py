#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import math

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

br = CvBridge()

vehicle = (0, 0, 255)
pedestrian = (0, 255, 0)
cyclist = (255, 0, 0)
sign = (0, 255, 255)
front_labels = MarkerArray()
frontleft_labels = MarkerArray()
frontright_labels = MarkerArray()
sideleft_labels = MarkerArray()
sideright_labels = MarkerArray()
front_image = Image()
frontleft_image = Image()
frontright_image = Image()
sideleft_image = Image()
sideright_image = Image()
get_front_image = False
get_frontleft_image = False
get_frontright_image = False
get_sideleft_image = False
get_sideright_image = False

camera_front_labelimage_pub = rospy.Publisher("camera_labelimage/front", Image, queue_size = 1000)
camera_frontleft_labelimage_pub = rospy.Publisher("camera_labelimage/frontleft", Image, queue_size = 1000)
camera_frontright_labelimage_pub = rospy.Publisher("camera_labelimage/frontright", Image, queue_size = 1000)
camera_sideleft_labelimage_pub = rospy.Publisher("camera_labelimage/sideleft", Image, queue_size = 1000)
camera_sideright_labelimage_pub = rospy.Publisher("camera_labelimage/sideright", Image, queue_size = 1000)

def camera_image_visualization():
	
	camera_image_front = br.imgmsg_to_cv2(front_image, "bgr8")			
	for index, label in enumerate(front_labels.markers):				
		if label.ns == "Vehicle":
			cv2.rectangle(camera_image_front, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), vehicle, 2)
		elif label.ns == "Pedestrian":
			cv2.rectangle(camera_image_front, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), pedestrian, 2)
		elif label.ns == "Sign":
			cv2.rectangle(camera_image_front, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), sign, 2)
		elif label.ns == "Cyclist":
			cv2.rectangle(camera_image_front, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), cyclist, 2)
	front_resize = cv2.resize(camera_image_front, (camera_image_front.shape[1]/4, camera_image_front.shape[0]/4), interpolation=cv2.INTER_CUBIC)
		
	camera_image_frontleft = br.imgmsg_to_cv2(frontleft_image, "bgr8")			
	for index, label in enumerate(frontleft_labels.markers):
		if label.ns == "Vehicle":
			cv2.rectangle(camera_image_frontleft, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), vehicle, 2)
		elif label.ns == "Pedestrian":
			cv2.rectangle(camera_image_frontleft, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), pedestrian, 2)
		elif label.ns == "Sign":
			cv2.rectangle(camera_image_frontleft, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), sign, 2)
		elif label.ns == "Cyclist":
			cv2.rectangle(camera_image_frontleft, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), cyclist, 2)
	frontleft_resize = cv2.resize(camera_image_frontleft, (camera_image_frontleft.shape[1]/4, camera_image_frontleft.shape[0]/4), interpolation=cv2.INTER_CUBIC)
	
	camera_image_frontright = br.imgmsg_to_cv2(frontright_image, "bgr8")			
	for index, label in enumerate(frontright_labels.markers):
		if label.ns == "Vehicle":
			cv2.rectangle(camera_image_frontright, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), vehicle, 2)
		elif label.ns == "Pedestrian":
			cv2.rectangle(camera_image_frontright, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), pedestrian, 2)
		elif label.ns == "Sign":
			cv2.rectangle(camera_image_frontright, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), sign, 2)
		elif label.ns == "Cyclist":
			cv2.rectangle(camera_image_frontright, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), cyclist, 2)
	frontright_resize = cv2.resize(camera_image_frontright, (camera_image_frontright.shape[1]/4, camera_image_frontright.shape[0]/4), interpolation=cv2.INTER_CUBIC)

	camera_image_sideleft = br.imgmsg_to_cv2(sideleft_image, "bgr8")			
	for index, label in enumerate(sideleft_labels.markers):
		if label.ns == "Vehicle":
			cv2.rectangle(camera_image_sideleft, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), vehicle, 2)
		elif label.ns == "Pedestrian":
			cv2.rectangle(camera_image_sideleft, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), pedestrian, 2)
		elif label.ns == "Sign":
			cv2.rectangle(camera_image_sideleft, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), sign, 2)
		elif label.ns == "Cyclist":
			cv2.rectangle(camera_image_sideleft, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), cyclist, 2)	
	sideleft_resize = cv2.resize(camera_image_sideleft, (camera_image_sideleft.shape[1]/4, camera_image_sideleft.shape[0]/4), interpolation=cv2.INTER_CUBIC)
		

	camera_image_sideright = br.imgmsg_to_cv2(sideright_image, "bgr8")			
	for index, label in enumerate(sideright_labels.markers):
		if label.ns == "Vehicle":
			cv2.rectangle(camera_image_sideright, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), vehicle, 2)
		elif label.ns == "Pedestrian":
			cv2.rectangle(camera_image_sideright, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), pedestrian, 2)
		elif label.ns == "Sign":
			cv2.rectangle(camera_image_sideright, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), sign, 2)
		elif label.ns == "Cyclist":
			cv2.rectangle(camera_image_sideright, (int(label.pose.position.x-label.scale.x/2), int(label.pose.position.y-label.scale.y/2)), (int(label.pose.position.x+label.scale.x/2), int(label.pose.position.y+label.scale.y/2)), cyclist, 2)	
	sideright_resize = cv2.resize(camera_image_sideright, (camera_image_sideright.shape[1]/4, camera_image_sideright.shape[0]/4), interpolation=cv2.INTER_CUBIC)
	
	img_front_image = br.cv2_to_imgmsg(front_resize)
	img_frontleft_image = br.cv2_to_imgmsg(frontleft_resize)
	img_frontright_image = br.cv2_to_imgmsg(frontright_resize)
	img_sideleft_image = br.cv2_to_imgmsg(sideleft_resize)
	img_sideright_image = br.cv2_to_imgmsg(sideright_resize)

	camera_front_labelimage_pub.publish(img_front_image)
	camera_frontleft_labelimage_pub.publish(img_frontleft_image)
	camera_frontright_labelimage_pub.publish(img_frontright_image)
	camera_sideleft_labelimage_pub.publish(img_sideleft_image)
	camera_sideright_labelimage_pub.publish(img_sideright_image)

	global front_labels
	del front_labels.markers[:]
	global frontleft_labels
	del frontleft_labels.markers[:]
	global frontright_labels
	del frontright_labels.markers[:]
	global sideleft_labels
	del sideleft_labels.markers[:]
	global sideright_labels
	del sideright_labels.markers[:]


def talker():
	rospy.init_node('camera_label_visualization', anonymous=True)    
	rospy.Subscriber("camera_image/front", Image, camera_front_image_callback)
	rospy.Subscriber("camera_image/frontleft", Image, camera_frontleft_image_callback)
	rospy.Subscriber("camera_image/frontright", Image, camera_frontright_image_callback)
	rospy.Subscriber("camera_image/sideleft", Image, camera_sideleft_image_callback)
	rospy.Subscriber("camera_image/sideright", Image, camera_sideright_image_callback)
	rospy.Subscriber("/camera_label/front", MarkerArray, camera_front_label_callback)
	rospy.Subscriber("/camera_label/frontleft", MarkerArray, camera_frontleft_label_callback)
	rospy.Subscriber("/camera_label/frontright", MarkerArray, camera_frontright_label_callback)
	rospy.Subscriber("/camera_label/sideleft", MarkerArray, camera_sideleft_label_callback)
	rospy.Subscriber("/camera_label/sideright", MarkerArray, camera_sideright_label_callback)

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():

		if get_front_image and get_frontleft_image and get_frontright_image and get_sideleft_image and get_sideright_image:
			camera_image_visualization()
		
		rate.sleep()


def camera_front_image_callback(msg):
	global get_front_image
	get_front_image = True
	global front_image
	front_image = msg
def camera_frontleft_image_callback(msg):
	global get_frontleft_image
	get_frontleft_image = True
	global frontleft_image
	frontleft_image = msg
def camera_frontright_image_callback(msg):
	global get_frontright_image
	get_frontright_image = True
	global frontright_image
	frontright_image = msg
def camera_sideleft_image_callback(msg):
	global get_sideleft_image
	get_sideleft_image = True
	global sideleft_image
	sideleft_image = msg
def camera_sideright_image_callback(msg):
	global get_sideright_image
	get_sideright_image = True
	global sideright_image
	sideright_image = msg
def camera_front_label_callback(msg):
	global front_labels
	front_labels = msg
def camera_frontleft_label_callback(msg):
	global frontleft_labels
	frontleft_labels = msg
def camera_frontright_label_callback(msg):
	global frontright_labels
	frontright_labels = msg
def camera_sideleft_label_callback(msg):
	global sideleft_labels
	sideleft_labels = msg
def camera_sideright_label_callback(msg):
	global sideright_labels
	sideright_labels = msg


if __name__ == '__main__': 
	try:
		talker()
	except  rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()

