#!/usr/bin/env python

import rospy
import rosbag
import csv
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
from waymo_viewer.msg import Label
from waymo_viewer.msg import CameraImageArray
from waymo_viewer.msg import CameraImage
from waymo_viewer.msg import Lidar
from waymo_viewer.msg import LidarArray
from waymo_viewer.msg import VehicleInfo

import math
import os
import imp
import tensorflow as tf
#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import math
import numpy as np
import itertools
import matplotlib.pyplot as plt
import pptk as pp

# TODO: Change this to your own setting
os.environ['PYTHONPATH']='/env/python:../../../../waymo-od'
m = imp.find_module('waymo_open_dataset', ['../../../../waymo-od'])
imp.load_module('waymo_open_dataset', m[0], m[1], m[2])

from waymo_open_dataset.utils import range_image_utils
from waymo_open_dataset.utils import transform_utils
from waymo_open_dataset import dataset_pb2 as open_dataset

#tf.enable_eager_execution()

def parse_range_image_and_camera_projection(frame):
    """Parse range images and camera projections given a frame.

    Args:
       frame: open dataset frame proto
    Returns:
       range_images: A dict of {laser_name,
         [range_image_first_return, range_image_second_return]}.
       camera_projections: A dict of {laser_name,
         [camera_projection_from_first_return,
          camera_projection_from_second_return]}.
      range_image_top_pose: range image pixel pose for top lidar.
    """
    range_images = {}
    camera_projections = {}
    range_image_top_pose = None

    for laser in frame.lasers:

        if len(laser.ri_return1.range_image_compressed) > 0:
            range_image_str_tensor = tf.io.decode_compressed(laser.ri_return1.range_image_compressed, 'ZLIB')
            ri = open_dataset.MatrixFloat()
            ri.ParseFromString(bytearray(range_image_str_tensor.numpy()))
            range_images[laser.name] = [ri]

            if laser.name == open_dataset.LaserName.TOP:
                range_image_top_pose_str_tensor = tf.io.decode_compressed(laser.ri_return1.range_image_pose_compressed, 'ZLIB')
                range_image_top_pose = open_dataset.MatrixFloat()
                range_image_top_pose.ParseFromString(bytearray(range_image_top_pose_str_tensor.numpy()))

            camera_projection_str_tensor = tf.io.decode_compressed(laser.ri_return1.camera_projection_compressed, 'ZLIB')
            cp = open_dataset.MatrixInt32()
            cp.ParseFromString(bytearray(camera_projection_str_tensor.numpy()))
            camera_projections[laser.name] = [cp]

        if len(laser.ri_return2.range_image_compressed) > 0:
            range_image_str_tensor = tf.io.decode_compressed(laser.ri_return2.range_image_compressed, 'ZLIB')
            ri = open_dataset.MatrixFloat()
            ri.ParseFromString(bytearray(range_image_str_tensor.numpy()))
            range_images[laser.name].append(ri)

            camera_projection_str_tensor = tf.io.decode_compressed(laser.ri_return2.camera_projection_compressed, 'ZLIB')
            cp = open_dataset.MatrixInt32()
            cp.ParseFromString(bytearray(camera_projection_str_tensor.numpy()))
            camera_projections[laser.name].append(cp)


    return range_images, camera_projections, range_image_top_pose

def convert_range_image_to_point_cloud(frame, range_images, camera_projections, range_image_top_pose, ri_index = 0):
    """Convert range images to point cloud.

    Args:
        frame: open dataset frame
        range_images: A dict of {laser_name, [range_image_first_return, range_image_second_return]}. camera_projections: A dict of {laser_name,
        [camera_projection_from_first_return, camera_projection_from_second_return]}.
        range_image_top_pose: range image pixel pose for top lidar.
        ri_index: 0 for the first return, 1 for the second return.
    Returns:
        points: {[N, 3]} list of 3d lidar points of length 5 (number of lidars).
        cp_points: {[N, 6]} list of camera projections of length 5 (number of lidars).
    """
    calibrations = sorted(frame.context.laser_calibrations, key=lambda c: c.name)
    lasers = sorted(frame.lasers, key=lambda laser: laser.name)
    points = [] 
    cp_points = []

    frame_pose = tf.convert_to_tensor(np.reshape(np.array(frame.pose.transform), [4, 4]))
    # [H, W, 6]
    range_image_top_pose_tensor = tf.reshape(tf.convert_to_tensor(range_image_top_pose.data), range_image_top_pose.shape.dims)
    # [H, W, 3, 3]
    range_image_top_pose_tensor_rotation = transform_utils.get_rotation_matrix(range_image_top_pose_tensor[..., 0], range_image_top_pose_tensor[..., 1],range_image_top_pose_tensor[..., 2])
    range_image_top_pose_tensor_translation = range_image_top_pose_tensor[..., 3:]
    range_image_top_pose_tensor = transform_utils.get_transform(range_image_top_pose_tensor_rotation,range_image_top_pose_tensor_translation)
    for c in calibrations:
        range_image = range_images[c.name][ri_index]

        if len(c.beam_inclinations) == 0:
            beam_inclinations = range_image_utils.compute_inclination(tf.constant([c.beam_inclination_min, c.beam_inclination_max]),height=range_image.shape.dims[0])
        else:
            beam_inclinations = tf.constant(c.beam_inclinations)

        beam_inclinations = tf.reverse(beam_inclinations, axis=[-1])
        extrinsic = np.reshape(np.array(c.extrinsic.transform), [4, 4])

        range_image_tensor = tf.reshape(tf.convert_to_tensor(range_image.data), range_image.shape.dims)
        pixel_pose_local = None
        frame_pose_local = None

        if c.name == open_dataset.LaserName.TOP:
            pixel_pose_local = range_image_top_pose_tensor
            pixel_pose_local = tf.expand_dims(pixel_pose_local, axis=0)
            frame_pose_local = tf.expand_dims(frame_pose, axis=0)
        range_image_mask = range_image_tensor[..., 0] > 0
        range_image_cartesian = range_image_utils.extract_point_cloud_from_range_image(tf.expand_dims(range_image_tensor[..., 0], axis=0),tf.expand_dims(extrinsic, axis=0),tf.expand_dims(tf.convert_to_tensor(beam_inclinations), axis=0),pixel_pose=pixel_pose_local,frame_pose=frame_pose_local)

        range_image_cartesian = tf.squeeze(range_image_cartesian, axis=0)
        points_tensor = tf.gather_nd(range_image_cartesian,tf.where(range_image_mask))

        cp = camera_projections[c.name][0]
        cp_tensor = tf.reshape(tf.convert_to_tensor(cp.data), cp.shape.dims)
        cp_points_tensor = tf.gather_nd(cp_tensor, tf.where(range_image_mask))
        points.append(points_tensor.numpy())
        cp_points.append(cp_points_tensor.numpy())

    return points, cp_points

def find_camera_label(name, camera_labels):
    for camera_label in camera_labels:
        if camera_label.name == name:
            labels = camera_label.labels
            break
    return labels

def find_camera_calibration(name, camera_calibrations):
    for calibrations in camera_calibrations:
    	if calibrations.name == name:
    		calibration = calibrations
    		break
    return calibration

def find_lidar_calibration(name, laser_calibrations):
    for calibrations in laser_calibrations:
        if calibrations.name == name:
            calibration = calibrations
            break
    return calibration

camera_images_pub = rospy.Publisher("camera_image", CameraImageArray, queue_size = 1000)
lidar_pub = rospy.Publisher("lidar_all", LidarArray, queue_size = 1000)
vehicle_pub = rospy.Publisher("vehicle_info", VehicleInfo, queue_size = 1000)

def main():

    try:
        # Read one file

        rospy.init_node("waymo_viewer", anonymous = True)
        #folderpath = '/media/user/0791d482-2911-43ec-912f-58dc32561470/Waymo_Dataset/file/training_0007'
        folderpath = '../../../../waymo-data/training_0012'
        files = os.listdir(folderpath)
        #csvfile = open("record.csv", "w")
        for f in files:
            fullpath = os.path.join(folderpath, f)
            if os.path.isfile(fullpath):
                FILENAME = fullpath
                dataset = tf.data.TFRecordDataset(FILENAME, compression_type = '')
                #name = FILENAME.replace("/media/user/0791d482-2911-43ec-912f-58dc32561470/Waymo_Dataset/file/training_0007/","").replace(".tfrecord","")
                #print(name)
                


                lidars = LidarArray()
                camera_images = CameraImageArray()
                vehicle_info = VehicleInfo()

                header = Header()
                header.frame_id = '/vehicle'

                # Read frame
                #raw_input("Press Enter to continue...")
                for data in dataset:
                    frame = open_dataset.Frame()
                    frame.ParseFromString(bytearray(data.numpy()))

                    header.stamp.secs = int(frame.timestamp_micros / 1000000)
                    header.stamp.nsecs = int((frame.timestamp_micros / 1000000.0 - header.stamp.secs)*1000000000)
                    print "time stamp : ", frame.timestamp_micros
                    camera_images.header = header
                    lidars.header = header
                    vehicle_info.header = header

                    lidars.lidars=[]
                    camera_images.camera_images=[]

                    vehicle_info.pose = frame.pose.transform
                    vehicle_pub.publish(vehicle_info)

                    #global csvfile
                    #csvfile.write(name+","+str(frame.timestamp_micros)+","+str(frame.context.stats.location)+"\n")

                    
                    ### publish camera image and label
                    for image in frame.images:
                        camera_image = CameraImage()
                        camera_image.header = header
                        camera_image.name = image.name
                        camera_image.shutter = image.shutter
                        camera_image.camera_trigger_time = image.camera_trigger_time
                        camera_image.camera_readout_done_time = image.camera_readout_done_time

                        """
                        labels = find_camera_label(image.name, frame.camera_labels)
                        for original_label in labels:
                            label = Label()
                            label.header = header
                            label.box.center_x = original_label.box.center_x
                            label.box.center_y = original_label.box.center_y
                            label.box.width = original_label.box.width
                            label.box.length = original_label.box.length
                            label.type = original_label.type
                            label.id = original_label.id
                            camera_image.labels.append(label)
                        """
                        
                        calibration = find_camera_calibration(image.name, frame.context.camera_calibrations)
                        camera_image.image.header = header
                        camera_image.image.height = calibration.height
                        camera_image.image.width = calibration.width
                        camera_image.image.encoding ='rgb8'
                        camera_image.image.data = np.array(tf.image.decode_jpeg(image.image)).tostring()
                        cv2.imwrite("1.jpg", np.array(tf.image.decode_jpeg(image.image)))
                        camera_image.image.step = calibration.width*3
                    
                        camera_image.intrinsic = calibration.intrinsic
                        camera_image.extrinsic = calibration.extrinsic.transform
                        camera_image.rolling_shutter_direction = calibration.rolling_shutter_direction

                        camera_images.camera_images.append(camera_image)               

                    camera_images_pub.publish(camera_images) 


                    ### lidar visualization
                    (range_images, camera_projections, range_image_top_pose) = parse_range_image_and_camera_projection(frame)
                    # Calculate the point cloud
                    points, cp_points = convert_range_image_to_point_cloud(frame, range_images, camera_projections,range_image_top_pose)
                    #points_ri2, cp_points_ri2 = convert_range_image_to_point_cloud(frame, range_images, camera_projections, range_image_top_pose, ri_index=1)


                    for index, lidar_points in enumerate(points):

                        lidar = Lidar()
                        lidar.header = header
                        lidar.name = index+1
                        lidar.pointcloud_ri1.header = header


                        for point in points[index]:
                            lidar.pointcloud_ri1.points.append(Point32(point[0], point[1], point[2]))

                        calibration = find_lidar_calibration(lidar.name, frame.context.laser_calibrations)
                        lidar.extrinsic = calibration.extrinsic.transform
                        lidar.beam_inclination_min = calibration.beam_inclination_min
                        lidar.beam_inclination_max = calibration.beam_inclination_max

                        if lidar.name == 1:
                            for laser_label in frame.laser_labels:
                                label = Label()
                                label.header = header
                                label.type = laser_label.type
                                label.id = laser_label.id
                                label.box.center_x = laser_label.box.center_x
                                label.box.center_y = laser_label.box.center_y
                                label.box.center_z = laser_label.box.center_z
                                label.box.heading = laser_label.box.heading
                                label.box.length = laser_label.box.length
                                label.box.width = laser_label.box.width
                                label.box.height = laser_label.box.height
                                lidar.labels.append(label)
                            lidar.beam_inclinations = calibration.beam_inclinations

                        lidars.lidars.append(lidar)

                    lidar_pub.publish(lidars)
                    break

                print(FILENAME)                   

                rospy.sleep(0.01)
        #csvfile.close()

        
    except rospy.ROSInterruptException:
        pass

main()  # Call main function
