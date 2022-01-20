#!/usr/bin/env python

import cv2
import numpy as np
from geopy import quart_to_rpy, euler_to_quaternion, euclid_distance
import rospy
import rospkg
import message_filters
from cv_bridge import CvBridge
# message
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Point
from water_msgs.msg import ShipState
# sensor data message
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

import os
import _thread
import math
import time
basedir = os.path.abspath(os.path.dirname(__file__))


def get_model_state(name):
    global g_get_state
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "world"
    try:
        model_state = g_get_state(model_name=name)
    except Exception as e:
        rospy.logerr('Error on calling service: %s', str(e))
        return
    latest_state = ShipState()
    latest_state.header = header
    latest_state.modelstate = model_state
    return latest_state


def label():
    ships_state = dict()
    ships_type = rospy.get_param(f"ships/total_ships_type")
    # first get all model states
    my_ship_state = get_model_state("wamv")
    my_x = my_ship_state.modelstate.pose.position.x
    my_y = my_ship_state.modelstate.pose.position.y
    for ship_type in ships_type:
        total_ships_name = rospy.get_param(
            f"ships/{ship_type}/ships_name")
        for ship_name in total_ships_name:
            ships_state[ship_name] = get_model_state(ship_name)
    # remove far models
    for obj_ship_name, obj_ship_state in list(ships_state.items()):
        obj_x = obj_ship_state.modelstate.pose.position.x
        obj_y = obj_ship_state.modelstate.pose.position.y
        if euclid_distance(my_x, my_y, obj_x, obj_y) > 100:
            del ships_state[obj_ship_name]
            continue
    print(f"my state: ({my_x},{my_y})")
    print(f"left number: {len(ships_state)}")
    print(ships_state)
    # second get all model states and collect sensor data


def collect():
    pass

# def callback(imageL, imageR, pc2_data):
#     global shotting
#     if shotting is False:
#         return

#     vely_pc = point_cloud2.read_points(pc2_data, field_names=("x", "y", "z"), skip_nans=True)

#     print type(gen)
#     # for p in gen:
#     #   print " x : %.3f  y: %.3f  z: %.3f" % (p[0], p[1], p[2])
#     time.sleep(1)


def callback(rgb_msg, camera_info):
    rospy.loginfo("camera collecting time: %s", rgb_msg.header.stamp.to_sec())
    rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
    camera_info_K = np.array(camera_info.K).reshape([3, 3])
    camera_info_D = np.array(camera_info.D)
    rgb_undist = cv2.undistort(rgb_image, camera_info_K, camera_info_D)

    # # info show
    # print("camera params:")
    # print(camera_info_K)
    # print(camera_info_D)
    cv2.imshow("image show", rgb_undist)
    cv2.waitKey(1)

    label()
    print("read")


def ros_init():
    rospy.wait_for_service("/gazebo/get_model_state")
    global g_get_state
    g_get_state = rospy.ServiceProxy(
        "/gazebo/get_model_state", GetModelState)

    image_L_sub = message_filters.Subscriber(
        '/wamv/sensors/cameras/front_left_camera/image_raw', Image)
    # image_R_sub = message_filters.Subscriber(
    #     '/wamv/sensors/cameras/front_right_camera/image_raw', Image)
    info_L_sub = message_filters.Subscriber(
        '/wamv/sensors/cameras/front_left_camera/camera_info', CameraInfo)
    # info_R_sub = message_filters.Subscriber(
    #     '/wamv/sensors/cameras/front_right_camera/camera_info', CameraInfo)

    # veledyne_sub = message_filters.Subscriber(
    #     '/my_camera/depth/points', PointCloud2, callback_pointcloud)
    # livox_sub = message_filters.Subscriber(
    #     '/scan', PointCloud2, callback_pointcloud)

    # publish which sensors
    ts = message_filters.TimeSynchronizer([image_L_sub, info_L_sub], 10)
    ts.registerCallback(callback)


if __name__ == '__main__':
    rospy.init_node('collect_and_label', anonymous=True)
    ros_init()

    rospy.spin()

    # closing all open windows
    cv2.destroyAllWindows()
