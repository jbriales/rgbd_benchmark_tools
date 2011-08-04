#!/usr/bin/python

import roslib; roslib.load_manifest('kinect_bagtools')
import rospy
import rosbag
import sensor_msgs.msg
import argparse
import sys
import os
import cv
import struct
import array
import tf
import numpy
import numpy.linalg.linalg
import geometry_msgs
import copy

def transform44(ts):
    return numpy.dot(tf.listener.xyz_to_mat44(ts.transform.translation), tf.listener.xyzw_to_mat44(ts.transform.rotation))


if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This scripts reads a bag file and outputs the intrinsic camera parameters.
    ''')
    parser.add_argument('bag', help='bag file (contains /camera/rgb/camera_info')
    args = parser.parse_args()

    calbag = rosbag.Bag(args.bag,'r')
    for topic, msg, t in calbag.read_messages(topics=["/camera/rgb/camera_info"]):
        print msg

