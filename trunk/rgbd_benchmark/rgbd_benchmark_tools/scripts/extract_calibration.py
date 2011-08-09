#!/usr/bin/python

import roslib; roslib.load_manifest('rgbd_benchmark_tools')
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
    This scripts reads a bag file and outputs the TF transform from /mocap to /openni_rgb_optical_frame.
    ''')
    parser.add_argument('calibration', help='calibration tf (contains any missing/required tfs)')
    args = parser.parse_args()

    if args.calibration:
        calbag = rosbag.Bag(args.calibration,'r')
        for topic, msg, t in calbag.read_messages(topics=["/tf"]):
            print msg

