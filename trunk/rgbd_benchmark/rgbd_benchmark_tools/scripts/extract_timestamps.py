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
    This scripts reads a bag file and outputs time stamps.
    ''')
    parser.add_argument('inputbag', help='input bag file')
    args = parser.parse_args()

    inbag = rosbag.Bag(args.inputbag,'r')
    ref = rospy.Time(0)
    for topic, msg, t in inbag.read_messages():
        if topic=="/tf":
            print
            print "Time: %0.3fs (%+0.3fs) ================================="%(t.to_sec(),(t-ref).to_sec())
            ref = t
        print topic,
        if topic[:7]=="/camera" or topic=="/imu":
            print "%0.3fs"%(msg.header.stamp-ref).to_sec()
        elif topic=="/tf":
            print "\n".join(["\t%s to %s: %0.3fs"%(m.header.frame_id,m.child_frame_id,(m.header.stamp-ref).to_sec()) for m in msg.transforms])
        elif topic=="/cortex_marker_array":
            print "\n".join(["\t%s: %0.3fs"%(m.ns,(m.header.stamp-ref).to_sec()) for m in msg.markers])
        else:
            print "[no timestamp/unknown message]"

