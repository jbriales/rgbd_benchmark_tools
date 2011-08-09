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
    parser.add_argument('--start', help='skip the first N seconds of  input bag file')
    parser.add_argument('--duration', help='only process N seconds of  input bag file')
    parser.add_argument('inputbag', help='input bag file')
    args = parser.parse_args()

    
#    print "Processing bag file:"
#    print "  in:",args.inputbag
#    print
    
    
    if not args.start:
        args.start = 0
        
    if args.duration:
        print "  duration: %s seconds"%(args.duration)
    
    inbag = rosbag.Bag(args.inputbag,'r')
    time_start = None
    for topic, msg, t in inbag.read_messages(topics=["/tf"]):
        if time_start==None:
            time_start=t
        if t - time_start < rospy.Duration.from_sec(float(args.start)):
            continue
        if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))):
            break
        print "t=%f\r"%(t-time_start).to_sec(),
        if topic=="/tf":
            tf_buffer={}
            for transform in msg.transforms:
                tf_buffer[ (transform.header.frame_id,transform.child_frame_id) ] = transform
                logtime = transform.header.stamp
            try:
                print tf_buffer[("/Kinect","/openni_camera")]
            except KeyError:
                print "waiting for tf"
                continue
            except:
                raise

