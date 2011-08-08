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
import cb_detector

parser = argparse.ArgumentParser(description='''
This script reads a bagfile, synchronizes the timestamps of the mocap and corrects the timestmaps in the mocap.
''')
parser.add_argument('inputbag', help='input bag file')
parser.add_argument('outputbag', nargs='?',help='output bag file')
parser.add_argument('--delay', help='time offset between files',default=0.00)
args = parser.parse_args()

if not args.outputbag:
    args.outputbag = os.path.splitext(args.inputbag)[0] + "-synced.bag"


print "Processing bag file:"
print "  in:",args.inputbag
print "  out:",args.outputbag
print "  cortex delay:",args.delay

buffer={}

with rosbag.Bag(args.outputbag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(args.inputbag).read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            buffer[(msg.transforms[0].header.stamp,topic)] = msg
        elif topic == "/cortex_marker_array" and msg.markers:
            for marker in msg.markers:
                marker.header.stamp = marker.header.stamp + rospy.Duration.from_sec(float(args.delay))
            buffer[(msg.markers[0].header.stamp,topic)] = msg 
        else:
            buffer[(msg.header.stamp if msg._has_header else t,topic)] = msg 

    stamps = buffer.keys()
    stamps.sort()
    for (t,topic) in stamps:
        #print t,topic             
        outbag.write(topic, buffer[(t,topic)], t)
            
print "Done!"
