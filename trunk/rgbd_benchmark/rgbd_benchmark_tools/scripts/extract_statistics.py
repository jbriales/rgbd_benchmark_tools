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
import cb_detector
from pgfutils import *
from evaluate import *


if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script reads a file and creates a pgf plot.
    ''')
    parser.add_argument('groundtruth', help='groundtruth trajectory file (format: timestamp x y z qx qy qz qw)')
    args = parser.parse_args()
    
    traj_gt = read_trajectory(args.groundtruth)
    
    print "axis range"
    keys = traj_gt.keys();
    keys.sort();
    for axis in range(3):
        values = [traj_gt[t][axis,3] for t in keys]
        #print values
        print axis, numpy.max(values)-numpy.min(values)
        
    delta = 1
    motion_trans = []
    for t_0 in keys:
        if t_0+delta > keys[-1]: 
            break
        t_1 = find_closest_stamp(keys,t_0 + delta)
        
        motion44 = ominus( traj_gt[t_1], traj_gt[t_0] )
        xyz = tuple(tf.transformations.translation_from_matrix(motion44))[:3]
        quat = tuple(tf.transformations.quaternion_from_matrix(motion44))
        
        motion_trans.append( numpy.linalg.norm(xyz) )
    print
    print "average speed (m/s): ",numpy.mean(motion_trans)
        
    