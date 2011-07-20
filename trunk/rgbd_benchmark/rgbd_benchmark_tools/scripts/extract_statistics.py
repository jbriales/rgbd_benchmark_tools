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
        print "%s-axis: %0.2fm"%(["x","y","z"][axis],numpy.max(values)-numpy.min(values))
        
    delta = 1
    motion_trans = []
    motion_rot = []
    for t_0 in keys:
        if t_0+delta > keys[-1]: 
            break
        t_1 = find_closest_stamp(keys,t_0 + delta)
        
        motion44 = ominus( traj_gt[t_1], traj_gt[t_0] )
        xyz = tuple(tf.transformations.translation_from_matrix(motion44))[:3]
        quat = tuple(tf.transformations.quaternion_from_matrix(motion44))
        
        motion_trans.append( numpy.linalg.norm(xyz) )
        
        # an invitation to 3-d vision, p 27
        motion_rot.append( numpy.arccos( (numpy.trace(motion44[0:3,0:3]) - 1)/2) )
        
    avg_delta = numpy.mean([a-b for a,b in zip(keys[1:],keys[:-1])])
        
    print
    print "average groundtruth frequency: %0.3fHz"%(1/avg_delta)
    print "average translational velocity: %0.3fm/s"%numpy.mean(motion_trans)
    print "average angular velocity: %0.3frad/s (%0.3fdeg/s)"%(
                                                         numpy.mean(motion_rot),
                                                         numpy.mean(motion_rot) / numpy.pi * 180)
    print
    print "trajectory length, translational motion: %0.3fm"%(numpy.sum(motion_trans)*avg_delta)
    print "trajectory length, rotational motion: %0.3frad (%0.3fdeg)"%(
                                                         numpy.sum(motion_rot)*avg_delta,
                                                         numpy.sum(motion_rot) / numpy.pi * 180*avg_delta)
        
    