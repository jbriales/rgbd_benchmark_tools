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
    output = []
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script reads a file and creates a pgf plot.
    ''')
    parser.add_argument('groundtruth', help='groundtruth trajectory file (format: timestamp x y z qx qy qz qw)')
    args = parser.parse_args()
    
    traj_gt = read_trajectory(args.groundtruth)
    
    keys = traj_gt.keys();
    keys.sort();
    for axis in range(3):
        values = [traj_gt[t][axis,3] for t in keys]
        #print values
        output.append("dimensions.%s=%0.2fm"%(["x","y","z"][axis],numpy.max(values)-numpy.min(values)))
        
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
        
    
    
    median_groundtruth_freq = 1/numpy.median([a-b for a,b in zip(keys[1:],keys[:-1])])
    output.append("groundtruth_freq.median=%0.2fHz"%median_groundtruth_freq)
    
    avg_groundtruth_freq = 1/numpy.mean([a-b for a,b in zip(keys[1:],keys[:-1])])
    output.append("groundtruth_freq.mean=%0.2fHz"%avg_groundtruth_freq)
    
    duration = keys[-1]-keys[0]
    output.append("duration=%0.2fs"%duration)

    duration_groundtruth = 1/median_groundtruth_freq * len(keys)
    output.append("duration.with_groundtruth=%0.2fs"%duration_groundtruth)

    output.append("translational_velocity.mean=%0.3fm/s"%numpy.mean(motion_trans))
    output.append("translational_velocity.median=%0.3fm/s"%numpy.median(motion_trans))
    output.append("translational_velocity.min=%0.3fm/s"%numpy.min(motion_trans))
    output.append("translational_velocity.max=%0.3fm/s"%numpy.max(motion_trans))
    #output.append("translational_velocity.p5=%0.3fm/s"%numpy.percentile(motion_trans,5))
    #output.append("translational_velocity.p95=%0.3fm/s"%numpy.percentile(motion_trans,95))

    output.append("angular_velocity.mean=%0.3frad/s"%(numpy.mean(motion_rot)))
    output.append("angular_velocity.median=%0.3frad/s"%(numpy.median(motion_rot)))
    output.append("angular_velocity.min=%0.3frad/s"%(numpy.min(motion_rot)))
    output.append("angular_velocity.max=%0.3frad/s"%(numpy.max(motion_rot)))
    #output.append("angular_velocity.p5=%0.3frad/s"%(numpy.percentile(motion_rot,5)))
    #output.append("angular_velocity.p95=%0.3frad/s"%(numpy.percentile(motion_rot,95)))

    output.append("angular_velocity.mean.deg=%0.3fdeg/s"%(numpy.mean(motion_rot)/numpy.pi*180))
    output.append("angular_velocity.median.deg=%0.3fdeg/s"%(numpy.median(motion_rot)/numpy.pi*180))
    output.append("angular_velocity.min.deg=%0.3fdeg/s"%(numpy.min(motion_rot)/numpy.pi*180))
    output.append("angular_velocity.max.deg=%0.3fdeg/s"%(numpy.max(motion_rot)/numpy.pi*180))
    #output.append("angular_velocity.p5.deg=%0.3fdeg/s"%(numpy.percentile(motion_rot,5)/numpy.pi*180))
    #output.append("angular_velocity.p95.deg=%0.3fdeg/s"%(numpy.percentile(motion_rot,95)/numpy.pi*180))

    output.append("trajectory_length.translational=%0.3fm"%(numpy.sum(motion_trans)/avg_groundtruth_freq))    
    output.append("trajectory_length.rotational=%0.3frad/s"%(numpy.sum(motion_rot)/avg_groundtruth_freq))
    output.append("trajectory_length.rotational.deg=%0.3fdeg/s"%(numpy.sum(motion_rot) / numpy.pi * 180/avg_groundtruth_freq))
                  
    output = "\n".join(output)
    
    f = open( os.path.splitext(args.groundtruth)[0] + "-statistics.txt", "w")
    f.write(output+"\n")
    f.close
    
    print output
    