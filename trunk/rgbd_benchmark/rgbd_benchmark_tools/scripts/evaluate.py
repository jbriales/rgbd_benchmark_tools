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

def transform44(ts):
    return numpy.dot(tf.listener.xyz_to_mat44(ts.translation), tf.listener.xyzw_to_mat44(ts.rotation))

def build_transform(l):
    transform = geometry_msgs.msg.Transform()
    transform.translation = geometry_msgs.msg.Vector3(*l[1:4])
    transform.rotation = geometry_msgs.msg.Quaternion(*l[4:8])
    return transform

def read_trajectory(filename):
    lines = open(filename).read().replace(","," ").replace("\t"," ").split("\n") 
    list = [[float(v.strip()) for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    traj = dict([(l[0],transform44(build_transform(l[0:]))) for l in list])
    return traj

def find_closest_stamp(stamps,t):
    return min([(abs(s-t),s) for s in stamps])[1]

def ominus(a,b):
    return numpy.dot(tf.transformations.inverse_matrix(a),b)

def compute_err(param_delta,param_delay,traj_gt,traj_est,stamps_gt,stamps_est):
    err_trans = []
    err_rot = []
    for stamp_est_0 in stamps_est:
        if stamp_est_0+param_delta > stamps_est[-1]: 
            break
        stamp_est_1 = find_closest_stamp(stamps_est,stamp_est_0 + param_delta)
        stamp_gt_0 = find_closest_stamp(stamps_gt,stamp_est_0 - param_delay)
        stamp_gt_1 = find_closest_stamp(stamps_gt,stamp_est_1 - param_delay)
        
        #print stamp_est_0,stamp_est_1,stamp_gt_0,stamp_gt_1
        
        error44 = ominus(  ominus( traj_est[stamp_est_1], traj_est[stamp_est_0] ),
                           ominus( traj_gt[stamp_gt_1], traj_gt[stamp_gt_0] ) )
        xyz = tuple(tf.transformations.translation_from_matrix(error44))[:3]
        quat = tuple(tf.transformations.quaternion_from_matrix(error44))
        
        err_trans.append( numpy.linalg.norm(xyz) )
        # an invitation to 3-d vision, p 27
        err_rot.append( numpy.arccos( (numpy.trace(error44[0:3,0:3]) - 1)/2) )
        
    output =[]
    output.append("translational_error.mean=%0.3fm"%(numpy.mean(err_trans)))
    output.append("translational_error.std=%0.3fm"%(numpy.std(err_trans)))
    output.append("translational_error.median=%0.3fm"%(numpy.median(err_trans)))
    output.append("translational_error.min=%0.3fm"%(numpy.min(err_trans)))
    output.append("translational_error.max=%0.3fm"%(numpy.max(err_trans)))
    
    output.append("rotational_error.mean=%0.3frad"%(numpy.mean(err_rot)))
    output.append("rotational_error.std=%0.3frad"%(numpy.std(err_rot)))
    output.append("rotational_error.median=%0.3frad"%(numpy.median(err_rot)))
    output.append("rotational_error.min=%0.3frad"%(numpy.min(err_rot)))
    output.append("rotational_error.max=%0.3frad"%(numpy.max(err_rot)))
    
    output.append("rotational_error.mean.deg=%0.3fdeg"%(numpy.mean(err_rot)/numpy.pi*180))
    output.append("rotational_error.std.deg=%0.3fdeg"%(numpy.std(err_rot)/numpy.pi*180))
    output.append("rotational_error.median.deg=%0.3fdeg"%(numpy.median(err_rot)/numpy.pi*180))
    output.append("rotational_error.min.deg=%0.3fdeg"%(numpy.min(err_rot)/numpy.pi*180))
    output.append("rotational_error.max.deg=%0.3fdeg"%(numpy.max(err_rot)/numpy.pi*180))
    
    return output


if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script reads a groundtruth trajectory and an estimated trajectory, and computes the translational error.
    ''')
    parser.add_argument('groundtruth', help='groundtruth trajectory file (format: timestamp x y z qx qy qz qw)')
    parser.add_argument('estimated', help='estimated trajectory file (format: timestamp x y z qx qy qz qw)')
    parser.add_argument('--delta', help='time delta for evaluation (see paper)',default=1.0)
    parser.add_argument('--delay', help='time offset between files (0.0 if synced, otherwise )',default=0.0)
#    parser.add_argument('--evaldelta', help='evaluate over different time deltas')
#    parser.add_argument('--evaldelay', help='evaluate over different time delays')
    args = parser.parse_args()
    
    traj_gt = read_trajectory(args.groundtruth)
    traj_est = read_trajectory(args.estimated)
    param_delta = float(args.delta)
    param_delay = float(args.delay)
    
    stamps_gt = list(traj_gt.keys())
    stamps_gt.sort()
    stamps_est = list(traj_est.keys())
    stamps_est.sort()

#    if args.evaldelta:
#        for param_delta in numpy.arange(0,10,0.0333):
#            err_trans = compute_err(param_delta,param_delay,traj_gt,traj_est,stamps_gt,stamps_est)    
#            print param_delta, " ".join(str(f) for f in err_trans)
#        sys.exit()
#
#    if args.evaldelay:
#        for param_delay in numpy.arange(-0.1,0.1,0.01):
#            err_trans = compute_err(param_delta,param_delay,traj_gt,traj_est,stamps_gt,stamps_est)    
#            print param_delay, " ".join(str(f) for f in err_trans)
#        sys.exit()

    output =[]
    #output.append("trajectory.groundtruth=%s"%args.groundtruth)
    #output.append("trajectory.estimated=%s"%args.estimated)
    output.append("parameter.time_delta=%0.3fs"%param_delta)
    #output.append("parameter.time_delay=%0.3fs"%param_delay)
    output = output + compute_err(param_delta,param_delay,traj_gt,traj_est,stamps_gt,stamps_est)    
    print "\n".join(str(f) for f in output)
