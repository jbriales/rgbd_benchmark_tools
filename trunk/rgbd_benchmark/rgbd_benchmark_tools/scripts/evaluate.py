#!/usr/bin/python
#
# Requirements: 
# sudo apt-get install python-argparse

import argparse
import sys
import os
import numpy

_EPS = numpy.finfo(float).eps * 4.0

def transform44(l):
    t = l[1:4]
    q = numpy.array(l[4:8], dtype=numpy.float64, copy=True)
    nq = numpy.dot(q, q)
    if nq < _EPS:
        return numpy.array((
        (                1.0,                 0.0,                 0.0, t[0])
        (                0.0,                 1.0,                 0.0, t[1])
        (                0.0,                 0.0,                 1.0, t[2])
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=numpy.float64)
    q *= numpy.sqrt(2.0 / nq)
    q = numpy.outer(q, q)
    return numpy.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], t[0]),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], t[1]),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], t[2]),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=numpy.float64)

def read_trajectory(filename):
    lines = open(filename).read().replace(","," ").replace("\t"," ").split("\n") 
    list = [[float(v.strip()) for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    traj = dict([(l[0],transform44(l[0:])) for l in list])
    return traj

def find_closest_stamp(stamps,t):
    beginning = 0
    difference = abs(stamps[0] - t)
    best = stamps[0]
    end = len(stamps)
    while beginning < end:
        middle = int((end+beginning)/2)
        if abs(stamps[middle] - t) < difference:
            difference = abs(stamps[middle] - t)
            best = stamps[middle]
        if t == stamps[middle]:
            return stamps[middle]
        elif stamps[middle] > t:
            end = middle
        else:
            beginning = middle + 1
    return best

def ominus(a,b):
    return numpy.dot(numpy.linalg.inv(a),b)

def evaluate_trajectory(traj_gt,traj_est,param_delta=1.00,param_delay=0.01,downsample=0):
    stamps_gt = list(traj_gt.keys())
    stamps_gt.sort()
    stamps_est = list(traj_est.keys())
    stamps_est.sort()
    
    err_trans = []
    err_rot = []
    
    if downsample>0 and len(stamps_est)>downsample:
        stamps_est_subset = random.sample(stamps_est,downsample)
    else:
        stamps_est_subset = stamps_est 
        
    for stamp_est_0 in stamps_est_subset:
        if stamp_est_0+param_delta > stamps_est[-1]: 
            continue
        stamp_est_1 = find_closest_stamp(stamps_est,stamp_est_0 + param_delta)
        stamp_gt_0 = find_closest_stamp(stamps_gt,stamp_est_0 - param_delay)
        stamp_gt_1 = find_closest_stamp(stamps_gt,stamp_est_1 - param_delay)
        
        #print stamp_est_0,stamp_est_1,stamp_gt_0,stamp_gt_1
        
        error44 = ominus(  ominus( traj_est[stamp_est_1], traj_est[stamp_est_0] ),
                           ominus( traj_gt[stamp_gt_1], traj_gt[stamp_gt_0] ) )
        
        err_trans.append( numpy.linalg.norm(error44[0:3,3]) )
        # an invitation to 3-d vision, p 27
        err_rot.append( numpy.arccos( min(1,max(-1, (numpy.trace(error44[0:3,0:3]) - 1)/2) )) )
        
    result ={}
    result["parameter.time_delta"] = (float(param_delta),"s")
    result["parameter.extra_delay"] = (float(param_delay),"s")
    result["number_of_samples"] = (len(err_trans),"samples")
    
    result["translational_error.mean"] = (numpy.mean(err_trans),"m")
    result["translational_error.std"] = (numpy.std(err_trans),"m")
    result["translational_error.median"] = (numpy.median(err_trans),"m")
    result["translational_error.min"] = (numpy.min(err_trans),"m")
    result["translational_error.max"] = (numpy.max(err_trans),"m")
    
    result["rotational_error.mean"] = (numpy.mean(err_rot),"rad")
    result["rotational_error.std"] = (numpy.std(err_rot),"rad")
    result["rotational_error.median"] = (numpy.median(err_rot),"rad")
    result["rotational_error.min"] = (numpy.min(err_rot),"rad")
    result["rotational_error.max"] = (numpy.max(err_rot),"rad")
    
    return result



if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script reads a groundtruth trajectory and an estimated trajectory, and computes the translational error.
    ''')
    parser.add_argument('groundtruth', help='groundtruth trajectory file (format: timestamp x y z qx qy qz qw)')
    parser.add_argument('estimated', help='estimated trajectory file (format: timestamp x y z qx qy qz qw)')
    parser.add_argument('--time_delta', help='time delta for evaluation (see paper)',default=1.0)
    parser.add_argument('--extra_delay', help='time offset between files (0.0 if synced, otherwise)',default=0.0)
    parser.add_argument('--full', help='print all evaluation data (otherwise, only the mean translational error measured in meters will be printed)', action='store_true')
    args = parser.parse_args()
    
    param_delta = float(args.time_delta)
    param_delay = float(args.extra_delay)
    
    traj_gt = read_trajectory(args.groundtruth)
    traj_est = read_trajectory(args.estimated)
    
    result = evaluate_trajectory(traj_gt,traj_est,param_delta,param_delay)
    if args.full:
        keys = list(result.keys())
        keys.sort()
        print("".join("%s = %0.5f %s\n"%(key,result[key][0],result[key][1]) for key in keys))
    else:
        print result["translational_error.mean"][0]

