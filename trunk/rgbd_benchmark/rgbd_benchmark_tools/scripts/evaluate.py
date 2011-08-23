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
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[float(v.strip()) for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    traj = dict([(l[0],transform44(l[0:])) for l in list if l[4:8]!=[0,0,0,0]])
    return traj

def find_closest_index(L,t):
    beginning = 0
    difference = abs(L[0] - t)
    best = 0
    end = len(L)
    while beginning < end:
        middle = int((end+beginning)/2)
        if abs(L[middle] - t) < difference:
            difference = abs(L[middle] - t)
            best = middle
        if t == L[middle]:
            return middle
        elif L[middle] > t:
            end = middle
        else:
            beginning = middle + 1
    return best

def ominus(a,b):
    return numpy.dot(numpy.linalg.inv(a),b)

def compute_distance(transform):
    return numpy.linalg.norm(transform[0:3,3])

def compute_angle(transform):
    # an invitation to 3-d vision, p 27
    return numpy.arccos( min(1,max(-1, (numpy.trace(transform[0:3,0:3]) - 1)/2) ))

def distances_along_trajectory(traj):
    keys = traj.keys()
    keys.sort()
    motion = [ominus(traj[keys[i+1]],traj[keys[i]]) for i in range(len(keys)-1)]
    distances = [0]
    sum = 0
    for t in motion:
        sum += compute_distance(t)
        distances.append(sum)
    return distances
    
def rotations_along_trajectory(traj):
    keys = traj.keys()
    keys.sort()
    motion = [ominus(traj[keys[i+1]],traj[keys[i]]) for i in range(len(keys)-1)]
    distances = [0]
    sum = 0
    for t in motion:
        sum += compute_angle(t)
        distances.append(sum)
    return distances
    

def evaluate_trajectory(traj_gt,traj_est,param_delta=1.00,param_delta_unit="s",param_offset=0.01,store_individual_errors=False):
    stamps_gt = list(traj_gt.keys())
    stamps_est = list(traj_est.keys())
    stamps_gt.sort()
    stamps_est.sort()

    if param_delta_unit=="s":
        index_gt = list(traj_gt.keys())
        index_est = list(traj_est.keys())
        index_gt.sort()
        index_est.sort()
    elif param_delta_unit=="m":
        index_gt = distances_along_trajectory(traj_gt)
        index_est = distances_along_trajectory(traj_est)
    elif param_delta_unit=="rad":
        index_gt = rotations_along_trajectory(traj_gt)
        index_est = rotations_along_trajectory(traj_est)
    else:
        raise Exception("Unknown unit for delta: '%s'"%param_delta_unit)
    
    result ={}
    result["parameter.delta"] = (float(param_delta),param_delta_unit)
    result["parameter.offset"] = (float(param_offset),"s")

    interval_gt = [abs(a-b) for a,b in zip(stamps_gt[:-1],stamps_gt[1:])]
    interval_est = [abs(a-b) for a,b in zip(stamps_est[:-1],stamps_est[1:])]
    
    result["input.groundtruth.samples"] = (float(len(stamps_gt)),"samples")
    result["input.estimated.samples"] = (float(len(stamps_est)),"samples")
    result["input.groundtruth.frequency"] = (1/numpy.median(interval_gt),"Hz")
    result["input.groundtruth.duration"] = (len(stamps_gt) * numpy.median(interval_gt),"s")
    result["input.estimated.frequency"] = (1/numpy.median(interval_est),"Hz")
    result["input.estimated.duration"] = (len(stamps_est) * numpy.median(interval_est),"s")
    result["input.coverage"] = (result["input.estimated.duration"][0] / result["input.groundtruth.duration"][0],"")
    
    err_trans = []
    err_rot = []
    
    matches_difference = []
    for i in range(len(traj_est)):
        j = find_closest_index(index_est,index_est[i] + param_delta)
        if j==len(traj_est)-1: 
            continue
        
        stamp_est_0 = stamps_est[i]
        stamp_est_1 = stamps_est[j]

        stamp_gt_0 = stamps_gt[ find_closest_index(stamps_gt,stamp_est_0 - param_offset) ]
        stamp_gt_1 = stamps_gt[ find_closest_index(stamps_gt,stamp_est_1 - param_offset) ]
        
        matches_difference.append( abs(stamp_est_0 - stamp_gt_0) )
        matches_difference.append( abs(stamp_est_1 - stamp_gt_1) )
        
        error44 = ominus(  ominus( traj_est[stamp_est_1], traj_est[stamp_est_0] ),
                           ominus( traj_gt[stamp_gt_1], traj_gt[stamp_gt_0] ) )
        
        trans = compute_distance(error44)
        err_trans.append( trans )

        rot = compute_angle(error44)
        err_rot.append( rot )
        
        if store_individual_errors:
            result["translational_error.list.%f"%stamp_est_0] = (trans,"m") 
            result["rotational_error.list.%f"%stamp_est_0] = (trans,"rad") 
        
    if(len(matches_difference)/2<2):
        raise Exception("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory!")
        
    result["evaluation.number_of_samples"] = (len(err_trans),"samples")
    result["evaluation.number_of_matches"] = (len(matches_difference)/2,"samples")
    
    result["evaluation.time_accuracy_of_matches.mean"] = (numpy.mean(matches_difference),"s")
    result["evaluation.time_accuracy_of_matches.std"] = (numpy.std(matches_difference),"s")
    result["evaluation.time_accuracy_of_matches.median"] = (numpy.median(matches_difference),"s")
    result["evaluation.time_accuracy_of_matches.min"] = (numpy.min(matches_difference),"s")
    result["evaluation.time_accuracy_of_matches.max"] = (numpy.max(matches_difference),"s")
    
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
    This script reads a ground-truth trajectory and an estimated trajectory, and computes the translational error.
    ''')
    parser.add_argument('groundtruth_file', help='ground-truth trajectory file (format: "timestamp tx ty tz qx qy qz qw")')
    parser.add_argument('estimated_file', help='estimated trajectory file (format: "timestamp tx ty tz qx qy qz qw")')
    parser.add_argument('--delta', help='delta for evaluation (default: 1.0)',default=1.0)
    parser.add_argument('--delta_unit', help='unit of delta (options: \'s\' for seconds, \'m\' for meters, \'rad\' for radians; default: \'s\')',default='s')
    parser.add_argument('--offset', help='time offset between ground-truth and estimated trajectory (default: 0.0)',default=0.0)
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the mean translational error measured in meters will be printed)', action='store_true')
    parser.add_argument('--store_individual_errors', help='additional store the individual errors (use with --verbose)', action='store_true')
    args = parser.parse_args()
    
    param_delta = float(args.delta)
    param_delta_unit = args.delta_unit
    param_offset = float(args.offset)
    
    traj_gt = read_trajectory(args.groundtruth_file)
    traj_est = read_trajectory(args.estimated_file)
    
    result = evaluate_trajectory(traj_gt,traj_est,param_delta,param_delta_unit,param_offset,args.store_individual_errors)
    if args.verbose:
        keys = list(result.keys())
        keys.sort()
        print("".join("%s = %0.5f %s\n"%(key,result[key][0],result[key][1]) for key in keys))
    else:
        print result["translational_error.mean"][0]

