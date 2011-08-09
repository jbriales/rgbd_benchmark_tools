#!/usr/bin/python

import argparse
import sys
import os
import numpy
import pickle
import random
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab

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

def find_closest_stamp2(stamps,t):
    return min([(abs(s-t),s) for s in stamps])[1]

def find_closest_stamp(stamps,t):
    beginning = 0
    difference = abs(stamps[0] - t)
    best = 0
    end = len(stamps)
    while beginning < end:
        middle = (end+beginning)/2
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
    parser.add_argument('--extra_delay', help='time offset between files (0.0 if synced, otherwise )',default=0.0)
    parser.add_argument('--eval_range', help='compute statistics over the whole range of possible time deltas', action='store_true')
    parser.add_argument('--plot_file', help='plot results and save as PNG to PLOT')
    args = parser.parse_args()
    
    param_delta = float(args.time_delta)
    param_delay = float(args.extra_delay)
    
    traj_gt = read_trajectory(args.groundtruth)
    traj_est = read_trajectory(args.estimated)
    
    if not args.eval_range and not args.plot_file:
        result = evaluate_trajectory(traj_gt,traj_est,param_delta,param_delay)
        keys = result.keys()
        keys.sort()
        print "".join("%s = %0.5f %s\n"%(key,result[key][0],result[key][1]) for key in keys)
        sys.exit()
        
    if args.eval_range:
        duration = numpy.max(traj_gt.keys()) - numpy.min(traj_gt.keys())
        delta_range = []
        delta = 0
        while delta < duration/2.0:
#        while delta < 2.0:
            delta_range.append(delta)
            delta += 0.1
        table = []
        print "# time delta [s], avg. transl. error [m]"
        for delta in delta_range:
            result = evaluate_trajectory(traj_gt,traj_est,delta,param_delay)
            table.append( (delta, result) )
            print "%0.3f, %0.5f"%(delta,result["translational_error.mean"][0])
            
        #pickle.dump(table,open('result.dat','wb'))

    if args.plot_file:
        #table = pickle.load(open('result.dat','rb'))
        x =[delta for (delta,result) in table]
        y_err_mean =[result["translational_error.mean"][0] for (delta,result) in table]
        y_err_mean_pstd =[result["translational_error.mean"][0]
                          +result["translational_error.std"][0] for (delta,result) in table]
        y_err_mean_nstd =[result["translational_error.mean"][0]
                          -result["translational_error.std"][0] for (delta,result) in table]
        y_err_min =[result["translational_error.min"][0] for (delta,result) in table]
        y_err_max =[result["translational_error.max"][0] for (delta,result) in table]
        
        #plt.semilogx()
        fig = plt.figure()
        ax = fig.add_subplot(111)        
        
        ax.fill_between(x,y_err_min,y_err_max,color='#cfcfff',facecolor='#cfcfff',label="min/max")
        ax.fill_between(x,y_err_mean_nstd,y_err_mean_pstd,color='#afafff',facecolor='#afafff',label="std")
        ax.plot(x,y_err_mean,'-',color="black")
        #leg = ax.legend(('Model length'),'upper center', shadow=True)
 #       plt.axis((x[0],x[-1],numpy.min(y),numpy.max(y)))
        ax.set_xlabel('time delta [s]')
        ax.set_ylabel('translational error [m]')
        #plt.title(r'window size $\delta$ [s]', fontsize=20)
        plt.savefig(args.plot_file)

        