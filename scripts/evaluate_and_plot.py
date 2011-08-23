#!/usr/bin/python

import argparse

import evaluate

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab

if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script reads a ground-truth trajectory and an estimated trajectory, and computes the translational error.
    ''')
    parser.add_argument('groundtruth_file', help='ground-truth trajectory file (format: "timestamp tx ty tz qx qy qz qw")')
    parser.add_argument('estimated_file', help='estimated trajectory file (format: "timestamp tx ty tz qx qy qz qw")')
    parser.add_argument('plot_file', help='file to which the plot will be saved (format: png)')
    parser.add_argument('--delta', help='time delta for evaluation (default: 1.0)',default=1.0)
    parser.add_argument('--offset', help='time offset between ground-truth and estimated trajectory (default: 0.0)',default=0.0)
    args = parser.parse_args()
    
    param_delta = float(args.delta)
    param_offset = float(args.offset)
    
    traj_gt = evaluate.read_trajectory(args.groundtruth_file)
    traj_est = evaluate.read_trajectory(args.estimated_file)
    
    result = evaluate.evaluate_trajectory(traj_gt,traj_est,param_delta,param_offset,True)
    err_trans  = [(float(key[26:]),value) for key,(value,unit) in result.iteritems() if key.startswith("translational_error.list.")]
    err_trans.sort()
    
    fig = plt.figure()
    ax = fig.add_subplot(111)        
    
    ax.plot([t for t,e in err_trans],[e for t,e in err_trans],'-',color="blue")
    #leg = ax.legend(('Model length'),'upper center', shadow=True)
    #       plt.axis((x[0],x[-1],numpy.min(y),numpy.max(y)))
    ax.set_xlabel('time delta [s]')
    ax.set_ylabel('translational error [m]')
    #plt.title(r'window size $\delta$ [s]', fontsize=20)
    plt.savefig(args.plot_file)


