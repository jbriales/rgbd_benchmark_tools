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
    parser.add_argument('--txt', help='text file to which the data will be saved (format: gnuplot)')
    parser.add_argument('--png', help='image file to which the plot will be saved (format: png)')
    parser.add_argument('--delta', help='delta for evaluation (default: 1.0)',default=1.0)
    parser.add_argument('--delta_unit', help='unit of delta (options: \'s\' for seconds, \'m\' for meters, \'rad\' for radians; default: \'s\')',default='s')
    parser.add_argument('--offset', help='time offset between ground-truth and estimated trajectory (default: 0.0)',default=0.0)
    args = parser.parse_args()
    
    param_delta = float(args.delta)
    param_delta_unit = args.delta_unit
    param_offset = float(args.offset)
    
    traj_gt = evaluate.read_trajectory(args.groundtruth_file)
    traj_est = evaluate.read_trajectory(args.estimated_file)
    
    result = evaluate.evaluate_trajectory(traj_gt,traj_est,0,True,param_delta,param_delta_unit,param_offset,True)
    err_trans  = [(float(key[26:]),value) for key,(value,unit) in result.iteritems() if key.startswith("translational_error.list.")]
    err_trans.sort()
    err_rot  = [(float(key[23:]),value) for key,(value,unit) in result.iteritems() if key.startswith("rotational_error.list.")]
    err_rot.sort()

    if args.txt:
        file = open(args.txt,"w")
        for err in zip(err_trans,err_rot):
            file.write("%f %f %f\n"%(err[0][0],err[0][1],err[1][1]))

    if args.png:    
        fig = plt.figure()
        ax = fig.add_subplot(111)        
        ax.plot([t for t,e in err_trans],[e for t,e in err_trans],'-',color="blue")
        #ax.plot([t for t,e in err_rot],[e for t,e in err_rot],'-',color="red")
        ax.set_xlabel('time [s]')
        ax.set_ylabel('translational error [m]')
        plt.savefig(args.png)
        

