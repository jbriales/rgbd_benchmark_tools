#!/usr/bin/python

import argparse
import random
import numpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import evaluate

if __name__ == '__main__':
    random.seed(0)

    parser = argparse.ArgumentParser(description='''
    This script reads a ground-truth trajectory and an estimated trajectory, and computes the (translational) error.
    ''')
    parser.add_argument('groundtruth_file', help='ground-truth trajectory file (format: "timestamp tx ty tz qx qy qz qw")')
    parser.add_argument('estimated_file', help='estimated trajectory file (format: "timestamp tx ty tz qx qy qz qw")')
    parser.add_argument('--max_pairs', help='maximum number of pose comparisons (default: 10000, set to zero to disable downsampling)', default=10000)
    parser.add_argument('--delta', help='delta for evaluation (default: 1.0)',default=1.0)
    parser.add_argument('--delta_unit', help='unit of delta (options: \'s\' for seconds, \'m\' for meters, \'rad\' for radians; default: \'s\')',default='s')
    parser.add_argument('--offset', help='time offset between ground-truth and estimated trajectory (default: 0.0)',default=0.0)
    parser.add_argument('--save', help='text file to which the evaluation will be saved (format: stamp_est0 stamp_est1 stamp_gt0 stamp_gt1 trans_error rot_error)')
    parser.add_argument('--plot', help='plot the result to a file (format: png)')
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the mean translational error measured in meters will be printed)', action='store_true')
    args = parser.parse_args()
    
    traj_gt = evaluate.read_trajectory(args.groundtruth_file)
    traj_est = evaluate.read_trajectory(args.estimated_file)
    
    result = evaluate.evaluate_trajectory(traj_gt,
                                 traj_est,
                                 args.max_pairs,
                                 True,
                                 float(args.delta),
                                 args.delta_unit,
                                 float(args.offset))
    
    stamps = numpy.array(result)[:,0]
    trans_error = numpy.array(result)[:,4]
    rot_error = numpy.array(result)[:,5]
    
    if args.save:
        f = open(args.save,"w")
        f.write("\n".join([" ".join(["%f"%v for v in line]) for line in result]))
        f.close()
    
    if args.verbose:
        print "compared_pose_pairs %d"%(len(trans_error))

        print "translational_error.rmse %f"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        print "translational_error.mean %f"%numpy.mean(trans_error)
        print "translational_error.std %f"%numpy.std(trans_error)
        print "translational_error.min %f"%numpy.min(trans_error)
        print "translational_error.max %f"%numpy.max(trans_error)

        print "rotational_error.rmse %f"%numpy.sqrt(numpy.dot(rot_error,rot_error) / len(rot_error))
        print "rotational_error.mean %f"%numpy.mean(rot_error)
        print "rotational_error.std %f"%numpy.std(rot_error)
        print "rotational_error.min %f"%numpy.min(rot_error)
        print "rotational_error.max %f"%numpy.max(rot_error)
    else:
        print numpy.mean(trans_error)

    if args.plot:    
        fig = plt.figure()
        ax = fig.add_subplot(111)        
        ax.plot(stamps,trans_error,'-',color="blue")
        #ax.plot([t for t,e in err_rot],[e for t,e in err_rot],'-',color="red")
        ax.set_xlabel('time [s]')
        ax.set_ylabel('translational error [m]')
        plt.savefig(args.plot,dpi=70)
        

