#!/usr/bin/python
#
# Requirements: 
# sudo apt-get install python-argparse

import sys
import numpy
import argparse
import associate

def align(model,data):
    numpy.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    W = numpy.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
    S = numpy.matrix(numpy.identity( 3 ))
    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh
    trans = data.mean(1) - rot * model.mean(1)
    
    model_aligned = rot * model + trans
    alignment_error = model_aligned - data
    
    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,trans,trans_error

def plot_traj(ax,stamps,traj,style,color,label):
    stamps.sort()
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x)>0:
            ax.plot(x,y,style,color=color,label=label)
            label=""
            x=[]
            y=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,style,color=color,label=label)
            
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

def interpolate_transformation(alpha,a,b):
    ta = numpy.matrix([float(v) for v in a[0:3]])
    tb = numpy.matrix([float(v) for v in b[0:3]])
    tr = (1-alpha) * ta + (alpha) * tb
    return tr.tolist()[0]
        
def interpolate_and_resample(traj_gt,stamps_est,param_offset,gt_max_time_difference):
    traj_new = {}
    
    stamps_gt = list(traj_gt.keys())
    stamps_gt.sort()
    
    for t_est in stamps_est:
        t_new = t_est + param_offset
        i_nearest = find_closest_index(stamps_gt,t_est + param_offset)
        if stamps_gt[i_nearest] >= t_new and i_nearest>0:
            i_low = i_nearest-1
            i_high = i_nearest
        else:
            i_low = i_nearest
            i_high = i_nearest+1
        if i_low<0 or i_high>=len(stamps_gt):
            continue

        t_low = stamps_gt[i_low]   
        t_high = stamps_gt[i_high]
           
        if(abs( t_new - (t_low + param_offset) ) > gt_max_time_difference or
           abs( t_new - (t_high + param_offset) ) > gt_max_time_difference):
            continue

        alpha = (t_new - (t_low + param_offset)) / (t_high - t_low)
        transform = interpolate_transformation(alpha,traj_gt[t_low],traj_gt[t_high])
        traj_new[t_new] = transform
    return traj_new

if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
    parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('second_file', help='estimated trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('--interpolate', help='interpolate and resample the ground truth trajectory', action='store_true')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)',default=1.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    parser.add_argument('--save_associations', help='save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)')
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)')
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    args = parser.parse_args()

    first_list = associate.read_file_list(args.first_file)
    second_list = associate.read_file_list(args.second_file)
    
    if args.interpolate:
       first_list = interpolate_and_resample(first_list,list(second_list.keys()),float(args.offset),float(args.max_difference)*2)

    matches = associate.associate(first_list, second_list,float(args.offset),float(args.max_difference))    
    if len(matches)<2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")

    first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
    second_xyz = numpy.matrix([[float(value)*float(args.scale) for value in second_list[b][0:3]] for a,b in matches]).transpose()
    rot,trans,trans_error = align(second_xyz,first_xyz)
    
    second_xyz_aligned = rot * second_xyz + trans
    
    first_stamps = first_list.keys()
    first_stamps.sort()
    first_xyz_full = numpy.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()
    
    second_stamps = second_list.keys()
    second_stamps.sort()
    second_xyz_full = numpy.matrix([[float(value)*float(args.scale) for value in second_list[b][0:3]] for b in second_stamps]).transpose()
    second_xyz_full_aligned = rot * second_xyz_full + trans
    
    if args.verbose:
        print "compared_pose_pairs %d pairs"%(len(trans_error))

        print "absolute_translational_error.rmse %f m"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        print "absolute_translational_error.mean %f m"%numpy.mean(trans_error)
        print "absolute_translational_error.median %f m"%numpy.median(trans_error)
        print "absolute_translational_error.std %f m"%numpy.std(trans_error)
        print "absolute_translational_error.min %f m"%numpy.min(trans_error)
        print "absolute_translational_error.max %f m"%numpy.max(trans_error)
    else:
        print "%f"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        
    if args.save_associations:
        file = open(args.save_associations,"w")
        file.write("\n".join(["%f %f %f %f %f %f %f %f"%(a,x1,y1,z1,b,x2,y2,z2) for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A)]))
        file.close()
        
    if args.save:
        file = open(args.save,"w")
        file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(second_stamps,second_xyz_full_aligned.transpose().A)]))
        file.close()

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.pylab as pylab
        from matplotlib.patches import Ellipse
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_traj(ax,first_stamps,first_xyz_full.transpose().A,'-',"black","ground truth")
        plot_traj(ax,second_stamps,second_xyz_full_aligned.transpose().A,'-',"blue","estimated")

        label="difference"
        for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A):
            ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
            label=""
            
        ax.legend()
            
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.savefig(args.plot,figsize=(8, 6), dpi=80)
        
