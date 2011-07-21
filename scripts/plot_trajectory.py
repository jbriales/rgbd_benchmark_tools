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

if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script reads a trajectory and creates a 3D pgf plot.
    ''')
    parser.add_argument('input', help='input file')
    args = parser.parse_args()
    
    plots = []
    files = [args.input]

    for f in files:
        traj = read_trajectory(f)
        keys = traj.keys()
        keys.sort()
        traj = [traj[k] for k in keys]
        data = [(t[0,3],t[1,3]) for t in traj]
        print data
        plots.append( list_to_plot(data,
                                           options='draw=black,thick',
                                           legend='trajectory' ) )
        
    axes = []
    axes.append( plots_to_axis(plots,options='xlabel={X [m},ylabel={Y [m}') )
    
    axes_to_file(axes,os.path.splitext(args.input)[0] + "-plot",
                 pdflatex=True,pdfcrop=True,pdfview=True,pdfconvert=True)
    