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

if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script reads a file and creates a pgf plot.
    ''')
    parser.add_argument('input', help='input file')
    parser.add_argument('input2',nargs='?', help='input file')
    args = parser.parse_args()
    
        
    plots = []
    files = [args.input]
    if args.input2:
        files.append(args.input2)

    for f in files:
        lines = open(f).read().replace(","," ").replace("\t"," ").split("\n")
        data = [[float(f) for f in l.split(" ") if len(f.strip())] for l in lines if len(l.strip())]
    
        plots.append( list_to_meanstdareaplot2(data,
                                           options_line='draw=black,thick',
                                           options_area='draw=black,fill=blue!20',
                                           legend_line='error',
                                           legend_area='') )
        
    axes = []
    axes.append( plots_to_axis(plots,options='xlabel={time scale $\Delta$ [s]},ylabel={error [m]},yticklabels={0,0.05,0.1,0.15,0.2}') )
    
    axes_to_file(axes,os.path.splitext(args.input)[0] + "-plot",
                 pdflatex=True,pdfcrop=True,pdfview=True,pdfconvert=True)
    