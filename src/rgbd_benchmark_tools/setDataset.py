#!/usr/bin/python

"""
This script creates the H5 file and internal groups.
"""

import argparse
import sys
import numpy
import h5py

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='''
    This script sets an H5 file and internal groups.
    ''')
    parser.add_argument('file', help='HDF5 file path (to create if non existing)')
    parser.add_argument('group', help='Internal group (to create together with intermediate groups if non existing)')
    parser.add_argument('gt_file', help='Groundtruth file with trajectory in TUM format')
    args = parser.parse_args()
    
    arr = numpy.loadtxt(args.gt_file)
    # Transpose the matrix to meet our usual format in HDF5
    arr = numpy.transpose(arr)
    
    f = h5py.File(args.file, 'a') # Read/write if exists, create otherwise
    g = f.require_group(args.group) # Open group, create if doesn't exist
    
    if 'groundtruth' in g:
        print 'Groundtruth is already stored in the current group'
    else:
        dset = g.create_dataset('groundtruth', data=arr)
    
    # Close the file before exiting
    f.close()