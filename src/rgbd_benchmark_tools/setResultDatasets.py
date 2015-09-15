#!/usr/bin/python

"""
This script creates the datasets for results in the experiment.
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
    parser.add_argument('num', help='Num of cases', type=int)
    args = parser.parse_args()

    f = h5py.File(args.file, 'a') # Read/write if exists, create otherwise
    g = f.require_group(args.group) # Open group, create if doesn't exist

    dsetNames = ['rmse','failRate','tOffset','tUtil','tSel','tLoop','tTotal']
    dims = [1,1,1,5,5,5,1]
    for name, dim in zip(dsetNames,dims):
        if not (name in g):
            g.create_dataset(name, (dim,args.num))
        
    # Close the file before exiting
    f.close()