#!/usr/bin/python

"""
This script creates the H5 file and internal groups.
"""

import argparse
import sys
import h5py

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='''
    This script sets an H5 file and internal groups.
    ''')
    parser.add_argument('file', help='HDF5 file path (to create if non existing)')
    parser.add_argument('group', help='Internal group (to create together with intermediate groups if non existing)')
    args = parser.parse_args()
    
    f = h5py.File(args.file, 'a') # Read/write if exists, create otherwise
    
    if args.group in f:
        sys.exit("The group already exists in the H5 file. Please use other group")
    else:
        g = f.create_group(args.group)
    
    # Close the file before exiting
    f.close()
    
        

