#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 17 09:02:31 2015

@author: jesus
"""

import argparse
import numpy as np

import h5py

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='''
    This script collects the metrics and results from several samples of an experiment into its parent group.
    ''')
    parser.add_argument('h5file', help='HDF5 file in which the metrics are stored in the group eval for each sample')
    parser.add_argument('group', help='H5 path of the main group containing sample minor groups')
    parser.add_argument('delta_unit', help='delta_unit of the metrics to collect')
    args = parser.parse_args()
        
    h5f = h5py.File(args.h5file,'a')
    unit = args.delta_unit
    
    # Save the evaluation metric values in the samples' parent group
    main_group = h5f[args.group]
    # Check if eval group already exists in the main group
    if 'eval' in main_group:
        print "Removing existing eval group in" + main_group.name
        del main_group['eval']
    numOfSamples = len(main_group)
    
    # Create new eval group in the main group
    samples = main_group.keys()
    eval_group = main_group.require_group('eval/'+args.delta_unit)
    names = ['rmse','median','mean','max']
    for name in names:
        # Preallocate arrays
        t_arr = np.empty(numOfSamples)
        r_arr = np.empty(numOfSamples)
        # Store metrics in sample in an array
        for i, sample in enumerate(samples):
            t_arr[i] = main_group[sample+'/eval/'+unit+'/t_'+name][()]
            r_arr[i] = main_group[sample+'/eval/'+unit+'/r_'+name][()]
        # Check if dataset already exists in the group
        if 't_'+name in eval_group:
            print "Removing existing trans dataset in " + eval_group.name
            del eval_group['t_'+name]
        if 'r_'+name in eval_group:
            print "Removing existing rot dataset in " + eval_group.name
            del eval_group['r_'+name]    
        # Save as a new dataset in the main group
        eval_group.create_dataset('t_'+name, data=t_arr)
        eval_group.create_dataset('r_'+name, data=r_arr)
