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
    Plot mean precision (averaged from samples) for different cases wrt sampling ratio.
    ''')
    parser.add_argument('h5file', help='HDF5 file in which the metrics are stored in the group eval for each sample')
    parser.add_argument('group', help='H5 path of the main group containing... ratioGroups')
    parser.add_argument('delta_unit', help='delta_unit of the metrics to collect')
    parser.add_argument('metric', help='metric to plot')
    args = parser.parse_args()
        
    h5f = h5py.File(args.h5file,'a')
    unit = args.delta_unit
    metric = args.metric
    
    # Open the base group for all the cases to show
    main_group = h5f[args.group]

    # Pre-create/allocate
    numOfRatios = len(main_group)
    #    cases = []
    cases = ['Dell_','GMS_','Max','Meil','Ramp_']
    lines = {}
    for case in cases:
        lines[case] = np.empty(numOfRatios)
        
    # Iterate through the different sampling ratios
    ratios = main_group.keys()
    for x,ratio in enumerate(ratios):
        # Iterate, for the current ratio, through all cases
        ratioGroup = main_group[ratio]
        
        # Preallocate if not done yet
#        if cases == []:
#            cases = ratioGroup.keys()
#            for case in cases:
#                lines[case] = np.empty(numOfRatios)

        # Store values in the lines dictionary                
        for case in cases:
            caseGroup = ratioGroup[case]
            dset = caseGroup['eval/'+unit+'/'+metric]
            lines[case][x] = np.mean( dset[:] )
            
    # Plot the graph
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.pylab as pylab
    fig = plt.figure()
    ax = fig.add_subplot(111)  
    for key in lines:
        ax.plot(lines[key],'-',color="blue")

    #ax.plot([t for t,e in err_rot],[e for t,e in err_rot],'-',color="red")
    ax.set_xlabel('time [s]')
    ax.set_ylabel('translational error [m]')
    plt.savefig('figure',dpi=300)            
