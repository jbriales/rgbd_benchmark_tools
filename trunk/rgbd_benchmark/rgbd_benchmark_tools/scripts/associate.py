#!/usr/bin/python
#
# Requirements: 
# sudo apt-get install python-argparse

import argparse
import sys
import os
import numpy

def read_file_list(filename):
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
    return dict(list)


if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script takes two data files with timestamps and associates them   
    ''')
    parser.add_argument('first_file', help='first text file (format: timestamp data)')
    parser.add_argument('second_file', help='second text file (format: timestamp data)')
    parser.add_argument('--extra_delay', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    args = parser.parse_args()

    args.max_difference = float(args.max_difference)
    args.extra_delay = float(args.extra_delay)
    
    first_list = read_file_list(args.first_file)
    second_list = read_file_list(args.second_file)
    
    first_keys = first_list.keys()
    second_keys = second_list.keys()
    
    potential_matches = [(abs(a-b+args.extra_delay),a,b-args.extra_delay) for a in first_keys for b in second_keys if abs(a-b+args.extra_delay) < args.max_difference]
    potential_matches.sort()

    matches = []
    for diff,a,b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a,b))

    matches.sort()    
    
    for a,b in matches:
        print("%f %s %f %s"%(a," ".join(first_list[a]),b+args.extra_delay," ".join(second_list[b])))
        
        