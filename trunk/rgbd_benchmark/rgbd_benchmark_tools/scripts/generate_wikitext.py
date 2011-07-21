#!/usr/bin/python

import sys


for filename in sys.argv[1:]:
    stat=dict([line.strip().split("=") for line in open(filename)])
    print ""
