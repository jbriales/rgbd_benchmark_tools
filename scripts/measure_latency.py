#!/usr/bin/python

import numpy
import time
import os
import random

delay = []

def do_something():
    f = open("latency-test-file.%0.4f"%random.random(),"w")
    f.write("something")
    f.close()

while True:
    t0 = time.time()
    do_something()
    t1 = time.time()
    delay.append( t1 - t0 )
#    print "n=%d, max=%0.2f, q(0.50)=%0.2f, q(0.70)=%0.2f, q(0.90)=%0.2f, q(0.95)=%0.2f, q(0.99)=%0.2f, last=%0.2f"%(
#                len(delay),numpy.max(delay),
#                numpy.percentile(delay, 50),
#                numpy.percentile(delay, 70),
#                numpy.percentile(delay, 90),
#                numpy.percentile(delay, 95),
#                numpy.percentile(delay, 99),delay[-1])
    sorted = list(delay)
    sorted.sort()
    
    print "n=%d, max=%0.2f, q50=%0.2f, q95=%0.2f, q99=%0.2f, q999=%0.2f, last=%0.2f"%(
                len(delay),
                numpy.max(delay),
                sorted[int(len(sorted)*0.50)],
                sorted[int(len(sorted)*0.95)],
                sorted[int(len(sorted)*0.99)],
                sorted[int(len(sorted)*0.999)],
                delay[-1])
    time.sleep(1)
    