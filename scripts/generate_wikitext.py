#!/usr/bin/python

import sys
import os

file_prefix = "/usr/wiss/sturmju/public_html/"
url_prefix = "http://www9.in.tum.de/~sturmju/" 

def url(filename):
    if not filename.startswith(file_prefix):
        raise Exception("wrong file prefix: %s, should have %s"%(filename,file_prefix))
    return url_prefix+filename[len(file_prefix):]

# arguments are bag file names
for bagfile in sys.argv[1:]:
    filename = os.path.splitext(bagfile)[0]
    seqname = os.path.split(filename)[1][15:]
    statfile = os.path.splitext(bagfile)[0] + "-statistics.txt"
    stat=dict([line.strip().split("=") for line in open(statfile)])
    infofile = os.path.splitext(bagfile)[0] + "-info.txt"
    if os.path.exists(infofile):    
        info="".join(open(infofile).readlines())
    else:
        info="(no additional info available)"
    
    originalbag = "%s.bag"%(filename)
    originalgt = "%s-groundtruth.txt"%(filename)
    rgbavi = "%s-rgb.png"%(filename)
    depthavi = "%s-depth.png"%(filename)
    
    output=[]
    output.append("==== Sequence '%s' ====="%seqname)
    output.append("{{%s?120 }}"%url(rgbavi))
    output.append(info)
    
    for hz in [2,5,10,30]:
        bag = "%s-%dhz.bag"%(filename,hz)
        rgbavi = "%s-%dhz-rgb.avi"%(filename,hz)
        depthavi = "%s-%dhz-depth.avi"%(filename,hz)
        tgz = "%s-%dhz.tgz"%(filename,hz)
        gt = "%s-%dhz-groundtruth.txt"%(filename,hz)
        imu = "%s-%dhz-imu.txt"%(filename,hz)
            
            
    
print "\n".join(output)