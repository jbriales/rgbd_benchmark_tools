#!/usr/bin/python

import sys
import os

#file_prefix = "/home/sturmju/Desktop/tmp/"
file_prefix = "/usr/wiss/sturmju/public_html/"
#url_prefix = "/home/sturmju/Desktop/tmp/" 
url_prefix = "http://www9.in.tum.de/~sturmju/" 

def fileinfo(filename):
    if os.path.exists(filename):
        size = os.stat(filename).st_size;
        return "%0.1fMB"%(size / (1024*1024));
    else:
        return "n/a"

def url(filename):
    if not filename.startswith(file_prefix):
        raise Exception("wrong file prefix: %s, should have %s"%(filename,file_prefix))
    return url_prefix+filename[len(file_prefix):]

if len(sys.argv)<3 or os.path.splitext(sys.argv[1])[1]!=".html":
    print "Usage: generate_wikitext.py <output.html> <dataset1.bag> .."
    sys.exit()
    
htmlfile = sys.argv[1]

# arguments are bag file names
output=[]
for bagfile in sys.argv[2:]:
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
    rgbpng = "%s-rgb.png"%(filename)
    depthpng = "%s-depth.png"%(filename)
    
    hz=30
    extavi = "%s-external.avi"%(filename)
    rgbavi = "%s-%dhz-rgb.avi"%(filename,hz)
    depthavi = "%s-%dhz-depth.avi"%(filename,hz)
    gt = "%s-groundtruth.txt"%(filename)

    output.append("<table border=0><tr>")
    output.append("<td><a href='%s'><img src='%s' width=120/></a><br><a href='%s'><img src='%s' width=120/></a></td>"%
                  (url(rgbavi),url(rgbpng),url(depthavi),url(depthpng)))
    output.append("<td valign='top'>")
    output.append("<b>Sequence '%s'</b><br>"%seqname)
    output.append("<i>%s</i><br>"%info)
    output.append("<br>")
    
    output.append("<table border=0><tr><td valign='top' width='300px'>")
    output.append("Duration: %s<br>"%stat["duration"])
    output.append("Trajectory length: %s<br>"%stat["trajectory_length.translational"])
    output.append("Avg. translational velocity: %s<br>"%stat["translational_velocity.mean"])
    output.append("Avg. angular velocity: %s<br>"%stat["angular_velocity.mean.deg"])
    #output.append("Volume: %s x %s x %s<br>"%(stat["dimensions.x"],stat["dimensions.y"],stat["dimensions.z"]))


    output.append("</td><td valign='top'>")
    for hz in [2,5,10,30]:
        bag = "%s-%dhz.bag"%(filename,hz)
        rgbavi = "%s-%dhz-rgb.avi"%(filename,hz)
        depthavi = "%s-%dhz-depth.avi"%(filename,hz)
        tgz = "%s-%dhz.tgz"%(filename,hz)
        gt = "%s-%dhz-groundtruth.txt"%(filename,hz)
        imu = "%s-%dhz-imu.txt"%(filename,hz)
        output.append("%dHz: "%hz)
        output.append("<a href='%s'>tgz</a>, "%(url(tgz)))
        output.append("<a href='%s'>bag</a><br> "%(url(tgz)))
        #output.append("<a href='%s'>groundtruth</a>, "%gt)
    output.append("</td></tr><tr><td>")

    output.append("</td></tr></table>")
    output.append("More downloads: <br>")
    output.append("<a href='%s'>original</a> data, "%(url(originalbag)))
    output.append("<a href='%s'>ground-truth</a> trajectory, "%(url(originalgt)))
    output.append("<a href='%s'>RGB</a> movie, "%(url(rgbavi)))
    output.append("<a href='%s'>depth</a> movie, "%(url(depthavi)))
    output.append("<a href='%s'>external</a> camera"%(url(extavi)))
    output.append("</td></tr></table><br>")
            
    
s = "\n".join(output)
f = open(htmlfile,"w")
f.write(s)
f.close()
