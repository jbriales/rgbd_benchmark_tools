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
    
def generate_wikitext(bagfile,name):
    # arguments are bag file names
    output=[]
    filename = os.path.splitext(bagfile)[0]
    seqname = os.path.split(filename)[1][13:]
    stat=dict()
    statfile = os.path.splitext(bagfile)[0] + "-statistics.txt"
    if os.path.exists(statfile):
        stat=dict([line.strip().split("=") for line in open(statfile)])
    infofile = os.path.splitext(bagfile)[0] + "-info.txt"
    if os.path.exists(infofile):    
        info="".join(open(infofile).readlines())
    else:
        info="(no additional info available)"
    
    rgbpng = "%s-rgb.png"%(filename)
    depthpng = "%s-depth.png"%(filename)
    
    extavi = "%s-external.avi"%(filename)
    rgbavi = "%s-rgb.avi"%(filename)
    depthavi = "%s-depth.avi"%(filename)

    output.append("<table border=0><tr>")
    output.append("<td width=130px>")
    if os.path.exists(rgbpng):
        output.append("<img src='%s' width=120/><br>"%url(rgbpng))
    if os.path.exists(depthpng):
        output.append("<img src='%s' width=120/><br>"%url(depthpng))
    output.append("</td>")
    output.append("<td valign='top'>")
    output.append("<b><a name='%s'>Sequence '%s'</a></b><br>"%(seqname,seqname))
    output.append("<i>%s</i><br>"%info)
    output.append("<br>")
    
    bag = "%s.bag"%(filename)
    rgbavi = "%s-rgb.avi"%(filename)
    depthavi = "%s-depth.avi"%(filename)
    tgz = "%s.tgz"%(filename)
    output.append("<table border=0><tr><td valign='top' width='250px'>")
    output.append("Download this dataset as <br>")
    if os.path.exists(tgz):
        output.append("<a href='%s'>tgz</a> archieve or "%(url(tgz)))
    if os.path.exists(bag):
        output.append("<a href='%s'>ROS bag</a> file<br><br> "%(url(bag)))
    if os.path.exists(rgbavi):
        output.append("<a href='%s'>RGB</a> movie<br> "%(url(rgbavi)))
    if os.path.exists(depthavi):
        output.append("<a href='%s'>depth</a> movie<br> "%(url(depthavi)))
    if os.path.exists(extavi):
        output.append("movie from <a href='%s'>external</a> camera"%(url(extavi)))

    output.append("</td><td valign='top' width=250px>")
    if "duration" in stat: output.append("Duration: %s<br>"%stat["duration"])
    if "trajectory_length.translational" in stat: output.append("Trajectory length: %s<br>"%stat["trajectory_length.translational"])
    if "translational_velocity.mean" in stat: output.append("Avg. translational velocity: %s<br>"%stat["translational_velocity.mean"])
    if "angular_velocity.mean.deg" in stat: output.append("Avg. angular velocity: %s<br>"%stat["angular_velocity.mean.deg"])
    #output.append("Volume: %s x %s x %s<br>"%(stat["dimensions.x"],stat["dimensions.y"],stat["dimensions.z"]))

    output.append("</td></tr></table>")
    output.append("</td></tr></table><br>")

    s = "\n".join(output)
    return s

