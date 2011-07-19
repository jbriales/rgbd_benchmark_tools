#!/usr/bin/python

import roslib; roslib.load_manifest('kinect_bagtools')
import rospy
import rosbag
import sensor_msgs.msg
import argparse
import sys
import os
import cv
import struct
import array
import tf
import binascii

from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This scripts reads a bag file containing kinect RGBD images and produces depth and rgb video. 
    ''')
    parser.add_argument('--start', help='skip the first N seconds of  input bag file')
    parser.add_argument('--duration', help='only process N seconds of  input bag file')
    parser.add_argument('inputbag', help='input bag file')
    args = parser.parse_args()
    
    rgbfile = os.path.splitext(args.inputbag)[0] + "-rgb"
    if not os.path.isdir(rgbfile): 
        os.mkdir(rgbfile)
    depthfile = os.path.splitext(args.inputbag)[0] + "-depth"
    if not os.path.isdir(depthfile): 
        os.mkdir(depthfile)
    tffile = os.path.splitext(args.inputbag)[0] + "-tf"
    if not os.path.isdir(tffile): 
        os.mkdir(tffile)
    imufile = os.path.splitext(args.inputbag)[0] + "-imu"
    if not os.path.isdir(imufile): 
        os.mkdir(imufile)
      
    print "Processing bag file:"
    print "  in:",args.inputbag
    print "  rgb:",rgbfile
    if args.start:
        print "  starting from: %s seconds"%(args.start)
    else:
        args.start = 0
        
    if args.duration:
        print "  duration: %s seconds"%(args.duration)
        
    inbag = rosbag.Bag(args.inputbag,'r')
    
    bridge = CvBridge()
    frame = 0 
    
    time_start = None
    depth_image=0
    for topic, msg, t in inbag.read_messages():
        if time_start==None:
            time_start=t
        if t - time_start < rospy.Duration.from_sec(float(args.start)):
            continue
        if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))):
            break
        print "t=%f\r"%(t-time_start).to_sec(),
        if topic == "/camera/depth/image":
            depth_image = msg
            cv_depth_image = bridge.imgmsg_to_cv(depth_image, "passthrough")
            img = cv.CreateImage( (640,480), 8, 3)
            for v in range(depth_image.height):
                for u in range(depth_image.width):
                    try:
                        d = (int) (cv_depth_image[v,u]*1000)
                        a = ((d / 256)%16)*16
                        
                        img[v,u] = (a + d%16,a+(d/16%16),a+(d/256*16)%16)
                    except:
                        img[v,u] = (0,0,0)
            cv.SaveImage(depthfile+"/%f.png"%depth_image.header.stamp.to_sec(),img)
        if topic == "/camera/rgb/image_color":
            rgb_image_color = msg
            cv_rgb_image_color = bridge.imgmsg_to_cv(rgb_image_color, "bgr8")
            cv.SaveImage(rgbfile+"/%f.png"%rgb_image_color.header.stamp.to_sec(),cv_rgb_image_color)
        if topic == "/tf":
           for t in msg.transforms:
               i=0
               while os.path.exists(tffile+"/%f-%d.txt"%(t.header.stamp.to_sec(),i)): i+=1
               f = open(tffile+"/%f-%d.txt"%(t.header.stamp.to_sec(),i),"w")
               f.write(t.__str__())
               f.close()
        if topic == "/imu":
               f = open(imufile+"/%f.txt"%msg.header.stamp.to_sec(),"w")
               f.write(msg.__str__())
               f.close()
    print