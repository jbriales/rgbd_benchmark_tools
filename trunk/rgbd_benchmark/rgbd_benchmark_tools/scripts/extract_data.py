#!/usr/bin/python

import roslib; roslib.load_manifest('rgbd_benchmark_tools')
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
    rgbstamps = os.path.splitext(args.inputbag)[0] + "-rgb.txt"
    if not os.path.isdir(rgbfile): 
        os.mkdir(rgbfile)
    depthfile16 = os.path.splitext(args.inputbag)[0] + "-depth"
    depthstamps = os.path.splitext(args.inputbag)[0] + "-depth.txt"
    if not os.path.isdir(depthfile16): 
        os.mkdir(depthfile16)
    depthfile8 = os.path.splitext(args.inputbag)[0] + "-depth8"
    if not os.path.isdir(depthfile8): 
        os.mkdir(depthfile8)
    imufile = os.path.splitext(args.inputbag)[0] + "-accelerometer.txt"
      
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
    imufile = open(imufile,"w")
    rgbstamps = open(rgbstamps,"w")
    depthstamps = open(depthstamps,"w")
    imufile.write("""# accelerometer data
# file: '%s'
# timestamp,ax,ay,az
"""%os.path.basename(args.inputbag))
    rgbstamps.write("""# color images
# file: '%s'
# timestamp,filename
"""%os.path.basename(args.inputbag))
    depthstamps.write("""# depth maps
# file: '%s'
# timestamp,filename
"""%os.path.basename(args.inputbag))
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
            img = cv.CreateImage( (640,480), 8, 1)
            for v in range(depth_image.height):
                for u in range(depth_image.width):
                    try:
                        d = (int) (cv_depth_image[v,u]*40)
                        img[v,u] = d
                    except:
                        img[v,u] = 0
            cv.SaveImage(depthfile8+"/%f.png"%depth_image.header.stamp.to_sec(),img)
            img = cv.CreateImage( (640,480), 16, 1)
            for v in range(depth_image.height):
                for u in range(depth_image.width):
                    try:
                        d = (int) (cv_depth_image[v,u]*5000)
                        img[v,u] = d
                    except:
                        img[v,u] = 0
            cv.SaveImage(depthfile16+"/%f.png"%depth_image.header.stamp.to_sec(),img)
            depthstamps.write("%+5.4f, %s\n"%(
		depth_image.header.stamp.to_sec(),
		os.path.basename(depthfile16)+"/%f.png"%depth_image.header.stamp.to_sec()))
        if topic == "/camera/rgb/image_color":
            rgb_image_color = msg
            cv_rgb_image_color = bridge.imgmsg_to_cv(rgb_image_color, "bgr8")
            cv.SaveImage(rgbfile+"/%f.png"%rgb_image_color.header.stamp.to_sec(),cv_rgb_image_color)
            rgbstamps.write("%+5.4f, %s\n"%(
		rgb_image_color.header.stamp.to_sec(),
		os.path.basename(rgbfile)+"/%f.png"%rgb_image_color.header.stamp.to_sec()))
        if topic == "/imu":
            imufile.write("%+5.4f, %+1.4f, %+1.4f, %+1.4f\n"%
                          (msg.header.stamp.to_sec(),
                           msg.linear_acceleration.x,
                           msg.linear_acceleration.y,
                           msg.linear_acceleration.z))
    print
    imufile.close()
    rgbstamps.close()
    depthstamps.close()

