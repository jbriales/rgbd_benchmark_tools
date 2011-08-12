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
    This scripts reads a bag file containing kinect RGBD images, 
    adds the corresponding PointCloud2 messages, and saves it again into a bag file. 
    Optional arguments allow to select only a portion of the original bag file.  
    ''')
    parser.add_argument('--start', help='skip the first N seconds of  input bag file',default=0.00)
    parser.add_argument('--duration', help='only process N seconds of  input bag file')
    parser.add_argument('--nth', help='only process every N-th frame of input bag file',default=1)
    parser.add_argument('--skip', help='skip N blocks in the beginning', default=1)
    parser.add_argument('--nopoints', help='do not add point clouds', action='store_true')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputbag', nargs='?',help='output bag file')
    args = parser.parse_args()
    
    if not args.outputbag:
        args.outputbag = os.path.splitext(args.inputbag)[0] + "-points.bag"
      
    print "Processing bag file:"
    print "  in:",args.inputbag
    print "  out:",args.outputbag
    print "  starting from: %s seconds"%(args.start)
        
    if args.duration:
        print "  duration: %s seconds"%(args.duration)
        
    print "  saving every %s-th frame"%(args.nth)
    args.skip = float(args.skip)
    print "  skipping %s blocks"%(args.skip)

    inbag = rosbag.Bag(args.inputbag,'r')
    outbag = rosbag.Bag(args.outputbag, 'w', compression=rosbag.bag.Compression.BZ2)
    
    depth_camera_info = None
    rgb_camera_info = None
    depth_image = None
    rgb_image_color = None
    cortex = None

    nan = float('nan')
    bridge = CvBridge()
    frame = 0 
    transforms = dict()
    
    time_start = None
    for topic, msg, t in inbag.read_messages():
        if time_start==None:
            time_start=t
        if t - time_start < rospy.Duration.from_sec(float(args.start)):
            continue
        if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))):
            break
        print "t=%f\r"%(t-time_start).to_sec(),
        if topic == "/cortex_marker_array":
            cortex = msg
            continue
        if topic == "/tf":
            for transform in msg.transforms:
                transforms[ (transform.header.frame_id,transform.child_frame_id) ] = transform
            continue
        if topic == "/imu":
            imu = msg
            continue
        if topic == "/camera/depth/camera_info":
            depth_camera_info = msg
            continue
        if topic == "/camera/rgb/camera_info":
            rgb_camera_info = msg
            continue
        if topic == "/camera/rgb/image_color" and rgb_camera_info:
            rgb_image_color = msg
            continue
        if topic == "/camera/depth/image" and depth_camera_info and rgb_image_color and rgb_camera_info:
            depth_image = msg
            # now process frame
            
            frame += 1
            if frame % float(args.nth) ==0:
                if args.skip > 0:
                    args.skip -= 1
                    continue
                # store messages
                msg = tf.msg.tfMessage()
                msg.transforms = list( transforms.itervalues() ) 
                outbag.write("/tf",msg,t)
                if cortex:
                    outbag.write("/cortex_marker_array",cortex,t)
                transforms = dict()
                outbag.write("/imu",imu,t)
                outbag.write("/camera/depth/camera_info",depth_camera_info,t)
                outbag.write("/camera/depth/image",depth_image,t)
                outbag.write("/camera/rgb/camera_info",rgb_camera_info,t)
                outbag.write("/camera/rgb/image_color",rgb_image_color,t)
                # consume the images
                imu = None
                depth_image = None
                rgb_image_color = None
#                print depth_camera_info.header.stamp.to_sec()
#                print depth_image.header.stamp.to_sec()
#                print rgb_camera_info.header.stamp.to_sec()
#                print rgb_image_color.header.stamp.to_sec()

                # generate monochrome image from color image
#                cv_rgb_image_mono = bridge.imgmsg_to_cv(rgb_image_color, "mono8")
#                rgb_image_mono = bridge.cv_to_imgmsg(cv_rgb_image_mono)
#                rgb_image_mono.header = rgb_image_color.header
#                outbag.write("/camera/rgb/image_mono",rgb_image_mono,t)
                
                # generate depth and colored point cloud
                cv_depth_image = bridge.imgmsg_to_cv(depth_image, "passthrough")
                cv_rgb_image_color = bridge.imgmsg_to_cv(rgb_image_color, "bgr8")
                #cv.NamedWindow("image_scaled")
                #cv.ShowImage("image_scaled", cv_rgb_image_color)
                #cv.WaitKey(10)
#                
                if args.nopoints:
                    continue
                centerX = depth_camera_info.K[2]
                centerY = depth_camera_info.K[5]
                depthFocalLength = depth_camera_info.K[0]
                depth_points = sensor_msgs.msg.PointCloud2()
                depth_points.header = depth_image.header
                depth_points.width = depth_image.width
                depth_points.height  = depth_image.height
                depth_points.fields.append(sensor_msgs.msg.PointField(
                    name = "x",offset = 0,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                depth_points.fields.append(sensor_msgs.msg.PointField(
                    name = "y",offset = 4,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                depth_points.fields.append(sensor_msgs.msg.PointField(
                    name = "z",offset = 8,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                depth_points.point_step = 16 
                depth_points.row_step = depth_points.point_step * depth_points.width
                buffer = []
                buffer_rgb = []
                for v in range(depth_image.height):
                    for u in range(depth_image.width):
                        d = cv_depth_image[v,u]
                        ptx = (u - centerX) * d / depthFocalLength;
                        pty = (v - centerY) * d / depthFocalLength;
                        ptz = d;
                        buffer.append(struct.pack('ffff',ptx,pty,ptz,1.0))
                depth_points.data = "".join(buffer)
                outbag.write("/camera/depth/points", depth_points, t)
                
                centerX = depth_camera_info.K[2]
                centerY = depth_camera_info.K[5]
                depthFocalLength = depth_camera_info.K[0]
                rgb_points = sensor_msgs.msg.PointCloud2()
                rgb_points.header = rgb_image_color.header
                rgb_points.width = depth_image.width
                rgb_points.height  = depth_image.height
                rgb_points.fields.append(sensor_msgs.msg.PointField(
                    name = "x",offset = 0,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                rgb_points.fields.append(sensor_msgs.msg.PointField(
                    name = "y",offset = 4,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                rgb_points.fields.append(sensor_msgs.msg.PointField(
                    name = "z",offset = 8,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                rgb_points.fields.append(sensor_msgs.msg.PointField(
                    name = "rgb",offset = 16,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                rgb_points.point_step = 32 
                rgb_points.row_step = rgb_points.point_step * rgb_points.width
                buffer = []
                for v in range(depth_image.height):
                    for u in range(depth_image.width):
                        d = cv_depth_image[v,u]
                        rgb = cv_rgb_image_color[v,u]
                        ptx = (u - centerX) * d / depthFocalLength;
                        pty = (v - centerY) * d / depthFocalLength;
                        ptz = d;
                        buffer.append(struct.pack('ffffBBBBIII',
                            ptx,pty,ptz,1.0,
                            rgb[0],rgb[1],rgb[2],0,
                            0,0,0))
                rgb_points.data = "".join(buffer)
                outbag.write("/camera/rgb/points", rgb_points, t)                
            continue
        # anything else: pass thru
        outbag.write(topic,msg,t)
                
                #print frame
                
    outbag.close()
    print
