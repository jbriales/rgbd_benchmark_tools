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
import numpy
import numpy.linalg.linalg
import geometry_msgs
import copy

def transform44(ts):
    return numpy.dot(tf.listener.xyz_to_mat44(ts.transform.translation), tf.listener.xyzw_to_mat44(ts.transform.rotation))


if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This scripts reads a bag file and outputs the TF transform from /mocap to /openni_rgb_optical_frame.
    ''')
    parser.add_argument('--start', help='skip the first N seconds of  input bag file')
    parser.add_argument('--duration', help='only process N seconds of  input bag file')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputlog', nargs='?',help='output log file')
    parser.add_argument('calibration', nargs='?',help='calibration tf (contains any missing/required tfs)')
    args = parser.parse_args()

    if not args.outputlog:
        args.outputlog = os.path.splitext(args.inputbag)[0] + ".log"
    
#    print "Processing bag file:"
#    print "  in:",args.inputbag
#    print
    
    
    if not args.start:
        args.start = 0
        
    if args.duration:
        print "  duration: %s seconds"%(args.duration)


    # read any TFs from calibration file and store in buffer
    tf_buffer = {}
    if args.calibration:
        calbag = rosbag.Bag(args.calibration,'r')
        for topic, msg, t in calbag.read_messages(topics=["/tf"]):
            for transform in msg.transforms:
                tf_buffer[ (transform.header.frame_id,transform.child_frame_id) ] = transform
    
    inbag = rosbag.Bag(args.inputbag,'r')
    time_start = None
    log = {}
    for topic, msg, t in inbag.read_messages(topics=["/tf"]):
        if time_start==None:
            time_start=t
        if t - time_start < rospy.Duration.from_sec(float(args.start)):
            continue
        if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))):
            break
        print "t=%f\r"%(t-time_start).to_sec(),
        if topic=="/tf":
            for transform in msg.transforms:
                tf_buffer[ (transform.header.frame_id,transform.child_frame_id) ] = transform
                if transform.child_frame_id=="/Kinect":
                    logtime = transform.header.stamp
            #print [(transform.header.frame_id,transform.child_frame_id) for transform in msg.transforms]
            # when we get an updated tranformation to the Kinect camera:
#            for tr in msg.transforms:
#                print tr.header.stamp.secs,tr.header.stamp.nsecs,tr.header.frame_id,tr.child_frame_id

            try:
                mat44_mocap_kinect = transform44(tf_buffer[("/world","/Kinect")])
                mat44_kinect_openni = transform44(tf_buffer[("/Kinect","/openni_camera")])
                mat44_openni_rgb = transform44(tf_buffer[("/openni_camera","/openni_rgb_frame")])
                mat44_rgb_optical = transform44(tf_buffer[("/openni_rgb_frame","/openni_rgb_optical_frame")])
                
                mat44 = numpy.dot(numpy.dot(numpy.dot(
                                                      mat44_mocap_kinect,
                                                      mat44_kinect_openni),
                                            mat44_openni_rgb),
                                  mat44_rgb_optical)
                
                xyz = tuple(tf.transformations.translation_from_matrix(mat44))[:3]
                quat = tuple(tf.transformations.quaternion_from_matrix(mat44))
                log[logtime.to_sec()] =  "%+5.4f,    %+1.4f, %+1.4f, %+1.4f,    %+1.4f, %+1.4f, %+1.4f, %+1.4f"%(logtime.to_sec(),xyz[0],xyz[1],xyz[2],quat[0],quat[1],quat[2],quat[3])
#                    print "t=%3.5f"%logtime.to_sec()  
            except KeyError:
                print "waiting for tf"
                continue
            except:
                raise
    print
    s = """# ground truth trajectory
# file: '%s'
# time,tx,ty,tz,qx,qy,qz,qw
"""%args.inputbag
    k = log.keys()
    k.sort()
    for t in k:
        s += log[t]+"\n"
    f = open(args.outputlog,"w")
    f.write(s)
    f.close
    print 
    print "Ground truth trajectory saved to '%s'"%args.outputlog
    