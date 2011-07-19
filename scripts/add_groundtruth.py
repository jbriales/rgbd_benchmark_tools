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
import cb_detector

def transform44(ts):
    return numpy.dot(tf.listener.xyz_to_mat44(ts.transform.translation), tf.listener.xyzw_to_mat44(ts.transform.rotation))

if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This scripts reads a bag file containing kinect RGBD images and a calibration bag file (containing the missing TFs) and   
    adds this transformation to the TF stream.
    Optional arguments allow to select only a portion of the original bag file.  
    ''')
    parser.add_argument('--prop', nargs='?',action='append',
                        help='prop file from the motion capture system (definition of rigid bodies)')
    parser.add_argument('--start', help='skip the first N seconds of  input bag file')
    parser.add_argument('--duration', help='only process N seconds of  input bag file')
    parser.add_argument('--calibration', help='read tf calibration from bag file')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputbag', nargs='?',help='output bag file')
    args = parser.parse_args()
    
    if not args.outputbag:
        args.outputbag = os.path.splitext(args.inputbag)[0] + "-gt.bag"
      
    print "Processing bag file:"
    print "  in:",args.inputbag
    print "  out:",args.outputbag
    print
    
    
    if args.start:
        print "  starting from: %s seconds"%(args.start)
    else:
        args.start = 0
        
    if args.duration:
        print "  duration: %s seconds"%(args.duration)

    inbag = rosbag.Bag(args.inputbag,'r')
    outbag = rosbag.Bag(args.outputbag, 'w') #compression=rosbag.bag.Compression.BZ2

    transformer = tf.TransformerROS()
    time_start = None
    detector = cb_detector.ImageCbDetectorNode()
    camera_info = None
    tf_buffer={}
    
    # parse prop files
    model_points = {}
    for propfile in args.prop:
        lines = open(propfile,"r").readlines()
        prop = dict([line.strip().split("=") for line in lines if len(line.strip().split("="))==2])
        points = [line.split(",")[0:3] for line in lines[-int(prop["NumberOfMarkers"]):]]
        points = numpy.matrix([[float(n)*0.001 for n in p] for p in points])
        model_points[ prop["Name"] ] = points - points.mean(0).repeat(points.shape[0],axis=0)
        print "Loaded prop file:",propfile
        print "  object name:",prop["Name"]
        print model_points[ prop["Name"] ]
        print    
    
    if args.calibration:
        print "Loading calibration from ",args.calibration
        for topic,calibmsg,t in rosbag.Bag(args.calibration).read_messages(topics=["/tf"]):
            for transform in calibmsg.transforms:
                print "adding transform: ",(transform.header.frame_id,transform.child_frame_id)
                tf_buffer[ (transform.header.frame_id,transform.child_frame_id) ] = transform
    
    for topic, msg, t in inbag.read_messages():
        if time_start==None:
            time_start=t
        if t - time_start < rospy.Duration.from_sec(float(args.start)):
            continue
        if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))):
            break
        print "t=%f\r"%(t-time_start).to_sec(),
        outbag.write(topic,msg,t)
#        if topic=="/tf":
#            for t in msg.transforms:
#                print t.header.stamp.secs,t.header.stamp.nsecs,t.header.frame_id,t.child_frame_id
        if topic=="/cortex_marker_array":
            tfmsg = copy.deepcopy(calibmsg)

            object = None
            for object in msg.markers:
                if object.ns in model_points.keys():
                    use_row = [not(numpy.isnan(p.x) or numpy.isnan(p.y) or numpy.isnan(p.z)) for p in object.points]
                    good_points = [1 for p in use_row if p]
                    if sum(good_points) < 3: continue
                    points = numpy.matrix([[p.x,p.y,p.z] for p in object.points])
                    obs_points = points - points.mean(0).repeat(points.shape[0],axis=0)
                    W = numpy.zeros( (3,3) )
                    for row in range(obs_points.shape[0]):
                        W += numpy.outer(obs_points[row,:],model_points[object.ns][row,:])
                    try:
                        U,d,Vh = numpy.linalg.linalg.svd(W)
                    except:
                        continue
                    S = numpy.matrix(numpy.identity( 3 ))
                    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
                        S[2,2] = -1
                    rot = U*S*Vh
                    trans = points.mean(0)
                    
                    sqrErr =0
                    for row in range(obs_points.shape[0]):
                        err=points[row,:] - (rot*model_points[object.ns][row,:].transpose()).transpose() - trans
                        sqrErr += numpy.linalg.norm(err)
#                    print "error=",sqrErr
                    
                    mat44 = [
                             [rot[0,0],rot[0,1],rot[0,2],trans[0,0]],
                             [rot[1,0],rot[1,1],rot[1,2],trans[0,1]],
                             [rot[2,0],rot[2,1],rot[2,2],trans[0,2]],
                             [0,0,0,1]
                             ]
                    xyz = tuple(tf.transformations.translation_from_matrix(mat44))[:3]
                    quat = tuple(tf.transformations.quaternion_from_matrix(mat44))
                    pose = geometry_msgs.msg.TransformStamped()
                    pose.transform.translation = geometry_msgs.msg.Vector3(*xyz)
                    pose.transform.rotation = geometry_msgs.msg.Quaternion(*quat)
                    pose.header = copy.deepcopy(object.header)
                    #pose.header.frame_id = object.header.frame_id
                    pose.header.frame_id = "/world"
                    pose.child_frame_id = "/"+object.ns
                    tfmsg.transforms.append(pose)

            # update timestamps of calibration TFs
            if object!=None:
                for transform in tfmsg.transforms:
                    transform.header.stamp = copy.deepcopy(object.header.stamp)
                    
            # save calibration TFs + mocap TFs to output bag file
            if len(tfmsg.transforms)>0:
                outbag.write("/tf",tfmsg,t)
#                for t in tfmsg.transforms:
#                    print t.header.stamp.secs,t.header.stamp.nsecs,t.header.frame_id,t.child_frame_id
            #print tfmsg

    outbag.close()
    print