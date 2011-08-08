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
import numpy
import numpy.linalg.linalg
import geometry_msgs
import copy
import cb_detector

def transform44(ts):
    return numpy.dot(tf.listener.xyz_to_mat44(ts.transform.translation), tf.listener.xyzw_to_mat44(ts.transform.rotation))

def create_parser():
    parser = argparse.ArgumentParser(description='''
    This scripts reads a bag file containing kinect RGBD images and   
    estimates the calibration between the MoCap system and the Kinect optical frame using a Checkerboard or
    Optional arguments allow to select only a portion of the original bag file.  
    ''')
    parser.add_argument('--prop', nargs='?',action='append',
                        help='prop file from the motion capture system (definition of rigid bodies)')
    parser.add_argument('--start', help='skip the first N seconds of  input bag file')
    parser.add_argument('--duration', help='only process N seconds of  input bag file')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputbag', nargs='?',help='output bag file')
    args = parser.parse_args()
    
    if not args.outputbag:
        args.outputbag = os.path.splitext(args.inputbag)[0] + "-calibration.bag"
      
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
    return args


def read_prop_files(filelist):
    model_points = {}
    # parse prop files
    for propfile in filelist:
        lines = open(propfile,"r").readlines()
        prop = dict([line.strip().split("=") for line in lines if len(line.strip().split("="))==2])
        points = [line.split(",")[0:3] for line in lines[-int(prop["NumberOfMarkers"]):]]
        points = numpy.matrix([[float(n)*0.001 for n in p] for p in points])
        model_points[ prop["Name"].lower() ] = points.transpose()
        print "Loaded prop file:",propfile
        print "  object name:",prop["Name"].lower()
        print model_points[ prop["Name"].lower() ]
        print
    return model_points

def align(model,data,frame_id,child_frame_id,stamp=None,show_error=False):
    numpy.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    W = numpy.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    try:
        U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
        S = numpy.matrix(numpy.identity( 3 ))
        if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
            S[2,2] = -1
        rot = U*S*Vh
        trans = data.mean(1) - rot * model.mean(1)
    except Exception as e:
        print "WARNING: ",e
        print "model (%s)="%frame_id
        print model
        print "data (%s)="%child_frame_id
        print data
        
        return None
        
    
    sqrErr = 0
    model_aligned = rot * model + trans
    alignment_error = model_aligned - data
    
    sqrErr = sum(sum(numpy.array(numpy.multiply(alignment_error,alignment_error)))) / model.shape[1]
    if(sqrErr>0.005 or show_error):
        print "WARNING: Average error for '%s' to '%s' = %0.2fmm over n=%d points"%(frame_id,child_frame_id,sqrErr*1000,model.shape[1])
        
    mat44 = [
             [rot[0,0],rot[0,1],rot[0,2],trans[0]],
             [rot[1,0],rot[1,1],rot[1,2],trans[1]],
             [rot[2,0],rot[2,1],rot[2,2],trans[2]],
             [0,0,0,1]
             ]

    xyz = tuple(tf.transformations.translation_from_matrix(mat44))[:3]
    quat = tuple(tf.transformations.quaternion_from_matrix(mat44))
    stamped_transform = geometry_msgs.msg.TransformStamped()
    stamped_transform.transform.translation = geometry_msgs.msg.Vector3(*xyz)
    stamped_transform.transform.rotation = geometry_msgs.msg.Quaternion(*quat)
    stamped_transform.header.frame_id = frame_id
    stamped_transform.child_frame_id = child_frame_id
    stamped_transform.header.stamp = stamp
    return stamped_transform


def get_rgb_to_mocap_checkerboard(model_points):
    """This function computes the transform between the mocap markers in the mocap frame 
       and the mocap markers in the frame of the optical checkerboard. 
    """
    points_oncheckerboard = numpy.matrix([
              [ 0.16, -0.02, -0.05 ],
              [ 0.16,  0.12, -0.05 ],
              [-0.02,  0.12, -0.05 ],
              [-0.008, -0.07, -0.05 ],
              [-0.02, -0.02, -0.05 ]
            ]).transpose()
    return align(points_oncheckerboard,model_points["checkerboard"],
                 "/checkerboard","/mocap_checkerboard")

def detect_checkerboard(color_image,camera_info,child_frame_id):
    rgb_to_checkerboard = detector.detect(color_image,camera_info)
    if not rgb_to_checkerboard:
        return None
    pose = geometry_msgs.msg.TransformStamped()
    pose.transform.translation.x = rgb_to_checkerboard.pose.position.x
    pose.transform.translation.y = rgb_to_checkerboard.pose.position.y
    pose.transform.translation.z = rgb_to_checkerboard.pose.position.z
    pose.transform.rotation.x = rgb_to_checkerboard.pose.orientation.x
    pose.transform.rotation.y = rgb_to_checkerboard.pose.orientation.y
    pose.transform.rotation.z = rgb_to_checkerboard.pose.orientation.z
    pose.transform.rotation.w = rgb_to_checkerboard.pose.orientation.w
    pose.header = copy.deepcopy(color_image.header)
    pose.header.frame_id = "/openni_rgb_optical_frame"
    pose.child_frame_id = "/rgb_checkerboard"
    return pose

def get_transform(tf_buffer,from_frame,to_frame,visited_frames=[]):
    if from_frame == to_frame:
        return numpy.eye(4,4)
#    print "? %s --> %s"%(from_frame,to_frame)
    for (a,b),transform_a_b in tf_buffer.iteritems():
        if a in visited_frames or b in visited_frames: continue;
        if (a!=from_frame): continue;
        mat44_b_to = get_transform(tf_buffer,b,to_frame,visited_frames+[from_frame])
        if tf:
#            print "%s --> %s --> %s"%(from_frame,b,to_frame)
            mat44_a_b = transform44(transform_a_b)
            return mat44_a_b.dot(mat44_b_to)
        
    for (a,b),transform_a_b in tf_buffer.iteritems():
        if a in visited_frames or b in visited_frames: continue;
        if (b!=from_frame): continue;
        mat44_a_to = get_transform(tf_buffer,a,to_frame,visited_frames+[from_frame])
        if tf:
            mat44_b_a = numpy.linalg.inv(transform44(transform_a_b))
            return mat44_b_a.dot(mat44_a_to)

if __name__ == '__main__':
    args = create_parser()
    model_points = read_prop_files(args.prop)
    
    inbag = rosbag.Bag(args.inputbag,'r')

    transformer = tf.TransformerROS()
    time_start = None
    detector = cb_detector.ImageCbDetectorNode()
    camera_info = None
    
    tf_buffer={}
    
    # add transformation between optical checkerboard and mocap checkerboard
    transform = get_rgb_to_mocap_checkerboard(model_points)
    tf_buffer[ (transform.header.frame_id,transform.child_frame_id) ] = transform
    
    
    # we use the 4 checkerboard corner points used for calibration of mocap <--> rgb camera 
    corners_x = 8
    corners_y = 6
    checkerboard_size = 0.02
    checkerboard_points = numpy.matrix([
        [0,0,0,0],
        [0,corners_x*checkerboard_size,0,corners_x*checkerboard_size],
        [0,0,corners_y*checkerboard_size,corners_y*checkerboard_size],
        [1,1,1,1],
    ])
    
    mocap_points=numpy.zeros([3,0])
    rgb_points=numpy.zeros([3,0])
    for topic, msg, t in inbag.read_messages():
        # print progress info
        if time_start==None: time_start=t
        if t - time_start < rospy.Duration.from_sec(float(args.start)): continue
        if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))): break
        print "t=%f\r"%(t-time_start).to_sec(),
        
        # this gives us the TF from /world to /kinect and to /checkerboard 
        if topic=="/cortex_marker_array":
            for object in msg.markers:
                if object.ns.lower() in model_points.keys():
                    #print "cortex: %s"%object.ns.lower()
                    use_points = [not(numpy.isnan(p.x) or numpy.isnan(p.y) or numpy.isnan(p.z)) for p in object.points]
                    good_points = [1 for p in use_points if p]
                    good_points_index = [i for i,p in enumerate(use_points) if p]
                    if sum(good_points) < 3: continue
                    
                    data_goodpoints = numpy.matrix([[p.x,p.y,p.z] for p in object.points]).transpose()[:,good_points_index]
                    model_goodpoints = model_points[object.ns.lower()][:,good_points_index]
                    
                    transform = align(model_goodpoints,data_goodpoints,
                                                   "/world","/"+object.ns.lower(),object.header.stamp)
                    tf_buffer[ (transform.header.frame_id,transform.child_frame_id) ] = transform
        
        # PART 2
        # this gives us the TF from /openni_rgb_optical_frame to / 
        if topic=="/camera/rgb/camera_info":
            camera_info = msg
        if topic=="/tf":
            for transform in msg.transforms:
                tf_buffer[ (transform.header.frame_id,transform.child_frame_id) ] = transform
        if topic=="/camera/rgb/image_color" and camera_info:
            # detect checkerboard
            transform = detect_checkerboard(msg,camera_info,"/rgb_checkerboard")
            
            if(transform):
                tf_buffer[ (transform.header.frame_id,transform.child_frame_id) ] = transform
        
            mat44_mocap_checkerboard = get_transform(tf_buffer,"/kinect","/mocap_checkerboard")
            mat44_rgb_checkerboard = get_transform(tf_buffer,"/openni_camera","/rgb_checkerboard")
            if mat44_mocap_checkerboard == None:
                print "WARNING: couldn't resolve /kinect --> /mocap_checkerboard"
                print "TF keys: ",tf_buffer.keys()
                continue
            if mat44_rgb_checkerboard == None:
                print "WARNING: couldn't resolve /openni_camera --> /rgb_checkerboard"
                print "TF keys: ",tf_buffer.keys()
                continue
            
            mocap_points = numpy.hstack((mocap_points,mat44_mocap_checkerboard.dot(checkerboard_points)[0:3,:]))
            rgb_points = numpy.hstack((rgb_points,mat44_rgb_checkerboard.dot(checkerboard_points)[0:3,:]))
            
            align(mocap_points,rgb_points,"mocap","rgb",show_error=True)
            