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
    return args


def read_prop_files(filelist):
    model_points = {}
    # parse prop files
    for propfile in filelist:
        lines = open(propfile,"r").readlines()
        prop = dict([line.strip().split("=") for line in lines if len(line.strip().split("="))==2])
        points = [line.split(",")[0:3] for line in lines[-int(prop["NumberOfMarkers"]):]]
        points = numpy.matrix([[float(n)*0.001 for n in p] for p in points])
        model_points[ prop["Name"].lower() ] = points - points.mean(0).repeat(points.shape[0],axis=0)
        print "Loaded prop file:",propfile
        print "  object name:",prop["Name"].lower()
        print model_points[ prop["Name"].lower() ]
        print
    return model_points

def align(model,data,desc=None):
    model_zerocentered = model - model.mean(0).repeat(model.shape[0],axis=0)
    W = numpy.zeros( (3,3) )
    for row in range(model_zerocentered.shape[0]):
        W += numpy.outer(model_zerocentered[row,:],data[row,:])
    U,d,Vh = numpy.linalg.linalg.svd(W)
    S = numpy.matrix(numpy.identity( 3 ))
    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh
    trans = model.mean(0)
    
    if(desc):
        sqrErr =0
        for row in range(model.shape[0]):
            err=points[row,:] - (rot*model.transpose()).transpose() - data
            sqrErr += numpy.linalg.norm(err)
            print "Alignment error for '%s' = %f"%(desc,sqrErr)
    mat44 = [
             [rot[0,0],rot[0,1],rot[0,2],trans[0,0]],
             [rot[1,0],rot[1,1],rot[1,2],trans[0,1]],
             [rot[2,0],rot[2,1],rot[2,2],trans[0,2]],
             [0,0,0,1]
             ]

def get_rgb_to_mocap_checkerboard(model_points):
    """This function computes the transform between the mocap markers in the mocap frame 
       and the mocap markers in the frame of the optical checkerboard. 
    """
    points_oncheckerboard = numpy.matrix([
              [ 0.16, -0.02, -0.0 ],
              [ 0.16,  0.12, -0.0 ],
              [-0.02,  0.12, -0.0 ],
              [-0.008, -0.07, -0.0 ],
              [-0.02, -0.02, -0.0 ]
            ])
    mat44 = align(points_oncheckerboard,model_points["checkerboard"])
    
    rgb_to_mocap_checkerboard = create_stamped_transform("/rgb_checkerboard","/mocap_checkerboard",mat44)
    
    
    xyz = tuple(tf.transformations.translation_from_matrix(mat44))[:3]
    quat = tuple(tf.transformations.quaternion_from_matrix(mat44))
    rgb_to_mocap_checkerboard = geometry_msgs.msg.TransformStamped()
    rgb_to_mocap_checkerboard.transform.translation = geometry_msgs.msg.Vector3(*xyz)
    rgb_to_mocap_checkerboard.transform.rotation = geometry_msgs.msg.Quaternion(*quat)
    rgb_to_mocap_checkerboard.header.frame_id = "/rgb_checkerboard"
    rgb_to_mocap_checkerboard.child_frame_id = "/mocap_checkerboard"
    return rgb_to_mocap_checkerboard

if __name__ == '__main__':
    args = create_parser()
    model_points = read_prop_files(args.prop)
    rgb_to_mocap_checkerboard = get_rgb_to_mocap_checkerboard(model_points)
    
    sys.exit()
        
    inbag = rosbag.Bag(args.inputbag,'r')

    transformer = tf.TransformerROS()
    time_start = None
    detector = cb_detector.ImageCbDetectorNode()
    camera_info = None
    
    tf_buffer={}
    for topic, msg, t in inbag.read_messages():
        # print progress info
        if time_start==None: time_start=t
        if t - time_start < rospy.Duration.from_sec(float(args.start)): continue
        if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))): break
        print "t=%f\r"%(t-time_start).to_sec(),
        
        # this gives us the TF from /mocap to /Kinect 
        if topic=="/cortex_marker_array":
            tfmsg = tf.msg.tfMessage()
            for object in msg.markers:
                if object.ns.lower() in model_points.keys():
                    #print "cortex: %s"%object.ns.lower()
                    use_row = [not(numpy.isnan(p.x) or numpy.isnan(p.y) or numpy.isnan(p.z)) for p in object.points]
                    good_points = [1 for p in use_row if p]
                    if sum(good_points) < 3: continue
                    points = numpy.matrix([[p.x,p.y,p.z] for p in object.points])
                    obs_points = points - points.mean(0).repeat(points.shape[0],axis=0)
                    W = numpy.zeros( (3,3) )
                    for row in range(obs_points.shape[0]):
                        W += numpy.outer(obs_points[row,:],model_points[object.ns.lower()][row,:])
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
                        err=points[row,:] - (rot*model_points[object.ns.lower()][row,:].transpose()).transpose() - trans
                        sqrErr += numpy.linalg.norm(err)

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
                    pose.header.frame_id = object.header.frame_id
                    pose.child_frame_id = "/"+object.ns.lower()
                    tfmsg.transforms.append(pose)
            for transform in tfmsg.transforms:
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
            tfmsg = tf.msg.tfMessage()
            rgb_to_checkerboard = detector.detect(msg,camera_info)
            if rgb_to_checkerboard:
                #print "checkerboard detected"
                #tfmsg = tf.msg.tfMessage()
                pose = geometry_msgs.msg.TransformStamped()
                pose.transform.translation.x = rgb_to_checkerboard.pose.position.x
                pose.transform.translation.y = rgb_to_checkerboard.pose.position.y
                pose.transform.translation.z = rgb_to_checkerboard.pose.position.z
                pose.transform.rotation.x = rgb_to_checkerboard.pose.orientation.x
                pose.transform.rotation.y = rgb_to_checkerboard.pose.orientation.y
                pose.transform.rotation.z = rgb_to_checkerboard.pose.orientation.z
                pose.transform.rotation.w = rgb_to_checkerboard.pose.orientation.w
                pose.header = copy.deepcopy(camera_info.header)
                pose.header.frame_id = "/openni_rgb_optical_frame"
                pose.child_frame_id = "/rgb_checkerboard"
                tfmsg.transforms.append(pose)
                
                rgb_to_mocap_checkerboard.header.stamp = copy.deepcopy(camera_info.header.stamp)
                tfmsg.transforms.append(rgb_to_mocap_checkerboard)
                
                #print pose
                # pose gives transform: rgb to checkerboard
                #print tf_buffer.keys()
                try:
                    mat44_mocap_kinect = transform44(tf_buffer[("/mocap","/kinect")])
                    mat44_mocap_checkerboard = transform44(tf_buffer[("/mocap","/checkerboard")])
                    mat44_openni_rgb = transform44(tf_buffer[("/openni_camera","/openni_rgb_frame")])
                    mat44_rgb_optical = transform44(tf_buffer[("/openni_rgb_frame","/openni_rgb_optical_frame")])
                    mat44_optical_checkerboard = transform44(pose)
                    mat44_opticalcheckerboard_mocapcheckerboard = transform44(rgb_to_mocap_checkerboard)
                except:
                    print "waiting for tf"
                    continue
    
    #            print mat44_mocap_kinect
    #            print mat44_mocap_checkerboard
    #            print mat44_openni_rgb
    #            print mat44_rgb_optical
    #            print mat44_optical_checkerboard
#                print mat44_mocapcheckerboard_opticalcheckerboard
                
                mat44_kinect_openni = (
                    numpy.dot(
                    numpy.dot(
                    numpy.dot(
                    numpy.dot(
                    numpy.dot(
                    tf.transformations.inverse_matrix(mat44_mocap_kinect), 
                    mat44_mocap_checkerboard),
                    tf.transformations.inverse_matrix(mat44_opticalcheckerboard_mocapcheckerboard)),
                    tf.transformations.inverse_matrix(mat44_optical_checkerboard)),
                    tf.transformations.inverse_matrix(mat44_rgb_optical)),
                    tf.transformations.inverse_matrix(mat44_openni_rgb))
                    )
                sum_mat44_kinect_openni += mat44_kinect_openni
                count_mat44_kinect_openni += 1
                
                mat44_kinect_openni_avg = sum_mat44_kinect_openni / count_mat44_kinect_openni
                U,d,Vh = numpy.linalg.linalg.svd(mat44_kinect_openni_avg[0:2,0:2])
                mat44_kinect_openni_avg[0:2,0:2] = U*Vh

    if mat44_kinect_openni_avg != None and camera_info!=None:
        # now generate tf message
        tfmsg = tf.msg.tfMessage()
        xyz = tuple(tf.transformations.translation_from_matrix(mat44_kinect_openni_avg))[:3]
        quat = tuple(tf.transformations.quaternion_from_matrix(mat44_kinect_openni_avg))
        pose = geometry_msgs.msg.TransformStamped()
        pose.transform = geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(*xyz), geometry_msgs.msg.Quaternion(*quat))
        pose.header = copy.deepcopy(camera_info.header)
        pose.header.frame_id = "/kinect"
        pose.child_frame_id = "/openni_camera"
        tfmsg.transforms.append(pose)
                

        print "Saving calibration to ",args.outputbag
        for transform in tfmsg.transforms:
            print "adding transform: ",(transform.header.frame_id,transform.child_frame_id)
            
        print tfmsg
    
        outbag = rosbag.Bag(args.outputbag, 'w', compression=rosbag.bag.Compression.BZ2)
        outbag.write("/tf",tfmsg,t)
        outbag.close()
        print
    else:
        raise Exception("Could not compute calibration!")
        