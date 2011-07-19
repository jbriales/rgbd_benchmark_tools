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
    This scripts reads a bag file containing kinect RGBD images and either  
    (1) estimates the calibration between the MoCap system and the Kinect optical frame using a Checkerboard or
    (2)  adds this transformation to the TF stream.
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
        
    if "CheckerBoard" in model_points.keys():
        points = numpy.matrix([
                  [ 0.16, -0.02, -0.0 ],
                  [ 0.16,  0.12, -0.0 ],
                  [-0.02,  0.12, -0.0 ],
                  [-0.008, -0.07, -0.0 ],
                  [-0.02, -0.02, -0.0 ]
                ])
        obs_points = points - points.mean(0).repeat(points.shape[0],axis=0)
        W = numpy.zeros( (3,3) )
        for row in range(obs_points.shape[0]):
            W += numpy.outer(obs_points[row,:],model_points["CheckerBoard"][row,:])
        U,d,Vh = numpy.linalg.linalg.svd(W)
        S = numpy.matrix(numpy.identity( 3 ))
        if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
            S[2,2] = -1
        rot = U*S*Vh
        trans = points.mean(0)
        #print rot,trans
        sqrErr =0
        for row in range(obs_points.shape[0]):
            err=points[row,:] - (rot*model_points["CheckerBoard"][row,:].transpose()).transpose() - trans
            sqrErr += numpy.linalg.norm(err)
        print sqrErr
        mat44 = [
                 [rot[0,0],rot[0,1],rot[0,2],trans[0,0]],
                 [rot[1,0],rot[1,1],rot[1,2],trans[0,1]],
                 [rot[2,0],rot[2,1],rot[2,2],trans[0,2]],
                 [0,0,0,1]
                 ]
        xyz = tuple(tf.transformations.translation_from_matrix(mat44))[:3]
        quat = tuple(tf.transformations.quaternion_from_matrix(mat44))
        rgb_to_mocap_checkerboard = geometry_msgs.msg.TransformStamped()
        rgb_to_mocap_checkerboard.transform.translation = geometry_msgs.msg.Vector3(*xyz)
        rgb_to_mocap_checkerboard.transform.rotation = geometry_msgs.msg.Quaternion(*quat)
        rgb_to_mocap_checkerboard.header.frame_id = "/rgb_checkerboard"
        rgb_to_mocap_checkerboard.child_frame_id = "/mocap_checkerboard"

    inbag = rosbag.Bag(args.inputbag,'r')
    #outbag = rosbag.Bag(args.outputbag, 'w')
    outbag = rosbag.Bag(args.outputbag, 'w', compression=rosbag.bag.Compression.BZ2)

    transformer = tf.TransformerROS()
    time_start = None
    detector = cb_detector.ImageCbDetectorNode()
    camera_info = None
    tf_buffer={}
    sum_mat44_kinect_openni = numpy.matrix(numpy.zeros( (4,4) ))
    count_mat44_kinect_openni = 0
    mat44_kinect_openni_avg = None
    
    if args.calibration:
        print "Loading calibration from ",args.calibration
        for topic,msg,t in rosbag.Bag(args.calibration).read_messages(topics=["/tf"]):
            for transform in msg.transforms:
                tf_buffer[ (transform.header.frame_id,transform.child_frame_id) ] = transform
        mat44_kinect_openni_avg = transform44(tf_buffer[("/Kinect","/openni_camera")])
        print mat44_kinect_openni_avg
    
    tf_buffer={}
    for topic, msg, t in inbag.read_messages():
        if time_start==None:
            time_start=t
        if t - time_start < rospy.Duration.from_sec(float(args.start)):
            continue
        if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))):
            break
        print "t=%f\r"%(t-time_start).to_sec(),
        outbag.write(topic,msg,t)
        if topic=="/cortex_marker_array":
            tfmsg = tf.msg.tfMessage()
            for object in msg.markers:
                if object.ns in model_points.keys():
                    use_row = [not(numpy.isnan(p.x) or numpy.isnan(p.y) or numpy.isnan(p.z)) for p in object.points]
                    good_points = [1 for p in use_row if p]
                    if sum(good_points) < 3: continue
#                    print "processing",object.ns
                    points = numpy.matrix([[p.x,p.y,p.z] for p in object.points])
                    obs_points = points - points.mean(0).repeat(points.shape[0],axis=0)
                    #print obs_points
                    #print model_points[object.ns]
                    W = numpy.zeros( (3,3) )
                    for row in range(obs_points.shape[0]):
                        W += numpy.outer(obs_points[row,:],model_points[object.ns][row,:])
                    #print W
                    try:
                        U,d,Vh = numpy.linalg.linalg.svd(W)
                    except:
                        continue
#                    print "u=",U
#                    print "d=",d
#                    print "vh=",Vh
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
                    
#                    rot = rot.transpose()
#                    trans = (rot * (-trans.transpose())).transpose()

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
                    pose.child_frame_id = "/"+object.ns
                    #print pose
                    tfmsg.transforms.append(pose)
            # save to file..
            for transform in tfmsg.transforms:
                tf_buffer[ (transform.header.frame_id,transform.child_frame_id) ] = transform
            if len(tfmsg.transforms)>0:
                outbag.write("/tf",tfmsg,t)
        if topic=="/camera/rgb/camera_info":
            camera_info = msg
        if topic=="/tf":
            tfmsg = copy.deepcopy(msg)
            for transform in msg.transforms:
                tf_buffer[ (transform.header.frame_id,transform.child_frame_id) ] = transform
            # add current estimate of kinect/mocap
            if mat44_kinect_openni_avg != None and camera_info!=None:
                xyz = tuple(tf.transformations.translation_from_matrix(mat44_kinect_openni_avg))[:3]
                quat = tuple(tf.transformations.quaternion_from_matrix(mat44_kinect_openni_avg))
                pose = geometry_msgs.msg.TransformStamped()
                pose.transform = geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(*xyz), geometry_msgs.msg.Quaternion(*quat))
                pose.header = copy.deepcopy(camera_info.header)
                pose.header.frame_id = "/Kinect"
                pose.child_frame_id = "/openni_camera"
                tfmsg.transforms.append(pose)
                
                try:
                    mat44_mocap_kinect = transform44(tf_buffer[("/mocap","/Kinect")])
                    mat44_openni_rgb = transform44(tf_buffer[("/openni_camera","/openni_rgb_frame")])
                    mat44_rgb_optical = transform44(tf_buffer[("/openni_rgb_frame","/openni_rgb_optical_frame")])
                    mat44_mocap_optical = numpy.dot(numpy.dot(numpy.dot(
                            mat44_mocap_kinect,mat44_kinect_openni_avg),  mat44_openni_rgb   ),mat44_rgb_optical)
                            
                    xyz = tuple(tf.transformations.translation_from_matrix(mat44_mocap_optical))[:3]
                    quat = tuple(tf.transformations.quaternion_from_matrix(mat44_mocap_optical))
                    pose = geometry_msgs.msg.TransformStamped()
                    pose.transform = geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(*xyz), geometry_msgs.msg.Quaternion(*quat))
                    pose.header = copy.deepcopy(camera_info.header)
                    pose.header.frame_id = "/mocap"
                    pose.child_frame_id = "/openni_rgb_optical_frame_direct"
                    tfmsg.transforms.append(pose)
                except:
                    pass
            outbag.write("/tf",tfmsg,t)
        if topic=="/camera/rgb/image_color" and camera_info and not args.calibration:
            # detect checkerboard
            rgb_to_checkerboard = detector.detect(msg,camera_info)
            if rgb_to_checkerboard:
                tfmsg = tf.msg.tfMessage()
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
                
                outbag.write("/tf",tfmsg,t)
                
                #print pose
                # pose gives transform: rgb to checkerboard
                #print tf_buffer.keys()
                try:
                    mat44_mocap_kinect = transform44(tf_buffer[("/mocap","/Kinect")])
                    mat44_mocap_checkerboard = transform44(tf_buffer[("/mocap","/CheckerBoard")])
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
                #print
                #print mat44_kinect_openni
                #print numpy.dot(  )
                # average transform
                sum_mat44_kinect_openni += mat44_kinect_openni
                count_mat44_kinect_openni += 1
                
                mat44_kinect_openni_avg = sum_mat44_kinect_openni / count_mat44_kinect_openni
                U,d,Vh = numpy.linalg.linalg.svd(mat44_kinect_openni_avg[0:2,0:2])
                mat44_kinect_openni_avg[0:2,0:2] = U*Vh
                
#                mat44_kinect_openni_avg = mat44_kinect_openni
#                xyz = tuple(tf.transformations.translation_from_matrix(mat44_kinect_openni))[:3]
#                quat = tuple(tf.transformations.quaternion_from_matrix(mat44_kinect_openni))

    outbag.close()
    print