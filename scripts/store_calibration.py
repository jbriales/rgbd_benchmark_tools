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
    parser.add_argument('calibration', help='calibration tf (contains any missing/required tfs)')
    args = parser.parse_args()
    
    xyz = [0.0671756,0.045045,-0.00418294]
    quat = [0.561532,-0.00517817,0.0142313,-0.827316]

    outbag = rosbag.Bag(args.calibration,'w')
    tfmsg = tf.msg.tfMessage()
    pose = geometry_msgs.msg.TransformStamped()
    pose.transform.translation = geometry_msgs.msg.Vector3(*xyz)
    pose.transform.rotation = geometry_msgs.msg.Quaternion(*quat)
    pose.header.frame_id = "/Kinect"
    pose.child_frame_id = "/openni_camera"
    tfmsg.transforms.append(pose)
    outbag.write("/tf",tfmsg,rospy.Time.from_sec(float(0)))
    outbag.close()
