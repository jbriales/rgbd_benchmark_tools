#! /usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************

# Need to explicitly enable 'with' in python 2.5
from __future__ import with_statement

import roslib; roslib.load_manifest('rgbd_benchmark_tools')

import rospy
import cv
import math
import threading
import numpy
import tf

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge, CvBridgeError

class ImageCbDetector:
  def get_board_corners(self, corners, corners_x, corners_y):
    return (corners[0], corners[corners_x  - 1], 
        corners[(corners_y - 1) * corners_x], corners[len(corners) - 1])

  def detect(self, image, corners_x, corners_y, spacing_x, spacing_y, width_scaling, height_scaling):
    #resize the image base on the scaling parameters we've been configured with
    scaled_width = int(.5 + image.width * width_scaling)
    scaled_height = int(.5 + image.height * height_scaling)
    
    #in cvMat its row, col so height comes before width
    image_scaled = cv.CreateMat(scaled_height, scaled_width, cv.GetElemType(image))
    cv.Resize(image, image_scaled, cv.CV_INTER_LINEAR)

    #Here, we'll actually call the openCV detector    
    found, corners = cv.FindChessboardCorners(image_scaled, (corners_x, corners_y), cv.CV_CALIB_CB_ADAPTIVE_THRESH)

    if found:
      board_corners = self.get_board_corners(corners, corners_x, corners_y)
      
      #find the perimeter of the checkerboard
      perimeter = 0.0
      for i in range(len(board_corners)):
        next = (i + 1) % 4
        xdiff = board_corners[i][0] - board_corners[next][0]
        ydiff = board_corners[i][1] - board_corners[next][1]
        perimeter += math.sqrt(xdiff * xdiff + ydiff * ydiff)

      #estimate the square size in pixels
      square_size = perimeter / ((corners_x - 1 + corners_y - 1) * 2)
      radius = int(square_size * 0.5 + 0.5)

      corners = cv.FindCornerSubPix(image_scaled, corners, (radius, radius), (-1, -1), (cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 30, 0.1))

      #uncomment to debug chessboard detection
      #cv.DrawChessboardCorners(image_scaled, (corners_x, corners_y), corners, 1)
      #cv.NamedWindow("image_scaled")
      #cv.ShowImage("image_scaled", image_scaled)
      #cv.WaitKey(600)

      object_points = None

      #we'll also generate the object points if the user has specified spacing
      if spacing_x != None and spacing_y != None:
        object_points = cv.CreateMat(3, corners_x * corners_y, cv.CV_32FC1)

        for y in range(corners_y):
          for x in range(corners_x):
            cv.SetReal2D(object_points, 0, y*corners_x + x, x * spacing_x)
            cv.SetReal2D(object_points, 1, y*corners_x + x, y * spacing_y)
            cv.SetReal2D(object_points, 2, y*corners_x + x, 0.0)

      #not sure why opencv functions return non opencv compatible datatypes... but they do so we'll convert
      corners_cv = cv.CreateMat(2, corners_x * corners_y, cv.CV_32FC1)
      for i in range(corners_x * corners_y):
        cv.SetReal2D(corners_cv, 0, i, corners[i][0])
        cv.SetReal2D(corners_cv, 1, i, corners[i][1])

      return (corners_cv, object_points)

    else:
      #cv.NamedWindow("image_scaled")
      #cv.ShowImage("image_scaled", image_scaled)
      #cv.WaitKey(600)
      print "Didn't find checkerboard\r",
      return (None, None)

class ImageCbDetectorNode:
  def __init__(self):
    self.mutex = threading.RLock()
    self.corners_x = 8
    self.corners_y = 6
    self.spacing_x = 0.02
    self.spacing_y = 0.02
    self.width_scaling = 1
    self.height_scaling = 1

    self.im_cb_detector = ImageCbDetector()

    self.bridge = CvBridge()
    self.cam_info = None
    self.ros_image = None

  def intrinsic_matrix_from_info(self, cam_info):
    intrinsic_matrix = cv.CreateMat(3, 3, cv.CV_32FC1)

    #Because we only want the upper 3x3 (normal) portion of the rectified intrinsic matrix
    for i in range(0, 3):
      for j in range(0, 3):
        intrinsic_matrix[i, j] = cam_info.P[4*i+j]

    return intrinsic_matrix

  def detect(self, ros_image,cam_info):
      self.ros_image = ros_image
      self.cam_info = cam_info
      pose = self.find_checkerboard_pose(ros_image, self.corners_x, self.corners_y, self.spacing_x, self.spacing_y, self.width_scaling, self.height_scaling)
      return pose

  def find_checkerboard_pose(self, ros_image, corners_x, corners_y, spacing_x, spacing_y, width_scaling, height_scaling):
    #we need to convert the ros image to an opencv image
    try:
      image = self.bridge.imgmsg_to_cv(ros_image, "mono8")
    except CvBridgeError, e:
      rospy.logerror("Error importing image %s" % e)
      return

    corners, model = self.im_cb_detector.detect(image, corners_x, corners_y, spacing_x, spacing_y, width_scaling, height_scaling)

    if corners != None and model != None and self.cam_info != None:
      #find the pose of the checkerboard
      rot = cv.CreateMat(3, 1, cv.CV_32FC1)
      trans = cv.CreateMat(3, 1, cv.CV_32FC1)
      with self.mutex:
        kc = cv.CreateMat(1, 4, cv.CV_32FC1)
        cv.Set(kc, 0.0)
        intrinsic_matrix = self.intrinsic_matrix_from_info(self.cam_info)

        cv.FindExtrinsicCameraParams2(model, corners, intrinsic_matrix, kc, rot, trans)

      #We want to build a transform now, but first we have to convert the 
      #rotation vector we get back from OpenCV into a rotation matrix
      rot_mat = cv.CreateMat(3, 3, cv.CV_32FC1)
      cv.Rodrigues2(rot, rot_mat)

      #Now we need to convert this rotation matrix into a quaternion
      #This can be done better in never versions of opencv, but we need to be boxturtle compatible
      numpy_mat = numpy.fromstring(rot_mat.tostring(), dtype = numpy.float32).reshape((3,3))

      #of course, tf expects all matricies passed to it to be 4x4... so we'll build a full matrix here
      full_pose = numpy.zeros((4, 4))

      #first, copy in the rotation matrix
      full_pose[0:3, 0:3] = numpy_mat[0:3, 0:3]

      #next, we'll copy in the translation
      full_pose[0:3, 3] = [trans[i, 0] for i in range(3)]

      #and make sure to add a 1 in the lower right corner
      full_pose[3][3] = 1.0

      rospy.logdebug("%s" % numpy_mat)
      rospy.logdebug("%s" % full_pose)

      tf_trans = tf.transformations.translation_from_matrix(full_pose)
      tf_rot = tf.transformations.quaternion_from_matrix(full_pose)

      #and now we'll actually build our pose stamped
      board_pose = PoseStamped()
      board_pose.header = ros_image.header
      board_pose.pose.position.x = tf_trans[0]
      board_pose.pose.position.y = tf_trans[1]
      board_pose.pose.position.z = tf_trans[2]
      board_pose.pose.orientation.x = tf_rot[0]
      board_pose.pose.orientation.y = tf_rot[1]
      board_pose.pose.orientation.z = tf_rot[2]
      board_pose.pose.orientation.w = tf_rot[3]
      rospy.logdebug("%s" % board_pose)

      #we'll publish the pose so we can display it in rviz
      return board_pose
