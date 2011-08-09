#!/usr/bin/python

import roslib; roslib.load_manifest('rgbd_benchmark_tools')
import rosbag
import tf, tf.msg
import geometry_msgs.msg

import sys, getopt, os.path
import numpy, numpy.linalg



def usage(name='./add_groundtruth_new.py'):
	print """This script reads a bag and generates a copy with additional ground
truth.
Usage:
    %s [opts] <inputbag>

Options:
    -h|--help: this help
    -o|--outputbag:    name of output bag (default: '<inputbag>-gt.bag')
    -m|--old-world-frame: name of old world frame (default: '/world')
    -w|--new-world-frame: name of the new world frame (default: '/world')
    -k|--old-object-frame: name of the old vicon/mocap frame (default:
            '/vicon_object')
    -f|--source-frame: name of the source frame of the calibration string (default:
			'/openni_rgb_optical_frame')
    -n|--new-groundtruth-frame: name of the new ground truth frame (default:
            '/openni_rgb_optical_frame_direct')
    -s|--calibration-string: name of a calibration string between
            <source-frame> and <old-object-frame>
    -b|--calibration-bag: name of a calibration bag between <end-frame> or
            <source-frame> and <old-object-frame> (default: 'calibration.bag',
            ignored if calibration string provided)
    -c|--compress: compress output bag
"""%name
#    -e|--end-frame: name of the end frame (default:
#            '/openni_rgb_optical_frame')


def main():
	# parse command line
	shopts = 'ho:m:w:k:f:n:s:b:c'
	lopts = ['help', 'outputbag=', 'old-world-frame=', 'new-world-frame=',
			'old-object-frame=', 'source-frame=', 'new-groundtruth-frame=',
			'calibration-string=', 'calibration-bag=','compress']
	try:
		opts, args = getopt.getopt(sys.argv[1:], shopts, lopts)

	except getopt.GetoptError, e:
		print e
		usage(sys.argv[0])
		return 1
	
	if len(args) < 1:
		usage(sys.argv[0])
		return 1
	inputbag = args[0]

	# default options
	outputbag = os.path.splitext(inputbag)[0] + '-gt.bag'
	oldworld = '/world'
	newworld = '/world'
	oldobject = '/vicon_object'
	source = '/openni_rgb_optical_frame'
	newgt = '/openni_rgb_optical_frame_direct'
	calibrationstring = ''
	calibrationbag = 'calibration.bag'
	calib_tf = None
	compress_mode = rosbag.bag.Compression.NONE


	# getting options
	for o, a in opts:
		if o in ('-h', '--help'):
			usage(sys.argv[0])
			return
		if o in ('-o', '--outputbag'):
			outputbag = a
		if o in ('-m', '--old-world-frame'):
			oldworld = a
		if o in ('-w', '--new-world-frame'):
			newworld = a
		if o in ('-k', '--old-object-frame'):
			oldobject = a
		if o in ('-f', '--source-frame'):
			source = a
		if o in ('-n', '--new-groundtruth-frame'):
			newgt = a
		if o in ('-s', '--calibration-string'):
			calibrationstring = a
		if o in ('-b', '--calibration-bag'):
			calibrationbag = a
		if o in ('-c', '--compress'):
			compress_mode = rosbag.bag.Compression.BZ2

	#getting static transforms:
	# openni_camera <- openni_optical_frame_rgb
	M_opt2cam = get_openni_matrix(inputbag)
	#print M_opt2cam

	# from camera frame to vicon/mocap object
	if calibrationstring:
		M_calib = get_calibration_from_string(calibrationstring)
		if source == '/openni_rgb_optical_frame':
			M_cam2obj = numpy.dot(M_calib,
					numpy.linalg.inv(M_opt2cam))	#TODO check
		elif source == '/openni_camera':
			M_cam2obj = M_calib
		else:
			print "Couldn't compute calibration between '/openni_camera' and\
			'%s' (from '%s')"%(oldobject, source)
	else:
		M_calib, frame = get_calibration_from_bag(calibrationbag, oldobject)
		if frame == '/openni_rgb_optical_frame':
			M_cam2obj = numpy.dot(M_calib,
					numpy.linalg.inv(M_opt2cam))	#TODO check
		elif frame == '/openni_camera':
			M_cam2obj = M_calib
		else:
			print "Couldn't compute calibration between '/openni_camera' and\
			'%s' (from '%s')"%(oldobject, frame)
	#print M_camera2object

	# do the recalibration
	calibrate(inputbag, outputbag, oldworld, newworld, oldobject, newgt, M_opt2cam, M_cam2obj,compress_mode)
	return 0



def calibrate(inbagname, outbagname, oldworld, newworld, oldobject, newgt, M_opt2cam, M_cam2obj,compress_mode):
	print "Starting calibration."
	inbag = rosbag.Bag(inbagname, 'r')
	outbag = rosbag.Bag(outbagname, 'w', compression=compress_mode)
	for topic, msg, t in inbag:
		if topic == "/tf":
			new_tf_list = []
			for mytf in msg.transforms:
				#print tf.header.frame_id, '<-', tf.child_frame_id
				if (mytf.header.frame_id == oldworld):
					if (mytf.child_frame_id == oldobject):
						print "Editing tf at time", mytf.header.stamp
						# computing new transforms
						M_obj2wld = mat_from_transform(mytf.transform)
						M_cam2wld = numpy.dot(M_obj2wld, M_cam2obj)
						M_opt2wld = numpy.dot(M_cam2wld, M_opt2cam)
						# editing current transform
						mytf.header.frame_id = newworld
						mytf.child_frame_id = '/openni_camera'
						mytf.transform = transform_from_mat(M_cam2wld)
						new_tf_list.append(mytf)
						# adding new ground truth
						new_tf = geometry_msgs.msg.TransformStamped()
						new_tf.header.stamp = mytf.header.stamp
						new_tf.header.frame_id = newworld
						new_tf.child_frame_id = newgt
						new_tf.transform = transform_from_mat(M_opt2wld)
						new_tf_list.append(new_tf)

					else:
						print "Ignoring tf: ", mytf.header.frame_id, '<-', mytf.child_frame_id
						pass
				else:
					new_tf_list.append(mytf)
			msg = tf.msg.tfMessage(new_tf_list)
		outbag.write(topic, msg, t)
	outbag.close()




def mat_from_transform(transform):
	vtr = (transform.translation.x, transform.translation.y,
			transform.translation.z)
	vrot = (transform.rotation.x, transform.rotation.y, transform.rotation.z,
			transform.rotation.w)
	return tf.TransformerROS().fromTranslationRotation(vtr, vrot)


def transform_from_mat(mat):
	trans = tf.transformations.translation_from_matrix(mat)
	quat = tf.transformations.quaternion_from_matrix(mat)
	transform = geometry_msgs.msg.Transform()
	transform.translation = geometry_msgs.msg.Vector3(*trans)
	transform.rotation = geometry_msgs.msg.Quaternion(*quat)
	return transform


def get_openni_matrix(bagname):
	'''get openni_camera <- openni_rgb_optical_frame from bag (via
	openni_rgb_frame'''
	bag = rosbag.Bag(bagname, 'r')
	tf_rgb2cam = None
	tf_optical2rgb = None
	for topic, msg, t in bag.read_messages(topics=['/tf']):
		for tf in msg.transforms:
			if (tf.header.frame_id == '/openni_camera') and\
					(tf.child_frame_id == '/openni_rgb_frame'):
				print "Got rgb2cam"
				#print tf
				tf_rgb2cam = tf.transform
			if (tf.header.frame_id == '/openni_rgb_frame') and\
					(tf.child_frame_id == '/openni_rgb_optical_frame'):
				print "Got optical2rgb"
				#print tf
				tf_optical2rgb = tf.transform
		if tf_rgb2cam and tf_optical2rgb:
			break
	else:
		print "Didn't find all the needed openni transforms"
		return
	M_rgb2cam = mat_from_transform(tf_rgb2cam)
	M_optical2rgb = mat_from_transform(tf_optical2rgb)
	M_optical2cam = numpy.dot(M_rgb2cam, M_optical2rgb)	#TODO check
	return M_optical2cam


def get_calibration_from_string(calibstring):
	vtr = calibstring.split()[:3]
	vrot = calibstring.split()[3:]
	return tf.TransformerROS().fromTranslationRotation(vtr, vrot)


def get_calibration_from_bag(bagname, oldobject):
	print 'Opening calibration bag: "%s"'%bagname
	calbag = rosbag.Bag(bagname, 'r')
	for topic, msg, t in calbag.read_messages(topics=['/tf']):
		for tf in msg.transforms:
			if (tf.header.frame_id == oldobject):
				print "Found calibration."
				return mat_from_transform(tf.transform), tf.child_frame_id
	print "Couldn't find suitable transform."
	return


if __name__=='__main__':
	main()
