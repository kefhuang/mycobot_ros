import numpy as np
import cv2
import sys
import argparse
import time
import rospy
import math
from opencv_aruco.msg import *


marker_length = -1 
aruco_dict_type = None
matrix_coefficients = None
distortion_coefficients = None


def pose_esitmation(frame, id):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, _ = cv2.aruco.detectMarkers(
        gray, 
        aruco_dict,
        parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients
    )

    if ids is None or not id in ids:
        return None, None
    
    corner = corners[ids.tolist().index([id])]
    # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
        corner, 
        marker_length, 
        matrix_coefficients,
        distortion_coefficients
    )
    
    marker_image = cv2.aruco.drawDetectedMarkers(frame, corners)
    marker_image = cv2.aruco.drawAxis(marker_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.03)
    cv2.imshow("Detected Marker", marker_image)
    cv2.waitKey(1)
    
    return rvec, tvec


def initialise_node():
    rospy.init_node("pose_estimation")
    rospy.loginfo("start...")

    gripper_pub = rospy.Publisher("gripper_pose", Pose)
    object_pub = rospy.Publisher("object_pose", Pose)

    gripper_id = rospy.get_param("~gripper_id")
    object_id = rospy.get_param("~object_id")

    global marker_length 
    marker_length = rospy.get_param("~marker_length")
    global aruco_dict_type 
    aruco_dict_type = getattr(cv2.aruco, rospy.get_param("~aruco_dict_type"))
    global matrix_coefficients 
    matrix_coefficients = np.load(rospy.get_param("~matrix_coefficients"))
    global distortion_coefficients 
    distortion_coefficients = np.load(rospy.get_param("~distortion_coefficients"))

    rospy.loginfo("ready")
    video = cv2.VideoCapture(0)
    g_pub = [0] * 6
    o_pub = [0] * 6
    while not rospy.is_shutdown():
        _, frame = video.read()
        g_rvec, g_tvec = pose_esitmation(frame, gripper_id)
        o_rvec, o_tvec = pose_esitmation(frame, object_id)

        if g_rvec is not None:
            g_pub = *g_tvec.reshape(3), *g_rvec.reshape(3)
        
        if o_rvec is not None:
            o_pub = *o_tvec.reshape(3), *o_rvec.reshape(3)

        gripper_pub.publish(*g_pub)
        object_pub.publish(*o_pub)


if __name__ == '__main__':
    initialise_node()
