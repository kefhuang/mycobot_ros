#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import os
import argparse


def generate(total_number, temp_path):
    camera = cv2.VideoCapture(0)

    count = 0
    while count <= total_number:
        path = os.path.join(temp_path, f"calib{count:03}.jpg")
        ret, img = camera.read()
        cv2.imshow(f"img_{count:03}", img)

        if cv2.waitKey(0) & 0xFF == ord('s'):
            cv2.imwrite(path, img)
            count += 1

        cv2.destroyWindow(f"img_{count:03}")


def calibrate(dirpath, square_size, width, height, visualize=False):
    # termination criteria
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 
        30, 
        0.001
    )

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(h,w,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = os.listdir(dirpath)
    for fname in images:
        img = cv2.imread(os.path.join(dirpath, fname))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)

        if visualize:
            cv2.imshow(f'img({fname})',img)
            cv2.waitKey(0)
            cv2.destroyWindow(f'img({fname})')

    _, mtx, dist, _, _= cv2.calibrateCamera(
        objpoints, 
        imgpoints, 
        gray.shape[::-1], 
        None, None
    )

    return mtx, dist


def clean(temp_path):
    images = os.listdir(temp_path)

    for fname in images:
        path = os.path.join(temp_path, fname)
        os.remove(path)


def main():
    rospy.init_node('aruco_calibrate')
    rospy.loginfo("start...")

    total_number = rospy.get_param("~total_number")
    temp_path = rospy.get_param("~temp_path")
    square_size = rospy.get_param("~square_size")
    width = rospy.get_param("~width")
    height = rospy.get_param("~height")

    visualize = rospy.get_param("~visualize")
    output_path = rospy.get_param("~output_path")

    generate(total_number, temp_path)

    mtx, dist = calibrate(
        temp_path,
        square_size, 
        visualize=visualize, 
        width=width, 
        height=height
    )

    np.save(os.path.join(output_path, "calibration_matrix"), mtx)
    np.save(os.path.join(output_path, "distortion_coefficients"), dist)

    clean(temp_path)


if __name__ == '__main__':
    main()