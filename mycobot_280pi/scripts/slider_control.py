#!/usr/bin/env python2
# encoding=utf-8

"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyAMA0'
    baud: serial prot baudrate. Defaults is 1000000.
"""

import math
import rospy
from rospy import ServiceException
from sensor_msgs.msg import JointState
from mycobot_control.srv import SetAngles

from pymycobot.mycobot import MyCobot


set_angles = None


def connect_ser():
    global set_angles

    rospy.wait_for_service("set_joint_angles")
    try:
        set_angles = rospy.ServiceProxy("set_joint_angles", SetAngles)
    except:
        print("start error ...")
        exit(1)

    print("Connect service success.")


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    print(data.position)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(math.degrees(value))

    data_list.append(50)

    try:
        print(data_list)
        set_angles(*data_list)
    except ServiceException:
        pass


def run():
    rospy.init_node("control_slider", anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)
    connect_ser()

    # spin() simply keeps python from exiting until this node is stopped
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    run()
