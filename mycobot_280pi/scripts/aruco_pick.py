#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import rospy
import Tkinter as tk
import numpy as np
import cv2
import math
from mycobot_control.srv import *
from opencv_aruco.msg import *


class Window:
    
    def __init__(self, handle):
        # Initialise class variables
        self.win = handle
        self.win.resizable(0, 0)

        self.speed = rospy.get_param("~speed", 20)

        # Gripper side
        self.joint6_flange = [0, 0, 0, 0, 0, 0]
        self.joint2marker = [0, -15, 40, 90, 0, 0]
        self.gripper_marker = [0, 0, 0, 0, 0, 0]
        self.joint2gripper = [0, 0, 100, 0, 0, 0]
        self.gripper_end = [0, 0, 0, 0, 0, 0] 

        # Camera
        self.camera = [0, 0, 0, 0, 0, 0]

        # Object side
        self.object_marker = [0, 0, 0, 0, 0, 0]
        self.marker2object = [0, 0, -20, 0, 0, 0]
        self.object_center = [0, 0, 0, 0, 0, 0]

        # Initialise ROS related
        rospy.init_node("aruco_pick", anonymous=True, disable_signals=True)
        self.connect_ser()

        # Initialise TK window
        self.ws = self.win.winfo_screenwidth()
        self.hs = self.win.winfo_screenheight()
        x = (self.ws / 2) - 450
        y = (self.hs / 2) - 150
        self.win.geometry("900x300+{}+{}".format(x, y))
        self.set_layout()
        self.update()
        print("Success")

    
    def connect_ser(self):
        rospy.wait_for_message("gripper_pose", Pose)
        rospy.wait_for_message("object_pose", Pose)
        rospy.wait_for_service("get_joint_coords")
        rospy.wait_for_service("set_joint_coords")
        rospy.wait_for_service("switch_gripper_status")
        rospy.wait_for_service("switch_servo_status")
        try:
            self.get_coords = rospy.ServiceProxy("get_joint_coords", GetCoords)
            self.set_coords = rospy.ServiceProxy("set_joint_coords", SetCoords)
            self.switch_gripper = rospy.ServiceProxy(
                "switch_gripper_status", GripperStatus
            )
            self.switch_servo = rospy.ServiceProxy(
                "switch_servo_status", ServoStatus
            )
        except:
            print("start error ...")
            exit(1)
        
        print("Connect service success.")
    

    def set_layout(self):

        self.frm_info = tk.Frame(width=900, height=250)
        self.frm_info.grid(row=0, column=0, padx=0, pady=0)
        self.set_info()

        self.frm_ctl= tk.Frame(width=250, height=50)
        self.frm_ctl.grid(row=1, column=0, padx=0, pady=0)
        self.set_ctl()


    def set_info(self):
        for i in range(1, 9):
            self.frm_info.columnconfigure(i, weight=1, uniform="group1")

        tk.Label(self.frm_info, text="x").grid(row=1, column=0)
        tk.Label(self.frm_info, text="y").grid(row=2, column=0)
        tk.Label(self.frm_info, text="z").grid(row=3, column=0)
        tk.Label(self.frm_info, text="rx").grid(row=4, column=0)
        tk.Label(self.frm_info, text="ry").grid(row=5, column=0)
        tk.Label(self.frm_info, text="rz").grid(row=6, column=0)

        tk.Label(self.frm_info, text="J6_Flange").grid(row=0, column=1)
        tk.Label(self.frm_info, text="Marker_Offset").grid(row=0, column=2)
        tk.Label(self.frm_info, text="Marker").grid(row=0, column=3)
        tk.Label(self.frm_info, text="Gripper_Offset").grid(row=0, column=4)
        # tk.Label(self.frm_info, text="Gripper_End").grid(row=0, column=5)
        tk.Label(self.frm_info, text="Camera").grid(row=0, column=5)
        tk.Label(self.frm_info, text="Marker").grid(row=0, column=6)
        tk.Label(self.frm_info, text="Marker_Offset").grid(row=0, column=7)
        tk.Label(self.frm_info, text="Object").grid(row=0, column=8)

        self.joint6_flange_var = []
        for i in range(6):
            var = tk.StringVar()
            var.set(0)
            tk.Label(self.frm_info, textvariable=var).grid(row=(i+1), column=1)
            self.joint6_flange_var.append(var)

        self.joint2marker_var = []
        for i in range(6):
            var = tk.StringVar()
            var.set(self.joint2marker[i])
            tk.Entry(self.frm_info, textvariable=var, width=10).grid(row=(i+1), column=2)
            self.joint2marker_var.append(var)

        self.gripper_marker_var = []
        for i in range(6):
            var = tk.StringVar()
            var.set(0)
            tk.Label(self.frm_info, textvariable=var).grid(row=(i+1), column=3)
            self.gripper_marker_var.append(var)

        self.joint2gripper_var = []
        for i in range(6):
            var = tk.StringVar()
            var.set(self.joint2gripper[i])
            tk.Entry(self.frm_info, textvariable=var, width=10).grid(row=(i+1), column=4)
            self.joint2gripper_var.append(var)

        self.camera_var = []
        for i in range(6):
            var = tk.StringVar()
            var.set(0)
            tk.Label(self.frm_info, textvariable=var).grid(row=(i+1), column=5)
            self.camera_var.append(var)

        self.object_marker_var = []
        for i in range(6):
            var = tk.StringVar()
            var.set(0)
            tk.Label(self.frm_info, textvariable=var).grid(row=(i+1), column=6)
            self.object_marker_var.append(var)

        self.marker2object_var = []
        for i in range(6):
            var = tk.StringVar()
            var.set(self.marker2object[i])
            tk.Entry(self.frm_info, textvariable=var, width=10).grid(row=(i+1), column=7)
            self.marker2object_var.append(var)

        self.object_center_var = []
        for i in range(6):
            var = tk.StringVar()
            var.set(0)
            tk.Label(self.frm_info, textvariable=var).grid(row=(i+1), column=8)
            self.object_center_var.append(var)
    
    
    def set_ctl(self):
        tk.Button(self.frm_ctl, text="Release to Move", width=20, command=self.release_all_servos)\
            .grid(row=0, column=0, pady=5, padx=40)
        tk.Button(self.frm_ctl, text="Hold Position", width=20, command=self.focus_all_servos)\
            .grid(row=0, column=1, pady=5, padx=40)

        tk.Button(self.frm_ctl, text="Update Info", width=20, command=self.update)\
            .grid(row=0, column=2, pady=5, padx=5)
        tk.Button(self.frm_ctl, text="Start", width=20, command=self.execute)\
            .grid(row=0, column=3, pady=5, padx=5)

    
    def update(self):
        # get robot coords
        t = time.time()
        while time.time() - t < 2:
            coords = self.get_coords()
            if coords.x != 0:
                break
            time.sleep(0.1)
        
        self.joint6_flange = [
            coords.x,
            coords.y,
            coords.z,
            coords.rx,
            coords.ry,
            coords.rz,
        ]

        # get gripper marker pose
        pose = rospy.wait_for_message("gripper_pose", Pose)
        print(pose)
        self.g_pose = [
            pose.x * 1000, # meter to milimeter
            pose.y * 1000,
            pose.z * 1000,
            pose.rx,
            pose.ry,
            pose.rz
        ]

        # get gripper marker pose
        pose = rospy.wait_for_message("object_pose", Pose)
        self.o_pose = [
            pose.x * 1000,
            pose.y * 1000,
            pose.z * 1000,
            pose.rx,
            pose.ry,
            pose.rz
        ]

        for i in range(6):
            self.joint2marker[i] = float(self.joint2marker_var[i].get())
            self.joint2gripper[i] = float(self.joint2gripper_var[i].get())
            self.marker2object[i] = float(self.marker2object_var[i].get())
        
        self.calculate_coords()

        for i in range(6):
            self.joint6_flange_var[i].set("{:.2f}".format(self.joint6_flange[i]))
            self.gripper_marker_var[i].set("{:.2f}".format(self.gripper_marker[i]))
            # self.gripper_end_var[i].set("{:.2f}".format(self.gripper_end[i]))
            self.camera_var[i].set("{:.2f}".format(self.camera[i]))
            self.object_marker_var[i].set("{:.2f}".format(self.object_marker[i]))
            self.object_center_var[i].set("{:.2f}".format(self.object_center[i]))


    def calculate_coords(self):
        base2joint_R = np.zeros((3, 3))
        cv2.Rodrigues(np.radians(self.joint6_flange[3:]), base2joint_R)

        offset = np.matmul(base2joint_R, self.joint2marker[:3])
        self.gripper_marker = [
            self.joint6_flange[0] + offset[0],
            self.joint6_flange[1] + offset[1],
            self.joint6_flange[2] + offset[2],
            self.joint6_flange[3],
            self.joint6_flange[4],
            self.joint6_flange[5],
        ]

        if self.g_pose[0] == 0:
            return

        joint2marker_R = np.zeros((3, 3))
        cv2.Rodrigues(np.radians(self.joint2marker[3:]), joint2marker_R)

        base2marker_R = np.matmul(base2joint_R, joint2marker_R)
        offset = np.matmul(base2marker_R, self.g_pose[:3])
        self.camera = [
            self.gripper_marker[0] + offset[0], 
            self.gripper_marker[1] + offset[1],
            self.gripper_marker[2] + offset[2],
            self.gripper_marker[3],
            self.gripper_marker[4],
            self.gripper_marker[5],
        ]
        
        if self.o_pose[0] == 0:
            return

        marker2cam_R = np.zeros((3, 3))
        cv2.Rodrigues(np.array(self.g_pose[3:]), marker2cam_R)

        base2cam_R = np.matmul(base2marker_R, marker2cam_R)
        offset = np.matmul(base2cam_R, self.o_pose[:3])
        self.object_marker = [
            self.camera[0] + offset[0],
            self.camera[1] + offset[1],
            self.camera[2] + offset[2],
            self.camera[3],
            self.camera[4],
            self.camera[5],
        ]

        cam2omarker_R = np.zeros((3, 3))
        cv2.Rodrigues(np.array(self.o_pose[3:]), cam2omarker_R)
        cam2omarker_R = np.transpose(cam2omarker_R)

        base2omarker_R = np.matmul(base2cam_R, cam2omarker_R)
        offset = np.matmul(base2omarker_R, self.marker2object[:3])
        self.object_center = [
            self.object_marker[0] + offset[0],
            self.object_marker[1] + offset[1],
            self.object_marker[2] + offset[2],
            self.object_marker[3],
            self.object_marker[4],
            self.object_marker[5],
        ]


    def execute(self):
        dest = (
            self.object_center[0] + self.joint2gripper[0],
            self.object_center[1] + self.joint2gripper[1],
            self.object_center[2] + self.joint2gripper[2],
            self.object_center[3],
            self.object_center[4],
            self.object_center[5],
            self.speed,
            0
        )
        rospy.loginfo(dest)
        self.set_coords(*dest)
    

    def release_all_servos(self):
        self.switch_servo(False)


    def focus_all_servos(self):
        self.switch_servo(True)
    

    def run(self):
        while True:
            try:
                self.win.update()
                time.sleep(0.001)
            except tk.TclError as e:
                if "application has been destroyed" in str(e):
                    break
                else:
                    raise


def main():
    win = tk.Tk()
    win.title("MyCobot Aruco Pick Example")
    Window(win).run()


if __name__ == "__main__":
    main()
