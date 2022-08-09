#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import rospy
import Tkinter as tk
from mycobot_control.srv import *


class Window:
    
    def __init__(self, handle):
        # Initialise class variables
        self.win = handle
        self.win.resizable(0, 0)

        self.speed = rospy.get_param("~speed", 20)

        self.pick_coords = [0, 0, 0, 0, 0, 0, self.speed]
        self.pick_joint_angles = [0, 0, 0, 0, 0, 0, self.speed]

        self.place_coords = [0, 0, 0, 0, 0, 0, self.speed]
        self.place_joint_angles = [0, 0, 0, 0, 0, 0, self.speed]

        # Initialise ROS related
        rospy.init_node("pick_and_place", anonymous=True, disable_signals=True)
        self.connect_ser()

        # Initialise TK window
        self.ws = self.win.winfo_screenwidth()
        self.hs = self.win.winfo_screenheight()
        x = (self.ws / 2) - 300 
        y = (self.hs / 2) - 250
        self.win.geometry("750x400+{}+{}".format(x, y))
        self.set_layout()

    
    def connect_ser(self):
        rospy.wait_for_service("get_joint_angles")
        rospy.wait_for_service("set_joint_angles")
        rospy.wait_for_service("get_joint_coords")
        rospy.wait_for_service("switch_gripper_status")
        rospy.wait_for_service("switch_servo_status")
        try:
            self.get_coords = rospy.ServiceProxy("get_joint_coords", GetCoords)
            self.get_angles = rospy.ServiceProxy("get_joint_angles", GetAngles)
            self.set_angles = rospy.ServiceProxy("set_joint_angles", SetAngles)
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
    

    def get_data(self):
        t = time.time()
        while time.time() - t < 2:
            coords_data = self.get_coords()
            if coords_data.x > 1:
                break
            time.sleep(0.1)

        t = time.time()
        while time.time() - t < 2:
            angles_data = self.get_angles()
            if angles_data.joint_1 > 1:
                break
            time.sleep(0.1)

        coords = [
            round(coords_data.x, 2),
            round(coords_data.y, 2),
            round(coords_data.z, 2),
            round(coords_data.rx, 2),
            round(coords_data.ry, 2),
            round(coords_data.rz, 2),
            self.speed,
        ]
        angles = [
            round(angles_data.joint_1, 2),
            round(angles_data.joint_2, 2),
            round(angles_data.joint_3, 2),
            round(angles_data.joint_4, 2),
            round(angles_data.joint_5, 2),
            round(angles_data.joint_6, 2),
            self.speed,
        ]

        return coords, angles
    

    def set_layout(self):
        self.frm_pick = tk.Frame(width=250, height=200)
        self.frm_pick.grid(row=0, column=0, padx=1, pady=3)
        self.set_pick()

        self.frm_ctl= tk.Frame(width=250, height=200)
        self.frm_ctl.grid(row=0, column=1, padx=1, pady=3)
        self.set_ctl()

        self.frm_place = tk.Frame(width=250, height=200)
        self.frm_place.grid(row=0, column=2, padx=1, pady=3)
        self.set_place()


    def set_pick(self):
        # Title
        tk.Label(self.frm_pick, text="Picking-Up").grid(row=0, column=0, columnspan=4, pady=15)

        # Joint Angles
        tk.Label(self.frm_pick, text="angles").grid(row=1, column=0, columnspan=2, pady=(5, 10), padx=30)
        tk.Label(self.frm_pick, text="Joint 1 ").grid(row=2, column=0)
        tk.Label(self.frm_pick, text="Joint 2 ").grid(row=3, column=0)
        tk.Label(self.frm_pick, text="Joint 3 ").grid(row=4, column=0)
        tk.Label(self.frm_pick, text="Joint 4 ").grid(row=5, column=0)
        tk.Label(self.frm_pick, text="Joint 5 ").grid(row=6, column=0)
        tk.Label(self.frm_pick, text="Joint 6 ").grid(row=7, column=0)

        # Joint Angles Values
        self.pick_joint_vars = []
        for i in range(6):
            joint_var = tk.StringVar()
            joint_var.set(str(self.pick_joint_angles[i]))
            tk.Label(self.frm_pick, textvariable=joint_var)\
                .grid(row=(i+2), column=1)
            self.pick_joint_vars.append(joint_var)

        # Coordinates
        tk.Label(self.frm_pick, text="coords").grid(row=1, column=2, columnspan=2, pady=(5, 10), padx=30)
        tk.Label(self.frm_pick, text="    x   ").grid(row=2, column=2)
        tk.Label(self.frm_pick, text="    y   ").grid(row=3, column=2)
        tk.Label(self.frm_pick, text="    z   ").grid(row=4, column=2)
        tk.Label(self.frm_pick, text="   rx   ").grid(row=5, column=2)
        tk.Label(self.frm_pick, text="   ry   ").grid(row=6, column=2)
        tk.Label(self.frm_pick, text="   rz   ").grid(row=7, column=2)

        # Joint Angles Values
        self.pick_coord_vars = []
        for i in range(6):
            coord_var = tk.StringVar()
            coord_var.set(str(self.pick_joint_angles[i]))
            tk.Label(self.frm_pick, textvariable=coord_var)\
                .grid(row=(i+2), column=3)
            self.pick_coord_vars.append(coord_var)
        
        tk.Button(self.frm_pick, text="Update Position", width=15, command=self.update_pick)\
            .grid(row=8, column=0, columnspan=4, pady=20)
    

    def update_pick(self):
        self.pick_coords, self.pick_joint_angles = self.get_data()

        for i in range(6):
            self.pick_coord_vars[i].set(str(self.pick_coords[i]))
            self.pick_joint_vars[i].set(str(self.pick_joint_angles[i]))
    
    
    def set_ctl(self):
        tk.Label(self.frm_ctl, text="Position Configure").grid(row=0, column=0, pady=5)
        tk.Button(self.frm_ctl, text="Release to Move", width=20, command=self.release_all_servos)\
            .grid(row=1, column=0, pady=5, padx=40)
        tk.Button(self.frm_ctl, text="Hold Position", width=20, command=self.focus_all_servos)\
            .grid(row=2, column=0, pady=5, padx=40)
        
        tk.Label(self.frm_ctl, text="Execution").grid(row=3, column=0, pady=(25, 5))
        tk.Button(self.frm_ctl, text="Start", width=10, command=self.execute)\
            .grid(row=4, column=0, pady=5, padx=5)


    def execute(self):
        # Return to start point
        self.set_angles(0, 0, 0, 0, 0, 0, self.speed)
        time.sleep(5)
        
        # open gripper and move to start point
        self.switch_gripper(True)
        self.set_angles(*self.pick_joint_angles)
        time.sleep(5)
        
        # pick it up
        self.switch_gripper(False)
        time.sleep(1)

        # return to origin
        self.set_angles(0, 0, 0, 0, 0, 0, self.speed)
        time.sleep(5)

        # move to desination
        self.set_angles(*self.place_joint_angles)
        time.sleep(5)

        # place it down
        self.switch_gripper(True)
        time.sleep(5)

        # Return to start point
        self.set_angles(0, 0, 0, 0, 0, 0, self.speed)
        time.sleep(5)
    

    def release_all_servos(self):
        self.switch_gripper(True)
        self.switch_servo(False)


    def focus_all_servos(self):
        self.switch_servo(True)
    

    def set_place(self):
        # Title
        tk.Label(self.frm_place, text="Placing").grid(row=0, column=0, columnspan=4, pady=15)

        # Joint Angles
        tk.Label(self.frm_place, text="angles").grid(row=1, column=0, columnspan=2, pady=(5, 10), padx=30)
        tk.Label(self.frm_place, text="Joint 1 ").grid(row=2, column=0)
        tk.Label(self.frm_place, text="Joint 2 ").grid(row=3, column=0)
        tk.Label(self.frm_place, text="Joint 3 ").grid(row=4, column=0)
        tk.Label(self.frm_place, text="Joint 4 ").grid(row=5, column=0)
        tk.Label(self.frm_place, text="Joint 5 ").grid(row=6, column=0)
        tk.Label(self.frm_place, text="Joint 6 ").grid(row=7, column=0)

        # Joint Angles Values
        self.place_joint_vars = []
        for i in range(6):
            joint_var = tk.StringVar()
            joint_var.set(str(self.place_joint_angles[i]))
            tk.Label(self.frm_place, textvariable=joint_var)\
                .grid(row=(i+2), column=1)
            self.place_joint_vars.append(joint_var)

        # Coordinates
        tk.Label(self.frm_place, text="coords").grid(row=1, column=2, columnspan=2, pady=(5, 10), padx=30)
        tk.Label(self.frm_place, text="    x   ").grid(row=2, column=2)
        tk.Label(self.frm_place, text="    y   ").grid(row=3, column=2)
        tk.Label(self.frm_place, text="    z   ").grid(row=4, column=2)
        tk.Label(self.frm_place, text="   rx   ").grid(row=5, column=2)
        tk.Label(self.frm_place, text="   ry   ").grid(row=6, column=2)
        tk.Label(self.frm_place, text="   rz   ").grid(row=7, column=2)

        # Joint Angles Values
        self.place_coord_vars = []
        for i in range(6):
            coord_var = tk.StringVar()
            coord_var.set(str(self.place_joint_angles[i]))
            tk.Label(self.frm_place, textvariable=coord_var)\
                .grid(row=(i+2), column=3)
            self.place_coord_vars.append(coord_var)

        tk.Button(self.frm_place, text="Update Position", width=15, command=self.update_place)\
            .grid(row=8, column=0, columnspan=4, pady=20)
    

    def update_place(self):
        self.place_coords, self.place_joint_angles = self.get_data()

        for i in range(6):
            self.place_coord_vars[i].set(str(self.place_coords[i]))
            self.place_joint_vars[i].set(str(self.place_joint_angles[i]))


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
    win.title("MyCobot Pick and Place")
    Window(win).run()


if __name__ == "__main__":
    main()