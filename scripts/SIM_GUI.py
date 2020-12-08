#!/usr/bin/env python
# QuadCopter GUI
# For Simulation, Monitoring, and Command/Control
# Author: Brendan Martin (C) 2020
#         
###################################################

###################################################
# Included Libraries
###################################################
import csv
import sys
import os
import json
import time
import datetime
import math
from Tkinter import *
import tkMessageBox
import tkFileDialog
import ttk
###################################################
# 3rd Party Libraries
###################################################
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.axes3d import Axes3D
from PIL import Image, ImageTk
import rospy
###################################################
# Author Libraries
###################################################
from picopter.msg import Sim_msg

class App():

    def __init__(self):
        # Create Master window
        self.master = Tk()
        self.master.title("PiCopter Monitor")
        self.master.geometry("{0}x{1}+0+0".format(
            self.master.winfo_screenwidth()-3, self.master.winfo_screenheight()-3))
        
        # Create the toolbar
        self.Create_Menu()

        # Create side data Frame & Listbox
        self.Create_SIM_Tree_View()

        # Create the default plotter view
        self.Plotter_View_Default()

        # used for internal timing routings
        self.time_now_tree = int(round(time.time() * 1000))
        self.time_last_tree = int(round(time.time() * 1000))
        self.time_now_plot = int(round(time.time() * 1000))
        self.time_last_plot = int(round(time.time() * 1000))

        # Start the ROS listener thread
        self.SIM_Subscriber()

        self.master.mainloop()

    def Create_Menu(self):
        self.menubar = Menu(self.master)
        self.master.config(menu=self.menubar)

        # File Menu
        self.fileMenu = Menu(self.menubar)
        self.fileMenu.add_command(label="Exit", command=self.master.destroy)
        self.menubar.add_cascade(label="File", menu=self.fileMenu)

        # View Menu
        self.viewMenu = Menu(self.menubar)
        self.sim_data_tree_enabled = BooleanVar()
        self.sim_data_tree_enabled.set(True)
        self.viewMenu.add_checkbutton(label="Show Sim Data Tree", onvalue=True, offvalue=False, variable=self.sim_data_tree_enabled, command=self.Toggle_Sim_Data_Tree)
        self.default_plotter_enabled = BooleanVar()
        self.default_plotter_enabled.set(True)
        self.viewMenu.add_checkbutton(label="Show Default Plotter", onvalue=True, offvalue=False, variable=self.default_plotter_enabled, command=self.Toggle_Default_Plot)
        self.menubar.add_cascade(label="View", menu=self.viewMenu)

    def Create_SIM_Tree_View(self):
        # Create side data Frame & Listbox
        self.sim_data_frame = Frame(self.master)
        self.sim_data_frame.grid(row=1, column=0, columnspan=1, sticky=W+E+N+S)
        self.master.rowconfigure(1, weight=1)
        self.master.columnconfigure(1, weight=1)
        self.SLB = Label(self.sim_data_frame, text="Simulation Data")   # SLB = Simulation ListBox
        self.SLB.pack()
        self.SSB = Scrollbar(self.sim_data_frame, orient="vertical")   # SSB + Simulation ScrollBar
        self.SSB.pack(side=RIGHT, fill=Y)
        self.STree = ttk.Treeview(self.sim_data_frame, selectmode="browse")
        self.STree.configure(yscrollcommand=self.SSB.set)
        self.STree["columns"] = ("1", "2", "3")
        self.STree.column("1", width=100, anchor='w')
        self.STree.column("2", width=100, anchor='e')
        self.STree.column("3", width=100, anchor='e')
        self.STree.heading("0", text="Group")
        self.STree.heading("1", text="Parameter")
        self.STree.heading("2", text="Value")
        self.STree.heading("3", text="Units")
        self.STree.insert('', 'end', 'attitude', text="Attitude", open=False)
        self.STree.insert('', 'end', 'body accelerations', text="Body Accelerations", open=False)
        self.STree.insert('', 'end', 'body velocities', text="Body Velocities", open=False)
        self.STree.insert('', 'end', 'motor forces', text="Motor Forces", open=False)
        self.STree.insert('', 'end', 'body forces', text="Body Forces", open=False)
        self.STree.insert('', 'end', 'body moments', text="Body Moments", open=False)
        self.STree.insert('attitude', 'end', 'roll', values=("roll", str(0.0000), "deg"))
        self.STree.insert('attitude', 'end', 'pitch', values=("pitch", str(0.0000), "deg"))
        self.STree.insert('attitude', 'end', 'yaw', values=("yaw", str(0.0000), "deg"))
        self.STree.insert('body accelerations', 'end', 'u_dot', values=("u_dot", str(0.0000), "m/s/s"))
        self.STree.insert('body accelerations', 'end', 'v_dot', values=("v_dot", str(0.0000), "m/s/s/"))
        self.STree.insert('body accelerations', 'end', 'w_dot', values=("w_dot", str(0.0000), "m/s/s/"))
        self.STree.insert('body accelerations', 'end', 'p_dot', values=("p_dot", str(0.0000), "deg/s/s/"))
        self.STree.insert('body accelerations', 'end', 'q_dot', values=("q_dot", str(0.0000), "deg/s/s/"))
        self.STree.insert('body accelerations', 'end', 'r_dot', values=("r_dot", str(0.0000), "deg/s/s/"))
        self.STree.insert('body velocities', 'end', 'u', values=("u", str(0.0000), "m/s"))
        self.STree.insert('body velocities', 'end', 'v', values=("v", str(0.0000), "m/s"))
        self.STree.insert('body velocities', 'end', 'w', values=("w", str(0.0000), "m/s"))
        self.STree.insert('body velocities', 'end', 'p', values=("p", str(0.0000), "deg/s"))
        self.STree.insert('body velocities', 'end', 'q', values=("q", str(0.0000), "deg/s"))
        self.STree.insert('body velocities', 'end', 'r', values=("r", str(0.0000), "deg/s"))
        self.STree.insert('motor forces', 'end', 'M1', values=("Motor 1", str(0.0000), "N"))
        self.STree.insert('motor forces', 'end', 'M2', values=("Motor 2", str(0.0000), "N"))
        self.STree.insert('motor forces', 'end', 'M3', values=("Motor 3", str(0.0000), "N"))
        self.STree.insert('motor forces', 'end', 'M4', values=("Motor 4", str(0.0000), "N"))
        self.STree.insert('body forces', 'end', 'X_Force', values=("X", str(0.0000), "N"))
        self.STree.insert('body forces', 'end', 'Y_Force', values=("Y", str(0.0000), "N"))
        self.STree.insert('body forces', 'end', 'Z_Force', values=("Z", str(0.0000), "N"))
        self.STree.insert('body moments', 'end', 'K_Moment', values=("K", str(0.0000), "N-m"))
        self.STree.insert('body moments', 'end', 'M_Moment', values=("M", str(0.0000), "N-m"))
        self.STree.insert('body moments', 'end', 'N_Moment', values=("N", str(0.0000), "N-m"))

        self.STree.pack(expand=True, fill=Y)
        self.SSB.config(command=self.STree.yview)
        self.STree.update_idletasks()

    def Plotter_View_Default(self):
        self.default_figure = Figure(figsize=(5,5), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.default_figure, master=self.master)
        self.default_plot_1 = self.default_figure.add_subplot(221)
        self.default_plot_2 = self.default_figure.add_subplot(222, projection='3d')
        self.default_plot_3 = self.default_figure.add_subplot(223)
        self.default_plot_4 = self.default_figure.add_subplot(224)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=1, columnspan=1, rowspan=6, stick=W+E+N+S)
        self.default_plot_frame = Frame(self.master)
        self.default_plot_frame.grid(row=2, column=1, columnspan=1)
        self.pbar = NavigationToolbar2Tk(self.canvas, self.default_plot_frame)

        ## Initialized default plotting values
        self.pitch_array = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.roll_array = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.yaw_array = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.yaw_array = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.elevation_array = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.altitude_array = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.northings_array = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.eastings_array = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.time_array = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])


    def update_default_plot(self):

        # First shift all the array data to the left by one index
        for i in range(9):
            self.pitch_array[i] = self.pitch_array[i+1]
            self.roll_array[i] = self.roll_array[i+1]
            self.yaw_array[i] = self.yaw_array[i+1]
            self.elevation_array[i] = self.elevation_array[i+1]
            self.altitude_array[i] = self.altitude_array[i+1]
            self.northings_array[i] = self.altitude_array[i+1]
            self.eastings_array[i] = self.altitude_array[i+1]
            self.time_array[i] = self.time_array[i+1]
        # populate the last index with the latest value
        self.pitch_array[9] = self.pitch
        self.roll_array[9] = self.roll
        self.yaw_array[9] = self.yaw
        self.elevation_array[9] = self.elevation
        self.altitude_array[9] = self.altitude
        self.northings_array[9] = self.northings
        self.eastings_array[9] = self.eastings
        self.time_array[9] = self.sim_time
        # self.pitch_array[9] = 0.0
        # self.roll_array[9] = 0.0
        # self.yaw_array[9] = 0.0
        # self.elevation_array[9] = 0.0
        # self.altitude_array[9] = 0.0
        # self.northings_array[9] = 0.0
        # self.eastings_array[9] = 0.0
        # self.time_array[9] = self.sim_time

        # Clear the plots
        self.default_plot_1.clear()
        self.default_plot_2.clear()
        self.default_plot_3.clear()
        self.default_plot_4.clear()

        # Plot the new data
        self.default_plot_1.plot(self.time_array, self.roll_array, label="Roll")
        self.default_plot_1.plot(self.time_array, self.pitch_array, label="Pitch")
        self.default_plot_1.legend()
        self.default_plot_1.grid()
        self.default_plot_1.set_xlim(round(self.time_array[0]-1), round(self.time_array[9])+1)
        self.default_plot_1.set_xlabel('Sim Time (sec)')
        self.default_plot_1.set_ylabel('Attitude (deg)')
        self.default_plot_2.plot(self.eastings_array, self.northings_array, self.elevation_array, label="3D Track")
        self.default_plot_2.legend()
        self.default_plot_2.grid()
        self.default_plot_2.set_xlabel('Eastings (m)')
        self.default_plot_2.set_ylabel('Northings (m)')
        self.default_plot_2.set_zlabel('Elevation (m)')
        self.default_plot_3.plot(self.time_array, self.yaw_array, label="Yaw")
        self.default_plot_3.legend()
        self.default_plot_3.grid()
        self.default_plot_3.set_xlabel('Sim Time (sec)')
        self.default_plot_3.set_ylabel('Yaw (deg)')
        self.default_plot_4.plot(self.time_array, self.elevation_array, label="Elevation")
        self.default_plot_4.legend()
        self.default_plot_4.grid()
        self.default_plot_4.set_xlabel('Sim Time (sec)')
        self.default_plot_4.set_ylabel('Elevation (m)')
        self.canvas.draw()


    def SIM_Subscriber(self):
        # Start a ROS subscriber that listens to the "sim_data" topic and updates
        # the GUI with simulation data
        rospy.init_node("sim_gui", anonymous=True)
        self.sub = rospy.Subscriber("sim_data", Sim_msg, self.Update_SIM_Data)

    def Update_SIM_Data(self, data):
        ## This function is a ROS subscriber that updates values from the picopter sim_data topic
        #  and then populates the Simulation List Box with updated values
        self.Unpack_SIM_Msg(data)

        # Only update tree view with data twice a second and only if the sim data view is enabled
        self.time_now_tree = int(round(time.time() * 1000))
        if ((self.time_now_tree - self.time_last_tree) >= 500) and self.sim_data_tree_enabled.get():
            self.time_last_tree = self.time_now_tree
            self.STree.set('roll', '2', str(round(self.roll, 4)))
            self.STree.set('pitch', '2', str(round(self.pitch, 4)))
            self.STree.set('yaw', '2', str(round(self.yaw, 4)))
            self.STree.set('u_dot', '2', str(round(self.u_dot, 4)))
            self.STree.set('v_dot', '2', str(round(self.v_dot, 4)))
            self.STree.set('w_dot', '2', str(round(self.w_dot, 4)))
            self.STree.set('p_dot', '2', str(round(self.p_dot, 4)))
            self.STree.set('q_dot', '2', str(round(self.q_dot, 4)))
            self.STree.set('r_dot', '2', str(round(self.r_dot, 4)))
            self.STree.set('u', '2', str(round(self.u, 4)))
            self.STree.set('v', '2', str(round(self.v, 4)))
            self.STree.set('w', '2', str(round(self.w, 4)))
            self.STree.set('p', '2', str(round(self.p, 4)))
            self.STree.set('q', '2', str(round(self.q, 4)))
            self.STree.set('r', '2', str(round(self.r, 4)))
            self.STree.set('M1', '2', str(round(self.M1, 4)))
            self.STree.set('M2', '2', str(round(self.M2, 4)))
            self.STree.set('M3', '2', str(round(self.M3, 4)))
            self.STree.set('M4', '2', str(round(self.M4, 4)))
            self.STree.set('X_Force', '2', str(round(self.X_Force, 4)))
            self.STree.set('Y_Force', '2', str(round(self.Y_Force, 4)))
            self.STree.set('Z_Force', '2', str(round(self.Z_Force, 4)))
            self.STree.set('K_Moment', '2', str(round(self.K_Moment, 4)))
            self.STree.set('M_Moment', '2', str(round(self.M_Moment, 4)))
            self.STree.set('N_Moment', '2', str(round(self.N_Moment, 4)))

        # If the default plotter is enabled - call the function to update those plots with SIM data
        self.time_now_plot = int(round(time.time() * 1000))
        if ((self.time_now_plot - self.time_last_plot) >= 500) and self.default_plotter_enabled.get():
            self.time_last_plot = self.time_now_plot
            self.update_default_plot()


    def Toggle_Sim_Data_Tree(self):

        # If the Sim Data Tree View is disabled, then hide it
        if self.sim_data_tree_enabled.get() == False:
            self.sim_data_frame.grid_forget() 
        else: # If the Sim Data Tree View is enabled, then show it
            self.sim_data_frame.grid(row=1, column=0, columnspan=1, sticky=W+E+N+S)
            self.master.rowconfigure(1, weight=1)
            self.master.columnconfigure(1, weight=1)

    def Toggle_Default_Plot(self):
        # If the Default Plot View is disabled, then hide it
        if self.default_plotter_enabled.get() == False:
            self.canvas.get_tk_widget().grid_forget()
            self.default_plot_frame.grid_forget()
        if self.default_plotter_enabled.get() == True: # If the Default Plot View is enabled, then show it
            self.canvas.get_tk_widget().grid(row=0, column=1, columnspan=1, rowspan=6, stick=W+E+N+S)
            self.default_plot_frame.grid(row=2, column=1, columnspan=1)


    def Unpack_SIM_Msg(self, data):
        self.roll = data.roll
        self.pitch = data.pitch
        self.yaw = data.yaw
        self.elevation = data.elevation
        self.altitude = data.altitude
        self.northings = data.northings
        self.eastings = data.eastings
        self.u_dot = data.u_dot
        self.v_dot = data.v_dot
        self.w_dot = data.w_dot
        self.p_dot = data.p_dot
        self.q_dot = data.q_dot
        self.r_dot = data.r_dot
        self.u = data.u
        self.v = data.v
        self.w = data.w
        self.p = data.p
        self.q = data.q
        self.r = data.r
        self.M1 = data.motor_1_force
        self.M2 = data.motor_2_force
        self.M3 = data.motor_3_force
        self.M4 = data.motor_4_force
        self.X_Force = data.X_Force
        self.Y_Force = data.Y_Force
        self.Z_Force = data.Z_Force
        self.K_Moment = data.K_Moment
        self.M_Moment = data.M_Moment
        self.N_Moment = data.N_Moment
        self.sim_time = data.sim_time

if __name__ == "__main__":
    app = App()