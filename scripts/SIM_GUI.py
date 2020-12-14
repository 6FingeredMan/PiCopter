#!/usr/bin/env python
# QuadCopter GUI
# For Simulation, Monitoring, and Command/Control
# Author: Brendan Martin (C) 2020
#         
###################################################

###################################################
# Included Libraries
###################################################
import ConfigParser
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
import rosgraph
###################################################
# Author Libraries
###################################################
from picopter.msg import Sim_msg
from picopter.msg import Autopilot_msg
from picopter.msg import Navigator_msg

class App():

    def __init__(self):

        # Read the configuration file
        config = ConfigParser.RawConfigParser(allow_no_value=True)
        config.read('/home/brendan/ros_catkin_ws/src/picopter/config/SIM_GUI_CONFIG.ini')
        self.update_tree_frequency = config.getfloat("Data Tree", "Update Frequency (Hz)")
        self.update_frequency = config.getfloat("Plotting", "Update Frequency (Hz)")
        self.plot_length_sec = config.getfloat("Plotting", "Plot Length (sec)")
        self.plot_array_size = int(self.update_frequency * self.plot_length_sec)
        self.plot_check_msec = (1.0 / self.update_frequency) * 1000
        self.data_tree_check_msec = (1.0 / self.update_tree_frequency) * 1000


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

        # Initialize variables
        self.Set_Variables()

        # TO DO!
        # First, check to see if the master is online, if not, 
        # pause and keep checking before proceeding

        # Start the ROS listener thread for Sim data
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
        self.autopilot_plotter_enabled = BooleanVar()
        self.autopilot_plot_initialized = BooleanVar()
        self.autopilot_plotter_enabled.set(False)
        self.autopilot_plot_initialized.set(False)
        self.viewMenu.add_checkbutton(label="Show Autopilot Plotter", onvalue=True, offvalue=False, variable=self.autopilot_plotter_enabled, command=self.Toggle_Autopilot_Plot)
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
        self.STree.insert('', 'end', 'attitude', text="Attitude", open=True)
        self.STree.insert('', 'end', 'body accelerations', text="Body Accelerations", open=True)
        self.STree.insert('', 'end', 'body velocities', text="Body Velocities", open=True)
        self.STree.insert('', 'end', 'motor forces', text="Motor Forces", open=True)
        self.STree.insert('', 'end', 'body forces', text="Body Forces", open=True)
        self.STree.insert('', 'end', 'body moments', text="Body Moments", open=True)
        self.STree.insert('', 'end', 'autopilot data', text="Autopilot", open=True)
        self.STree.insert('', 'end', 'navigator data', text="Navigator", open=True)
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
        self.STree.insert('autopilot data', 'end', 'Z Command', values=('Z Command', str(0.0000), "% Throttle"))
        self.STree.insert('autopilot data', 'end', 'Roll Command', values=('Roll Command', str(0.0000), "% Throttle"))
        self.STree.insert('autopilot data', 'end', 'Pitch Command', values=('Pitch Command', str(0.0000), "% Throttle"))
        self.STree.insert('autopilot data', 'end', 'Yaw Command', values=('Yaw Command', str(0.0000), "% Throttle"))
        self.STree.insert('navigator data', 'end', 'Target Elevation', values=('Target Elevation', str(0.0000), 'm'))
        self.STree.insert('navigator data', 'end', 'Target Course', values=('Target Course', str(0.0000), 'deg'))
        self.STree.insert('navigator data', 'end', 'Target Speed', values=('Target Speed', str(0.0000), 'm/s'))
        self.STree.insert('navigator data', 'end', 'Idle Status', values=('Idle Status', "-", 'boolean'))
        self.STree.insert('navigator data', 'end', 'Current Objective', values=('Current Objective', "-", 'm'))
        self.STree.insert('navigator data', 'end', 'Objectives Remaining', values=('Objectives Remaining', str(0), '#'))

        self.STree.pack(expand=True, fill=Y)
        self.SSB.config(command=self.STree.yview)
        self.STree.update_idletasks()

    def Plotter_View_Default(self):
        self.default_figure = Figure(figsize=(5,5), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.default_figure, master=self.master)
        self.default_plot_1 = self.default_figure.add_subplot(221)
        self.default_plot_2 = self.default_figure.add_subplot(222)
        self.default_plot_3 = self.default_figure.add_subplot(223)
        self.default_plot_4 = self.default_figure.add_subplot(224)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=1, columnspan=1, rowspan=6, stick=W+E+N+S)
        self.default_plot_frame = Frame(self.master)
        self.default_plot_frame.grid(row=2, column=1, columnspan=1)
        self.pbar = NavigationToolbar2Tk(self.canvas, self.default_plot_frame)

        ## Initialized default plotting values
        self.pitch_array = np.zeros(self.plot_array_size)
        self.roll_array = np.zeros(self.plot_array_size)
        self.yaw_array = np.zeros(self.plot_array_size)
        self.elevation_array = np.zeros(self.plot_array_size)
        self.altitude_array = np.zeros(self.plot_array_size)
        self.northings_array = np.zeros(self.plot_array_size)
        self.eastings_array = np.zeros(self.plot_array_size)
        self.time_array = np.zeros(self.plot_array_size)

    def Plotter_View_Autopilot(self):
        self.autopilot_figure = Figure(figsize=(5,5), dpi=100)
        self.canvasAP = FigureCanvasTkAgg(self.autopilot_figure, master=self.master)
        self.autopilot_plot_1 = self.autopilot_figure.add_subplot(421)
        self.autopilot_plot_2 = self.autopilot_figure.add_subplot(422)
        self.autopilot_plot_3 = self.autopilot_figure.add_subplot(423)
        self.autopilot_plot_4 = self.autopilot_figure.add_subplot(424)
        self.autopilot_plot_5 = self.autopilot_figure.add_subplot(425)
        self.autopilot_plot_6 = self.autopilot_figure.add_subplot(426)
        self.autopilot_plot_7 = self.autopilot_figure.add_subplot(427)
        self.autopilot_plot_8 = self.autopilot_figure.add_subplot(428)
        self.canvasAP.draw()
        self.canvasAP.get_tk_widget().grid(row=0, column=1, columnspan=1, rowspan=6, stick=W+E+N+S)
        self.autopilot_plot_frame = Frame(self.master)
        self.autopilot_plot_frame.grid(row=2, column=1, columnspan=1)
        self.pbarAP = NavigationToolbar2Tk(self.canvasAP, self.autopilot_plot_frame)

        ## Initialized default plotting values
        self.pitch_array = np.zeros(self.plot_array_size)
        self.pitch_rate_array = np.zeros(self.plot_array_size)
        self.roll_array = np.zeros(self.plot_array_size)
        self.roll_rate_array = np.zeros(self.plot_array_size)
        self.yaw_array = np.zeros(self.plot_array_size)
        self.yaw_rate_array = np.zeros(self.plot_array_size)
        self.elevation_array = np.zeros(self.plot_array_size)
        self.elevation_rate_array = np.zeros(self.plot_array_size)
        self.northings_array = np.zeros(self.plot_array_size)
        self.northings_rate_array = np.zeros(self.plot_array_size)
        self.eastings_array = np.zeros(self.plot_array_size)
        self.eastings_rate_array = np.zeros(self.plot_array_size)
        self.pitch_target_array = np.zeros(self.plot_array_size)
        self.roll_target_array = np.zeros(self.plot_array_size)
        self.yaw_target_array = np.zeros(self.plot_array_size)
        self.elevation_target_array = np.zeros(self.plot_array_size)
        self.pitch_rate_target_array = np.zeros(self.plot_array_size)
        self.roll_rate_target_array = np.zeros(self.plot_array_size)
        self.yaw_rate_target_array = np.zeros(self.plot_array_size)
        self.elevation_rate_target_array = np.zeros(self.plot_array_size)
        #self.time_array = np.zeros(self.plot_array_size)


    def update_default_plot(self):

        # First shift all the array data to the left by one index
        for i in range(self.plot_array_size - 1):
            self.pitch_array[i] = self.pitch_array[i+1]
            self.roll_array[i] = self.roll_array[i+1]
            self.yaw_array[i] = self.yaw_array[i+1]
            self.elevation_array[i] = self.elevation_array[i+1]
            self.altitude_array[i] = self.altitude_array[i+1]
            self.northings_array[i] = self.northings_array[i+1]
            self.eastings_array[i] = self.eastings_array[i+1]
            self.time_array[i] = self.time_array[i+1]
        # populate the last index with the latest value
        self.pitch_array[self.plot_array_size - 1] = self.pitch
        self.roll_array[self.plot_array_size - 1] = self.roll
        self.yaw_array[self.plot_array_size - 1] = self.yaw
        self.elevation_array[self.plot_array_size - 1] = self.elevation
        self.altitude_array[self.plot_array_size - 1] = self.altitude
        self.northings_array[self.plot_array_size - 1] = self.northings
        self.eastings_array[self.plot_array_size - 1] = self.eastings
        self.time_array[self.plot_array_size - 1] = self.sim_time

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
        self.default_plot_1.set_xlim(round(self.time_array[0]-1), round(self.time_array[self.plot_array_size - 1])+1)
        self.default_plot_1.set_ylim(-45, 45)
        self.default_plot_1.set_xlabel('Sim Time (sec)')
        self.default_plot_1.set_ylabel('Attitude (deg)')
        self.default_plot_2.scatter(self.eastings_array, self.northings_array, label="2D Track")
        self.default_plot_2.legend()
        self.default_plot_2.grid()
        self.default_plot_2.ticklabel_format(useOffset=False, style='plain')
        self.default_plot_2.set_ylim(round(self.northings_array[self.plot_array_size - 1] - 5), round(self.northings_array[self.plot_array_size - 1] + 5))
        self.default_plot_2.set_xlim(round(self.eastings_array[self.plot_array_size - 1] - 5), round(self.eastings_array[self.plot_array_size - 1] + 5))
        self.default_plot_2.set_xlabel('Eastings (m)')
        self.default_plot_2.set_ylabel('Northings (m)')
        self.default_plot_3.plot(self.time_array, self.yaw_array, label="Yaw")
        self.default_plot_3.legend()
        self.default_plot_3.grid()
        self.default_plot_3.set_xlim(round(self.time_array[0]-1), round(self.time_array[self.plot_array_size - 1])+1)
        self.default_plot_3.set_ylim(0, 360)
        self.default_plot_3.set_xlabel('Sim Time (sec)')
        self.default_plot_3.set_ylabel('Yaw (deg)')
        self.default_plot_4.plot(self.time_array, self.elevation_array, label="Elevation")
        self.default_plot_4.legend()
        self.default_plot_4.grid()
        self.default_plot_4.set_xlim(round(self.time_array[0]-1), round(self.time_array[self.plot_array_size - 1])+1)
        self.default_plot_4.set_ylim(0, round(self.elevation_array[self.plot_array_size - 1])+1)
        self.default_plot_4.set_xlabel('Sim Time (sec)')
        self.default_plot_4.set_ylabel('Elevation (m)')
        self.canvas.draw()

    def update_autopilot_plot(self):

        # First shift all the array data to the left by one index
        for i in range(self.plot_array_size - 1):
            self.pitch_array[i] = self.pitch_array[i+1]
            self.pitch_rate_array[i] = self.pitch_rate_array[i+1]
            self.roll_array[i] = self.roll_array[i+1]
            self.roll_rate_array[i] = self.roll_rate_array[i+1]
            self.yaw_array[i] = self.yaw_array[i+1]
            self.yaw_rate_array[i] = self.yaw_rate_array[i+1]
            self.elevation_array[i] = self.elevation_array[i+1]
            self.elevation_rate_array[i] = self.elevation_rate_array[i+1]
            self.pitch_target_array[i] = self.pitch_target_array[i+1]
            self.roll_target_array[i] = self.roll_target_array[i+1]
            self.yaw_target_array[i] = self.yaw_target_array[i+1]
            self.elevation_target_array[i] = self.elevation_target_array[i+1]
            self.pitch_rate_target_array[i] = self.pitch_rate_target_array[i+1]
            self.roll_rate_target_array[i] = self.roll_rate_target_array[i+1]
            self.yaw_rate_target_array[i] = self.yaw_rate_target_array[i+1]
            self.elevation_rate_target_array[i] = self.elevation_rate_target_array[i+1]
            self.time_array[i] = self.time_array[i+1]

        # populate the last index with the latest value
        self.pitch_array[self.plot_array_size - 1] = self.pitch
        self.pitch_rate_array[self.plot_array_size - 1] = self.q
        self.roll_array[self.plot_array_size - 1] = self.roll
        self.roll_rate_array[self.plot_array_size - 1] = self.p
        self.yaw_array[self.plot_array_size - 1] = self.yaw
        self.yaw_rate_array[self.plot_array_size - 1] = self.r
        self.elevation_array[self.plot_array_size - 1] = self.elevation
        self.elevation_rate_array[self.plot_array_size - 1] = self.w
        self.time_array[self.plot_array_size - 1] = self.sim_time
        self.pitch_target_array[self.plot_array_size - 1] = self.Target_Pitch_Position
        self.roll_target_array[self.plot_array_size - 1] = self.Target_Roll_Position
        self.yaw_target_array[self.plot_array_size - 1] = self.Target_Yaw_Position
        self.elevation_target_array[self.plot_array_size - 1] = self.Target_Z_Position
        self.pitch_rate_target_array[self.plot_array_size - 1] = self.Target_Pitch_Rate
        self.roll_rate_target_array[self.plot_array_size - 1] = self.Target_Roll_Rate
        self.yaw_rate_target_array[self.plot_array_size - 1] = self.Target_Yaw_Rate
        self.elevation_rate_target_array[self.plot_array_size - 1] = self.Target_Z_Rate

        # Clear the plots
        self.autopilot_plot_1.clear()
        self.autopilot_plot_2.clear()
        self.autopilot_plot_3.clear()
        self.autopilot_plot_4.clear()
        self.autopilot_plot_5.clear()
        self.autopilot_plot_6.clear()
        self.autopilot_plot_7.clear()
        self.autopilot_plot_8.clear()

        # Plot the new data
        self.autopilot_plot_1.plot(self.time_array, self.roll_target_array, label="Roll Target")
        self.autopilot_plot_1.plot(self.time_array, self.roll_array, label="Roll")
        self.autopilot_plot_1.legend()
        self.autopilot_plot_1.grid()
        self.autopilot_plot_1.set_xlim(round(self.time_array[0]-1), round(self.time_array[self.plot_array_size - 1])+1)
        self.autopilot_plot_1.set_xlabel('Sim Time (sec)')
        self.autopilot_plot_1.set_ylabel('Roll (deg)')
        self.autopilot_plot_2.plot(self.time_array, self.roll_rate_target_array, label="Roll Rate Target")
        self.autopilot_plot_2.plot(self.time_array, self.roll_rate_array, label="Roll Rate")
        self.autopilot_plot_2.legend()
        self.autopilot_plot_2.grid()
        self.autopilot_plot_2.set_xlim(round(self.time_array[0]-1), round(self.time_array[self.plot_array_size - 1])+1)
        self.autopilot_plot_2.set_xlabel('Sim Time (sec)')
        self.autopilot_plot_2.set_ylabel('Roll Rate (deg/s)')

        self.autopilot_plot_3.plot(self.time_array, self.pitch_target_array, label="Pitch Target")
        self.autopilot_plot_3.plot(self.time_array, self.pitch_array, label="Pitch")
        self.autopilot_plot_3.legend()
        self.autopilot_plot_3.grid()
        self.autopilot_plot_3.set_xlim(round(self.time_array[0]-1), round(self.time_array[self.plot_array_size - 1])+1)
        self.autopilot_plot_3.set_xlabel('Sim Time (sec)')
        self.autopilot_plot_3.set_ylabel('Pitch (deg)')
        self.autopilot_plot_4.plot(self.time_array, self.pitch_rate_target_array, label="Pitch Rate Target")
        self.autopilot_plot_4.plot(self.time_array, self.pitch_rate_array, label="Pitch Rate")
        self.autopilot_plot_4.legend()
        self.autopilot_plot_4.grid()
        self.autopilot_plot_4.set_xlim(round(self.time_array[0]-1), round(self.time_array[self.plot_array_size - 1])+1)
        self.autopilot_plot_4.set_xlabel('Sim Time (sec)')
        self.autopilot_plot_4.set_ylabel('Pitch Rate (deg/s)')

        self.autopilot_plot_5.plot(self.time_array, self.yaw_target_array, label="Yaw Target")
        self.autopilot_plot_5.plot(self.time_array, self.yaw_array, label="Yaw")
        self.autopilot_plot_5.legend()
        self.autopilot_plot_5.grid()
        self.autopilot_plot_5.set_xlim(round(self.time_array[0]-1), round(self.time_array[self.plot_array_size - 1])+1)
        self.autopilot_plot_5.set_xlabel('Sim Time (sec)')
        self.autopilot_plot_5.set_ylabel('Yaw (deg)')
        self.autopilot_plot_6.plot(self.time_array, self.yaw_rate_target_array, label="Yaw Rate Target")
        self.autopilot_plot_6.plot(self.time_array, self.yaw_rate_array, label="Yaw Rate")
        self.autopilot_plot_6.legend()
        self.autopilot_plot_6.grid()
        self.autopilot_plot_6.set_xlim(round(self.time_array[0]-1), round(self.time_array[self.plot_array_size - 1])+1)
        self.autopilot_plot_6.set_xlabel('Sim Time (sec)')
        self.autopilot_plot_6.set_ylabel('Yaw Rate (deg/s)')

        self.autopilot_plot_7.plot(self.time_array, self.elevation_target_array, label="Elevation Target")
        self.autopilot_plot_7.plot(self.time_array, self.elevation_array, label="Elevation")
        self.autopilot_plot_7.legend()
        self.autopilot_plot_7.grid()
        self.autopilot_plot_7.set_xlim(round(self.time_array[0]-1), round(self.time_array[self.plot_array_size - 1])+1)
        self.autopilot_plot_7.set_xlabel('Sim Time (sec)')
        self.autopilot_plot_7.set_ylabel('Elevation (m)')
        self.autopilot_plot_8.plot(self.time_array, self.elevation_rate_target_array, label="Elevation Rate Target")
        self.autopilot_plot_8.plot(self.time_array, self.elevation_rate_array, label="Elevation Rate")
        self.autopilot_plot_8.legend()
        self.autopilot_plot_8.grid()
        self.autopilot_plot_8.set_xlim(round(self.time_array[0]-1), round(self.time_array[self.plot_array_size - 1])+1)
        self.autopilot_plot_8.set_xlabel('Sim Time (sec)')
        self.autopilot_plot_8.set_ylabel('Elevation Rate (m/s)')

        self.canvasAP.draw()

    def SIM_Subscriber(self):

        # Start a ROS subscriber that listens to the "sim_data" topic and updates
        # the GUI with simulation data
        rospy.init_node("sim_gui", anonymous=True)
        self.sub = rospy.Subscriber("sim_data", Sim_msg, self.Update_SIM_Data)
        self.ap_sub = rospy.Subscriber("autopilot_data", Autopilot_msg, self.Unpack_AP_Data)
        self.nav_sub = rospy.Subscriber("nav_data", Navigator_msg, self.Unpack_Nav_Data)


    def Update_SIM_Data(self, data):
        ## This function is a ROS subscriber that updates values from the picopter sim_data topic
        #  and then populates the Simulation List Box with updated values
        self.Unpack_SIM_Msg(data)

        # Only update tree view with data twice a second and only if the sim data view is enabled
        self.time_now_tree = int(round(time.time() * 1000))
        if ((self.time_now_tree - self.time_last_tree) >= self.data_tree_check_msec) and self.sim_data_tree_enabled.get():
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
            self.STree.set('Z Command', '2', str(round(self.Z_Command, 4)))
            self.STree.set('Roll Command', '2', str(round(self.Roll_Command, 4)))
            self.STree.set('Pitch Command', '2', str(round(self.Pitch_Command, 4)))
            self.STree.set('Yaw Command', '2', str(round(self.Yaw_Command, 4)))
            self.STree.set('Target Elevation', '2', str(round(self.target_elevation, 4)))
            self.STree.set('Target Course', '2', str(round(self.target_course, 4)))
            self.STree.set('Target Speed', '2', str(round(self.target_speed, 4)))
            self.STree.set('Idle Status', '2', str(self.idle_status))
            self.STree.set('Current Objective', '2', str(self.current_objective))
            self.STree.set('Objectives Remaining', '2', str(self.objectives_remaining))


        # If the default plotter is enabled - call the function to update those plots with SIM data
        self.time_now_plot = int(round(time.time() * 1000))
        if ((self.time_now_plot - self.time_last_plot) >= self.plot_check_msec) and self.default_plotter_enabled.get():
            self.time_last_plot = self.time_now_plot
            self.update_default_plot()
        
        # If the autopilot plotter is enabled - call the function to update those plots with SIM data
        self.time_now_plot = int(round(time.time() * 1000))
        if ((self.time_now_plot - self.time_last_plot) >= self.plot_check_msec) and self.autopilot_plotter_enabled.get():
            self.time_last_plot = self.time_now_plot
            self.update_autopilot_plot()


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
        if self.default_plotter_enabled.get() == True: # If the Default Plot View is enabled, then show it but hide other plots
            self.autopilot_plotter_enabled.set(False)
            self.Toggle_Autopilot_Plot()
            self.canvas.get_tk_widget().grid(row=0, column=1, columnspan=1, rowspan=6, stick=W+E+N+S)
            self.default_plot_frame.grid(row=2, column=1, columnspan=1)

    def Toggle_Autopilot_Plot(self):
        # Check to see if the Autopilot plot has ever been created, and if not, create it
        if self.autopilot_plot_initialized.get() == False:
            self.autopilot_plot_initialized.set(True)
            self.Plotter_View_Autopilot()

        # Then toggle the view on/off
        if self.autopilot_plotter_enabled.get() == False:
            self.canvasAP.get_tk_widget().grid_forget()
            self.autopilot_plot_frame.grid_forget()
        if self.autopilot_plotter_enabled.get() == True: # If the Autopilot Plot View is enabled, then show it & hide other plots
            for i in range(self.plot_array_size):
                self.pitch_array[i] = self.pitch
                self.pitch_rate_array[i] = self.q
                self.roll_array[i] = self.roll
                self.roll_rate_array[i] = self.p
                self.yaw_array[i] = self.yaw
                self.yaw_rate_array[i] = self.r
                self.elevation_array[i] = self.elevation
                self.elevation_rate_array[i] = self.w
                self.time_array[i] = self.sim_time
                self.pitch_target_array[i] = self.Target_Pitch_Position
                self.roll_target_array[i] = self.Target_Roll_Position
                self.yaw_target_array[i] = self.Target_Yaw_Position
                self.elevation_target_array[i] = self.Target_Z_Position
                self.pitch_rate_target_array[i] = self.Target_Pitch_Rate
                self.roll_rate_target_array[i] = self.Target_Roll_Rate
                self.yaw_rate_target_array[i] = self.Target_Yaw_Rate
                self.elevation_rate_target_array[i] = self.Target_Z_Rate
                
            self.default_plotter_enabled.set(False)
            self.Toggle_Default_Plot()
            self.canvasAP.get_tk_widget().grid(row=0, column=1, columnspan=1, rowspan=6, stick=W+E+N+S)
            self.autopilot_plot_frame.grid(row=2, column=1, columnspan=1)



    def Unpack_SIM_Msg(self, data):
        self.roll = data.roll * self.R2D
        self.pitch = data.pitch * self.R2D
        self.yaw = data.yaw * self.R2D
        self.elevation = data.elevation
        self.altitude = data.altitude
        self.northings = data.northings
        self.eastings = data.eastings
        self.u_dot = data.u_dot 
        self.v_dot = data.v_dot
        self.w_dot = data.w_dot
        self.p_dot = data.p_dot * self.R2D
        self.q_dot = data.q_dot * self.R2D
        self.r_dot = data.r_dot * self.R2D
        self.u = data.u
        self.v = data.v
        self.w = data.w
        self.p = data.p * self.R2D
        self.q = data.q * self.R2D
        self.r = data.r * self.R2D
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

    def Unpack_AP_Data(self, data):
        self.Z_Command = data.z_cmd
        self.Pitch_Command = data.pitch_cmd
        self.Roll_Command = data.roll_cmd
        self.Yaw_Command = data.yaw_cmd
        self.Target_Z_Position = data.target_z_position
        self.Target_Z_Rate = data.target_z_rate
        self.Target_Pitch_Position = data.target_pitch_position
        self.Target_Pitch_Rate = data.target_pitch_rate
        self.Target_Roll_Position = data.target_roll_position
        self.Target_Roll_Rate = data.target_roll_rate
        self.Target_Yaw_Position = data.target_yaw_position
        self.Target_Yaw_Rate = data.target_yaw_rate


    def Unpack_Nav_Data(self, data):
        self.target_elevation = data.target_elevation
        self.target_course = data.target_course
        self.target_speed = data.target_speed
        self.idle_status = data.idle
        self.current_objective = data.current_objective
        self.objectives_remaining = data.objectives_remaining

    def Set_Variables(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.elevation = 0.0
        self.altitude = 0.0
        self.northings = 0.0
        self.eastings = 0.0
        self.u_dot = 0.0
        self.v_dot = 0.0
        self.w_dot = 0.0
        self.p_dot = 0.0
        self.q_dot = 0.0
        self.r_dot = 0.0
        self.u = 0.0
        self.v = 0.0
        self.w = 0.0
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        self.M1 = 0.0
        self.M2 = 0.0
        self.M3 = 0.0
        self.M4 = 0.0
        self.X_Force = 0.0
        self.Y_Force = 0.0
        self.Z_Force = 0.0
        self.K_Moment = 0.0
        self.M_Moment = 0.0
        self.N_Moment = 0.0
        self.sim_time = 0.0

        self.Z_Command = 0.0
        self.Pitch_Command = 0.0
        self.Roll_Command = 0.0
        self.Yaw_Command = 0.0
        self.Target_Z_Position = 0.0
        self.Target_Z_Rate = 0.0
        self.Target_Pitch_Position = 0.0
        self.Target_Pitch_Rate = 0.0
        self.Target_Roll_Position = 0.0
        self.Target_Roll_Rate = 0.0
        self.Target_Yaw_Position = 0.0
        self.Target_Yaw_Rate = 0.0

        self.target_elevation = 0.0
        self.target_course = 0.0
        self.target_speed = 0.0
        self.idle_status = False
        self.current_objective = "N/A"
        self.objectives_remaining = 0.0

        self.R2D = 57.2958 # radians to degrees

if __name__ == "__main__":
    app = App()