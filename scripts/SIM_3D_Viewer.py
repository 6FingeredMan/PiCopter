#!/usr/bin/env python
# QuadCopter 3D Visualization Process
# For live 3D visualization of the quadcopter while simulating
# Author: Brendan Martin (C) 2020
#         
###################################################

###################################################
# Included Libraries
###################################################
import ConfigParser
from time import sleep
import numpy as np
import sys
from direct.stdpy import thread

###################################################
# 3rd Party Libraries
###################################################
from direct.showbase.ShowBase import ShowBase   # import the bits of panda
from panda3d.core import GeoMipTerrain          # that we need
from panda3d.core import WindowProperties
from panda3d.core import CompassEffect
from panda3d.core import VBase4, VBase3, TransparencyAttrib
from panda3d.core import AmbientLight, DirectionalLight, Vec4, Vec3, Fog
from direct.task import Task
from direct.gui.OnscreenText import OnscreenText 
from direct.showbase import DirectObject
from direct.gui.DirectGui import DirectFrame, OnscreenImage
from panda3d.core import TransparencyAttrib
from panda3d.core import TextNode
from panda3d.core import ConfigVariableString
from panda3d.core import BitMask32, Texture, TextNode, TextureStage
from direct.interval.LerpInterval import LerpTexOffsetInterval, LerpPosInterval
from direct.particles.ParticleEffect import ParticleEffect
from direct.gui.OnscreenImage import OnscreenImage
import rospy

###################################################
# Author Libraries
###################################################
from picopter.msg import Sim_msg


class SimViewer(ShowBase):
    
    def __init__(self):

        self.loadConfig()

        window_title = ConfigVariableString('window-title', 'PiCopter SIM 3D') # Set the window title
        ShowBase.__init__(self)         # Create the base window

        ## Create the Terrain - Only use to generate a new .bam file
        # terrain = GeoMipTerrain("worldTerrain")
        # terrain.setHeightfield("/home/brendan/ros_catkin_ws/src/picopter/Panda3D/HeightMap1.png")
        # terrain.setColorMap("/home/brendan/ros_catkin_ws/src/picopter/Panda3D/TextureMap1.png")
        # terrain.setBruteforce(True)
        # root = terrain.getRoot()
        # root.reparentTo(render)
        # root.setSz(5)
        # terrain.generate()
        # root.writeBamFile("/home/brendan/ros_catkin_ws/src/picopter/Panda3D/World1.bam")

        ## Load the .bam world
        self.world = self.loader.loadModel("/home/brendan/ros_catkin_ws/src/picopter/Panda3D/World1.bam")
        self.world.reparentTo(self.render)
        self.worldsize = 1024

        ## Add the Quadcopter Model
        self.vehicle = self.loader.loadModel("/home/brendan/ros_catkin_ws/src/picopter/Panda3D/quad.egg")
        self.vehicle.setPos(512,512,2)
        self.vehicle.setH(0)
        self.vehicle.setP(0)
        self.vehicle.setR(0)
        self.vehicle.reparentTo(self.render)

        ## Add the camera
        self.maxdistance = 1000
        self.camLens.setFar(self.maxdistance)
        self.camLens.setFov(60)
        self.cam.setPos(self.vehicle, 2, 2, .5)
        self.cam.setHpr(self.vehicle, 0, 0, 0)
        self.cam.lookAt(self.vehicle)
        base.disableMouse()

        ## Add the top left text data display
        Pitch_Label = " Pitch: "
        Roll_Label = "   Roll: "
        Yaw_Label = "  Yaw: "
        X_Label = "X Pos: "
        Y_Label = "Y Pos: "
        Z_Label = "Z Pos: "
        PitchTxt = OnscreenText(text = Pitch_Label, pos = (+.1, -.05), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=0)
        PitchTxt.reparentTo(base.a2dTopLeft)
        self.PitchVTxt = OnscreenText(text = "0.00", pos = (+.25, -.05), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=1)
        self.PitchVTxt.reparentTo(base.a2dTopLeft)
        RollTxt = OnscreenText(text = Roll_Label, pos = (+.1, -.1), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=0)
        RollTxt.reparentTo(base.a2dTopLeft)
        self.RollVTxt = OnscreenText(text = "0.00", pos = (+.25, -.1), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=1)
        self.RollVTxt.reparentTo(base.a2dTopLeft)
        YawTxt = OnscreenText(text = Yaw_Label, pos = (+.1, -.15), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=0)
        YawTxt.reparentTo(base.a2dTopLeft)
        self.YawVTxt = OnscreenText(text = "0.00", pos = (+.25, -.15), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=1)
        self.YawVTxt.reparentTo(base.a2dTopLeft)
        XTxt = OnscreenText(text = X_Label, pos = (+.1, -.2), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=0)
        XTxt.reparentTo(base.a2dTopLeft)
        self.XVTxt = OnscreenText(text = "0.00", pos = (+.25, -.2), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=1)
        self.XVTxt.reparentTo(base.a2dTopLeft)
        YTxt = OnscreenText(text = Y_Label, pos = (+.1, -.25), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=0)
        YTxt.reparentTo(base.a2dTopLeft)
        self.YVTxt = OnscreenText(text = "0.00", pos = (+.25, -.25), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=1)
        self.YVTxt.reparentTo(base.a2dTopLeft)
        ZTxt = OnscreenText(text = Z_Label, pos = (+.1, -.3), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=0)
        ZTxt.reparentTo(base.a2dTopLeft)
        self.ZVTxt = OnscreenText(text = "0.00", pos = (+.25, -.3), scale = 0.05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=1)
        self.ZVTxt.reparentTo(base.a2dTopLeft)

        # TOP RIGHT WINDOW TEXT
        TimeTxt = OnscreenText(text = "Mission Clock (sec): ", pos = (-.5, -.05), scale = .05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=0)
        TimeTxt.reparentTo(base.a2dTopRight)
        self.TimeVTxt = OnscreenText(text = "0.0", pos = (-.15, -.05), scale = .05, fg=(0,0,0,1), bg=(1,1,1,1), mayChange=1)
        self.TimeVTxt.reparentTo(base.a2dTopRight)

        # Show frame rate or not based on config value
        base.setFrameRateMeter(self.show_frame_rate)

        ## Create the Main Update Task
        self.MainTask = taskMgr.doMethodLater(self.frame_rate_sec, self.Update_Visual, "Update Visual", appendTask=True)
        
        ## Create the ROS Subscriber 
        self.ROS_SIM_SUB()

    def ROS_SIM_SUB(self):
        rospy.init_node("sim_3D_node", anonymous=True)
        self.sub = rospy.Subscriber("sim_data", Sim_msg, self.Update_Pose)

    def Update_Pose(self, data):
        self.roll = data.roll * self.R2D
        self.pitch = data.pitch * self.R2D
        self.yaw = data.yaw * self.R2D
        self.elevation = data.elevation
        self.sim_time = data.sim_time

    def Update_Visual(self, task):
        self.vehicle.setH(self.yaw)
        self.vehicle.setP(self.pitch)
        self.vehicle.setR(self.roll)
        self.vehicle.setPos(512, 512, self.elevation + 2)
        self.cam.setPos(self.vehicle, 2, 2, .5)
        self.cam.setHpr(self.vehicle, 0, 0, 0)
        self.cam.lookAt(self.vehicle)
        self.PitchVTxt.setText(str(round(self.pitch,2)))
        self.RollVTxt.setText(str(round(self.roll,2)))
        self.YawVTxt.setText(str(round(self.yaw,2)))
        self.XVTxt.setText(str(round(0,2)))
        self.YVTxt.setText(str(round(0,2)))
        self.ZVTxt.setText(str(round(self.elevation,2)))
        self.TimeVTxt.setText(str(round(self.sim_time, 1)))

        return Task.again

    def loadConfig(self):
        config = ConfigParser.RawConfigParser(allow_no_value=True)
        config.read('/home/brendan/ros_catkin_ws/src/picopter/config/SIM_3D_CONFIG.ini')
        self.frame_rate_hz = config.getfloat("Visualization", "target frame rate (hz)")
        self.show_frame_rate = config.getboolean("Visualization", "show frame rate")
        self.frame_rate_sec = 1.0 / self.frame_rate_hz

        # Initialize Variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.elevation = 0.0
        self.sim_time = 0.0

        self.R2D = 57.2958 # radians to degrees





if __name__ == "__main__":
    app = SimViewer()
    app.run()

