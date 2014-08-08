## Iris Aguilar August 4th
## IrisTwelveth generates a five-button GUI that can control the Nerf Launcher,
## has a dynamic right-sided display, and shows transformed maps on the bottom panel.
## It also has additional toolbar options for exiting, keyboard mode, picture saving, 
## generating and comparing a binary brightness pattern(BP), generating and comparing a monochromatic brightness pattern (MBP),
## generating and comparing a binary BP using top-down approach(BTD), generating and comparing an MBP using the top-down approach (MTD),
## and generating and comparing an RGB pattern (RGBP).
## Files saved: last_, last_a (resized), last_ABW (resized and B/W), last_ABWH (resized, B/W, Harris corners), last_BP (brightness pattern)
## Details:
## resized captured image : imgs/last_a.png
## B/W resized captured image : imgs/last_aBW.png
## BBP from ^ : imgs/last_BBP.png
## MBP from ^^ : imgs/last_MBP.png
## matching score for BBP : imgs/MatchingScoreArray_.txt
## matching score for MBP : imgs/fancyMatchingScoreArray_.txt

## Before running IrisTwelveth, make sure the launcher is connected to your machine
## run roscore, then sudo chmod -R 777 /dev/bus/usb/, enter password
## List of files you'll need [lines 98 through 109]: mapColorOriginal.jpg mapColorRzd.jpg a 800 x 250 'map' for display purposes only,
## mapBBPrzd.jpg a brightness pattern resized to 800 x 250 for display purposes only,
## mapBinaryArrayBP.txt a binary array of the map BP generated with the IrisBP.py code..used 50 as the gridValue
## mapMBPrzd.jpg, mapMonochromaticBP.txt
## mapBTDrzd.jpg, mapBinaryArrayTD.txt
## mapMTDrzd.jpg, mapMonochromaticTD.txt

##############
##TO-DO LIST############################################################################
##add actual live feed##################################################################
##work on better hide() option##########################################################
##add results on a different updateable tab?############################################
##orrrrr, erase the buttons and after an image is captured, do all the comparisons######
##spit this out onto the bottom (results) and the left (transformed) panels#############
##orr if unlock, display four more tabs between the original tabs that contain results##
##fix a loop acc. to IrisX##############################################################
##change D.arrayOne#####################################################################
########################################################################################

##TABLE OF CONTENTS:
##imports : lines 49 to 75...should go through these and make sure I actually need all of these
##instantiation of global variables : lines 82 to 96
##connectToLauncher: function that connects to launcher
##initUI : function that sets up the toolbar with Exit, Keyboard, Camera, BP, fancyBP, and Harris icons
##buttonPanel : sets up the left dock with the five directional buttons
##liveFeedPanel : place holder for right dock that will manage live feed from launcher
##cameraFeedPanel : re-sets up the right dock after the camera icon is pressed with saved image in place of live feed
##mapPanel : displays the map
##switchToKeyboard : called when keyboard icon is pressed, keyboard keys have control
##saveImage : called when camera button is pressed
##findBrightnessPattern : 
##matchIndexing : matches up brightness patterns of the map and newly generated one of image captured

##resizeImage : resizes captured image to D.imgX and D.imgY values and turns resized image BW for use with the two BPs
##moveUp/moveDown/moveLeft/moveRight/stopMoving/sendCdeToLauncher


import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import irobot_mudd
import cv_bridge
import cv
import cv2
import sensor_msgs.msg as sm

import os
import usb.core

from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *

import sys
from PySide import QtCore, QtGui ##import GUI widgets, ##
from PySide.QtCore import Slot

import L9strippedImage
from PIL import Image
import numpy as np

from numpy import *
from scipy.ndimage import filters

from pylab import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class Data:
	def __init__(self): pass

D = Data()
D.imgNum = 0 	 				##used throughtout the program to number related saved images and .txt files
D.initial = True 				##used in cameraFeedPanel function to identify which box should be hidden ###could be better if we found a function to overwrite rather than hide...send a new img to the og left panel?? oooh
D.mapInitial = True 			##used in mapPanel function for identifying when map box should be overwritten
D.gridValue = 20 				##used throughout the program, this is the grid value for the images and could use some calibration if you have the time
D.mapGridValue = 50 			##used throughout the program, this is the grid value of the map and could use some calibration if you have the time
D.n = 0 						##used throughout the program as a generic counter 
D.imgX = 320 					##used throughout the program as the width dimension for the captured image ##what is captured img dimensions?
D.imgY = 240  					##used throughout the program as the length dimension for the captured image
D.BLACK = 0
D.WHITE = 255

D.columns = D.imgX/D.gridValue ##columns of squares in the BPimage ##16 when length is 320 and grids are 20
D.rows = D.imgY/D.gridValue ##rows of squares in the BPimage ##12 when width is 240 and grids are 20

D.colorMap = 'mapColorOriginal.jpg'  		##full size, color map used for measurements and for displaying the two matching results
D.displayColorMap = 'mapColorRzd.jpg'	 	##used as initial value of map image in ImagePanel and is updated when mapPanel is called again 
D.displayBPbinary = 'mapBBPrzd.jpg'			## saved resized binary BP map
D.displayBPmonochromatic = 'mapMBPrzd.jpg'	## saved resized monochromatic BP map
D.displayTDbinary = 'mapBTDrzd.jpg'
D.displayTDmonochromatic = 'mapMTDrzd.jpg'
D.displayRGB = 'mapRGBBPrzd.jpg'
D.resultMap = 'mapResultSpaceHolder.jpg'

D.mapArrayTxtbBP = 'mapBinaryArrayBP.txt'#'mapUpstairsBinaryArrayBP.txt' ##saved binary translation of BP map image
D.mapArrayTxtmBP = 'mapMonochromaticBP.txt'#'mapUpstairsMonochromaticBP.txt' ##saved numeral translation of fancy BP map image
D.mapArrayTxtbTD = 'mapBinaryArrayTD.txt'#'mapBinaryArrayTD.txt' ##saved binary translation of fancy BP map image TD method
D.mapArrayTxtmTD = 'mapMonochromaticTD.txt'#'mapMonochromaticTD.txt' ####saved numeral translation of fancy BP map image TD method
D.mapRGBArray = 'mapRGBarray.txt'
##complete list of D.__ (global variables) even if they are not instantiated here


##**********************************************************
##*********DEFAULT MAP AND IMAGE FOR DEMO PURPOSES**********
##**********************************************************

#D.defaultMap = 'mapUpstairsClrOg.jpg'
#D.defaultDisplayColorMap = 'mapUpstairsClrRzd.jpg'
#D.defaultDisplayBPbin = 'mapUpstairsBinBPrzd.jpg'

#D.displayBPmonochromatic = 'mapMonochromaticBPrzd.jpg'  ## saved resized fancy BP map
#D.displayTDbinary = 'binaryTDmapRzd.jpg'
#D.displayTDMonochromatic = 'monochromaticTDmapRzd.jpg
#D.mapArrayTxtbBP = 'mapBinaryArrayBP.txt' ##saved binary translation of BP map image
#D.mapArrayTxtmBP = 'mapMonochromaticBP.txt' ##saved numeral translation of fancy BP map image
#D.mapArrayTxtbTD = 'mapBinaryArrayTD.txt' ##saved binary translation of fancy BP map image TD method
#D.mapArrayTxtmTD = 'mapMonochromaticTD.txt' ####saved numeral translation of fancy BP map image TD method
#D.defaultImage = 


class IrisEleventh(QtGui.QMainWindow): ## class inherits from QtGui.QMainWindow
	def __init__(self):         ##self is an instance of QtGui.QWidget
		super(IrisEleventh, self).__init__() ## two constructors called

		self.connectToLauncher() ##line 90 - connects to Nerf launcher/camera
		self.initUI()			 ##line 101 - generates the toolbar buttons with calls to their functions
		self.buttonPanel()		 ##line 151 - generates five buttons to control the launcher (function calls and position)
		self.liveFeedPanel()	 ##line 212 - before any tool bar buttons are pressed a live feed from the launcher should be shown in the right dock 
		self.mapPanel()		 	 ##line 267 - displays resized map image in the bottom dock ###change this function's name

	def connectToLauncher(self):
		self.launcher = usb.core.find(idVendor=0x2123, idProduct=0x1010)
		if self.launcher is None:
			raise ValueError('Launcher not found.')
		if self.launcher.is_kernel_driver_active(0) is True:
			self.launcher.detach_kernel_driver(0)
		self.launcher.set_configuration()
		#print "Just making sure the launcher is still connected"

		
	def initUI(self):                           
		exitAction = QtGui.QAction('Exit', self)
		exitAction = QtGui.QAction(QtGui.QIcon('exit.png'), 'Exit', self)
		exitAction.setShortcut('Ctrl+Q')
		exitAction.setStatusTip('Exit app')
		exitAction.triggered.connect(self.close)
		self.toolbar = self.addToolBar('Exit')
		self.toolbar.addAction(exitAction)

		keyboardAction = QtGui.QAction('Keyboard', self)
		keyboardAction = QtGui.QAction(QtGui.QIcon('keyboard.png'), 'Keyboard', self)
		keyboardAction.setShortcut('Ctrl+K')
		keyboardAction.setStatusTip('Switch to keyboard')
		keyboardAction.triggered.connect(self.switchToKeyboard)
		self.toolbar = self.addToolBar('Keyboard')
		self.toolbar.addAction(keyboardAction)

		cameraAction = QtGui.QAction('Camera', self)
		cameraAction = QtGui.QAction(QtGui.QIcon('camera.png'), 'Camera', self)
		cameraAction.setShortcut('Ctrl+C')
		cameraAction.setStatusTip('Save Image')
		cameraAction.triggered.connect(self.saveImage)
		self.toolbar = self.addToolBar('Camera')
		self.toolbar.addAction(cameraAction)

		brightnessAction = QtGui.QAction('Binary Brightness Pattern', self)
		brightnessAction = QtGui.QAction(QtGui.QIcon('brghtnssPttrn.png'), 'Binary Brightness Pattern', self)
		brightnessAction.setShortcut('Ctrl+B')
		brightnessAction.setStatusTip('Find binary brightness pattern of map and image')
		brightnessAction.triggered.connect(self.brightnessPattern)
		self.toolbar = self.addToolBar('Binary Brightness Pattern')
		self.toolbar.addAction(brightnessAction)

		fancyBPAction = QtGui.QAction('Monochromatic Brightness Pattern', self)
		fancyBPAction = QtGui.QAction(QtGui.QIcon('fancyBP.jpg'), 'Monochromatic Brightness Pattern', self)
		fancyBPAction.setShortcut('Ctrl+F')
		fancyBPAction.setStatusTip('Find monochromatic brightness pattern of map and image')
		fancyBPAction.triggered.connect(self.fancyBrightnessPattern)
		self.toolbar = self.addToolBar('Monochromatic Brightness Pattern')
		self.toolbar.addAction(fancyBPAction)

		topDownAction = QtGui.QAction('Binary Top Down', self)
		topDownAction = QtGui.QAction(QtGui.QIcon('fallingpixels.jpg'), 'Binary Top Down', self)
		topDownAction.setShortcut('Ctrl+T')
		topDownAction.setStatusTip('Find binary brightness pattern of map and image using the top-down approach')
		topDownAction.triggered.connect(self.topDownBP)
		self.toolbar = self.addToolBar('Top Down')
		self.toolbar.addAction(topDownAction)

		fancyTDAction = QtGui.QAction('Monochromatic Top Down', self)
		fancyTDAction = QtGui.QAction(QtGui.QIcon('topDownFancy.jpg'), 'Monochromatic Top Down', self)
		fancyTDAction.setShortcut('Ctrl+D')
		fancyTDAction.setStatusTip('Find Monochromatic  brightness pattern of map and image using the top-down approach')
		fancyTDAction.triggered.connect(self.fancyTopDownBP)
		self.toolbar = self.addToolBar('Monochromatic Top Down')
		self.toolbar.addAction(fancyTDAction)

		bgrAction = QtGui.QAction('RGB pattern', self)
		bgrAction = QtGui.QAction(QtGui.QIcon('gbrBP.jpg'), 'RGB pattern', self)
		bgrAction.setShortcut('Ctrl+G')
		bgrAction.setStatusTip('Find RGB pattern of map and image')
		bgrAction.triggered.connect(self.bgrBP)######
		self.toolbar = self.addToolBar('RGB pattern')
		self.toolbar.addAction(bgrAction)

		####ADD A BUTTON THAT SETS THE DEFAULTS...INCLUDING A CAPTURED IMAGE
		#D.colorMap = D.defaultMap
		#D.displayColorMap = D.defaultDisplayColorMap
		#D.displayBPbinary = D.defaultDisplayBPbin


	def buttonPanel(self):
		buttonBox = QtGui.QWidget(self)
		buttonBoxLayout = QtGui.QGridLayout()
		buttonBox.setLayout(buttonBoxLayout)
		
		directionalButtonGroup = QtGui.QGroupBox(" ")
		directionalButtonGroup.setAlignment(QtCore.Qt.AlignHCenter)
		
		upButton = QtGui.QPushButton("Up", self)
		upButton.clicked.connect(self.moveUp)
		##
		leftButton = QtGui.QPushButton("Left", self)
		leftButton.clicked.connect(self.moveLeft)
		##
		stopButton = QtGui.QPushButton("Stop", self)
		stopButton.clicked.connect(self.stopMoving)
		##
		rightButton = QtGui.QPushButton("Right", self)
		rightButton.clicked.connect(self.moveRight)
		##
		downButton = QtGui.QPushButton("Down", self)
		downButton.clicked.connect(self.moveDown)

		self.statusBar().showMessage('Idle Mode - Idle Mode - Idle Mode')
		
		directionalButtonLayout = QtGui.QGridLayout()
		directionalButtonGroup.setLayout(directionalButtonLayout)
		directionalButtonLayout.setRowStretch(0, 1) ##but what does this mean..one horizontal space? #####test this ooot

		directionalButtonLayout.addWidget(upButton, 0, 1, QtCore.Qt.AlignHCenter)
		directionalButtonLayout.setRowMinimumHeight(1, 10)

		directionalButtonLayout.addWidget(leftButton, 1, 0, QtCore.Qt.AlignHCenter)
		directionalButtonLayout.setRowMinimumHeight(1, 10)

		directionalButtonLayout.addWidget(stopButton, 1, 1, QtCore.Qt.AlignHCenter)
		directionalButtonLayout.setRowMinimumHeight(1, 10)
 
		directionalButtonLayout.addWidget(rightButton, 1, 2, QtCore.Qt.AlignHCenter)
		directionalButtonLayout.setRowMinimumHeight(1, 10)

		directionalButtonLayout.addWidget(downButton, 2, 1, QtCore.Qt.AlignHCenter)
		directionalButtonLayout.setRowMinimumHeight(1, 10)

		buttonBoxLayout.setRowStretch(0,1)
		buttonBoxLayout.addWidget(directionalButtonGroup, 1, 0)

		######
		#dock#
		######        
		buttonDock = QtGui.QDockWidget("", self)
		buttonDock.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea)
		buttonDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
		buttonDock.setWidget(buttonBox)
		self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, buttonDock)
		############################################################PROGRAM'S ICON
		self.setWindowIcon(QtGui.QIcon('sunsetIris.jpeg'))

		self.show()
		

	def liveFeedPanel(self): ##///////////////////////////////////////////////////RIGHT LIVE FEED FROM LAUNCHER
		D.liveImageBox = QtGui.QWidget(self)
		liveImageBoxLayout = QtGui.QGridLayout()
		D.liveImageBox.setLayout(liveImageBoxLayout)

		liveImage = QtGui.QPixmap()
		liveImage.load('launcherLiveFeed.png')
		liveImageLabel = QtGui.QLabel(self)
		liveImageLabel.setPixmap(liveImage)
		
		liveImageBoxLayout.setRowStretch(0,1)
		liveImageBoxLayout.addWidget(liveImageLabel)

		liveImageBoxLayout.setRowStretch(3,1)
		######
		#dock#
		######        
		liveImageDock = QtGui.QDockWidget("", self)
		liveImageDock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
		liveImageDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
		liveImageDock.setWidget(D.liveImageBox)
		self.addDockWidget(QtCore.Qt.RightDockWidgetArea, liveImageDock)


	def cameraFeedPanel(self): ##/////////////////////////////////////////////////////RIGHT WHEN IMAGE CAPTURED
		if (D.initial == True):
			D.liveImageBox.hide()

		if (D.initial != True):
			D.pictureBox.hide()
			#D.pictureBox.QWidget()
			##update() ## not sure how this works ##deleteLater() ##makes new = dock? ##hide() just scoots down each time
			##Destroy didnt work either

		D.pictureBox = QtGui.QWidget(self)
		pictureBoxLayout = QtGui.QGridLayout()
		D.pictureBox.setLayout(pictureBoxLayout)

		picture = QtGui.QPixmap()
		picture.load(D.displayImage)
		pictureLabel = QtGui.QLabel(self)
		pictureLabel.setPixmap(picture)
		
		pictureBoxLayout.setRowStretch(0,0) ##row0 has a stretch of zero
		pictureBoxLayout.addWidget(pictureLabel)

		######
		#dock#
		######
		pictureDock = QtGui.QDockWidget("", self)
		pictureDock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
		pictureDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
		pictureDock.setWidget(D.pictureBox)
		self.addDockWidget(QtCore.Qt.RightDockWidgetArea, pictureDock)
		D.initial = False


##renders the smaller map images in their relative tabs
	
	def mapPanel(self): ##/////////////////////////////////////////////////////////////////////BOTTOM MAP d.mapinitial not necessary
		"""
		if (D.initial != True):
			D.resultImageBox.hide()
			##D.resultMap in sixth tab
			D.resultImageBox = QtGui.QWidget(self)
			resultImageBoxLayout = QtGui.QGridLayout()
			D.resultImageBox.setLayout(resultImageBoxLayout)

			resultMapImage = QtGui.QPixmap()
			resultMapImage.load(D.resultMap)#############
			resultMapImageLabel = QtGui.QLabel(self)
			resultMapImageLabel.setPixmap(resultMapImage)
			resultImageBoxLayout.setRowStretch(0,1)
			resultImageBoxLayout.addWidget(resultMapImageLabel)

			resultMapGroup = QtGui.QWidget(self)
			resultMapLayout = QtGui.QGridLayout()
			resultMapGroup.setLayout(resultMapLayout)
			resultMapLayout.addWidget(D.resultImageBox)
			##
			D.mapTabs = QtGui.QTabWidget(self)
			D.mapTabs.addTab(resultMapGroup, "Matching Result")

		else:
		"""
		##D.displayMap in first tab
		D.imageBox = QtGui.QWidget(self)
		imageBoxLayout = QtGui.QGridLayout()
		D.imageBox.setLayout(imageBoxLayout)

		mapImage = QtGui.QPixmap()
		mapImage.load(D.displayColorMap)#####
		mapImageLabel = QtGui.QLabel(self)
		mapImageLabel.setPixmap(mapImage)
		imageBoxLayout.setRowStretch(0,1)
		imageBoxLayout.addWidget(mapImageLabel)
		
		ogImageGroup = QtGui.QWidget(self)
		ogImageLayout = QtGui.QGridLayout()
		ogImageGroup.setLayout(ogImageLayout)
		ogImageLayout.addWidget(D.imageBox)
		##CAN WE ADD A STATUS BAR MESSAGE ??
		##self.statusBar().showMessage('THIS IS THE COLOR MAP, NO TRANSFORMATIONS')
		##
		##D.displayBPbinary in second tab
		D.bwBPimageBox = QtGui.QWidget(self)
		bwBPimageBoxLayout = QtGui.QGridLayout()
		D.bwBPimageBox.setLayout(bwBPimageBoxLayout)

		bwBPmapImage = QtGui.QPixmap()
		bwBPmapImage.load(D.displayBPbinary)#####
		bwBPmapImageLabel = QtGui.QLabel(self)
		bwBPmapImageLabel.setPixmap(bwBPmapImage)
		bwBPimageBoxLayout.setRowStretch(0,1)
		bwBPimageBoxLayout.addWidget(bwBPmapImageLabel)
		
		bwBPimageGroup = QtGui.QWidget(self)
		bwBPimageLayout = QtGui.QGridLayout()
		bwBPimageGroup.setLayout(bwBPimageLayout)
		bwBPimageLayout.addWidget(D.bwBPimageBox)
		##
		##D.displayBPmonochromatic in third tab
		D.mnBPimageBox = QtGui.QWidget(self)
		mnBPimageBoxLayout = QtGui.QGridLayout()
		D.mnBPimageBox.setLayout(mnBPimageBoxLayout)

		mnBPmapImage = QtGui.QPixmap()
		mnBPmapImage.load(D.displayBPmonochromatic)#####
		mnBPmapImageLabel = QtGui.QLabel(self)
		mnBPmapImageLabel.setPixmap(mnBPmapImage)
		mnBPimageBoxLayout.setRowStretch(0,1)
		mnBPimageBoxLayout.addWidget(mnBPmapImageLabel)
		
		mnBPimageGroup = QtGui.QWidget(self)
		mnBPimageLayout = QtGui.QGridLayout()
		mnBPimageGroup.setLayout(mnBPimageLayout)
		mnBPimageLayout.addWidget(D.mnBPimageBox)
		##
		##D.displayTDbinary in fourth tab
		D.bwTDimageBox = QtGui.QWidget(self)
		bwTDimageBoxLayout = QtGui.QGridLayout()
		D.bwTDimageBox.setLayout(bwTDimageBoxLayout)

		bwTDmapImage = QtGui.QPixmap()
		bwTDmapImage.load(D.displayTDbinary)
		bwTDmapImageLabel = QtGui.QLabel(self)
		bwTDmapImageLabel.setPixmap(bwTDmapImage)
		bwTDimageBoxLayout.setRowStretch(0,1)
		bwTDimageBoxLayout.addWidget(bwTDmapImageLabel)

		bwTDimageGroup = QtGui.QWidget(self)
		bwTDimageLayout = QtGui.QGridLayout()
		bwTDimageGroup.setLayout(bwTDimageLayout)
		bwTDimageLayout.addWidget(D.bwTDimageBox)		
		##
		##D.displayTDmonochromatic in fifth tab
		D.mnTDimageBox = QtGui.QWidget(self)
		mnTDimageBoxLayout = QtGui.QGridLayout()
		D.mnTDimageBox.setLayout(mnTDimageBoxLayout)

		mnTDmapImage = QtGui.QPixmap()
		mnTDmapImage.load(D.displayTDmonochromatic)
		mnTDmapImageLabel = QtGui.QLabel(self)
		mnTDmapImageLabel.setPixmap(mnTDmapImage)
		mnTDimageBoxLayout.setRowStretch(0,1)
		mnTDimageBoxLayout.addWidget(mnTDmapImageLabel)

		mnTDimageGroup = QtGui.QWidget(self)
		mnTDimageLayout = QtGui.QGridLayout()
		mnTDimageGroup.setLayout(mnTDimageLayout)
		mnTDimageLayout.addWidget(D.mnTDimageBox)
		##
		##D.displayRGB in sixth tab
		D.RGBimageBox = QtGui.QWidget(self)
		RGBimageBoxLayout = QtGui.QGridLayout()
		D.RGBimageBox.setLayout(RGBimageBoxLayout)

		RGBmapImage = QtGui.QPixmap()
		RGBmapImage.load(D.displayRGB)
		RGBmapImageLabel = QtGui.QLabel(self)
		RGBmapImageLabel.setPixmap(RGBmapImage)
		RGBimageBoxLayout.setRowStretch(0,1)
		RGBimageBoxLayout.addWidget(RGBmapImageLabel)

		RGBimageGroup = QtGui.QWidget(self)
		RGBimageLayout = QtGui.QGridLayout()
		RGBimageGroup.setLayout(RGBimageLayout)
		RGBimageLayout.addWidget(D.RGBimageBox)
		##

		"""
		##D.resultMap in seventh tab
		D.resultImageBox = QtGui.QWidget(self)
		resultImageBoxLayout = QtGui.QGridLayout()
		D.resultImageBox.setLayout(resultImageBoxLayout)

		resultMapImage = QtGui.QPixmap()
		resultMapImage.load(D.resultMap)#############
		resultMapImageLabel = QtGui.QLabel(self)
		resultMapImageLabel.setPixmap(resultMapImage)
		resultImageBoxLayout.setRowStretch(0,1)
		resultImageBoxLayout.addWidget(resultMapImageLabel)

		resultMapGroup = QtGui.QWidget(self)
		resultMapLayout = QtGui.QGridLayout()
		resultMapGroup.setLayout(resultMapLayout)
		resultMapLayout.addWidget(D.resultImageBox)
		##
		"""

		##tabs (saved and resized maps after transformations are rendered at the begining of program)
		D.mapTabs = QtGui.QTabWidget(self)
		D.mapTabs.addTab(ogImageGroup, "Color Map")
		D.mapTabs.addTab(bwBPimageGroup, "Binary BP")
		D.mapTabs.addTab(mnBPimageGroup, "Monochromatic BP")
		D.mapTabs.addTab(bwTDimageGroup, "Binary TD")
		D.mapTabs.addTab(mnTDimageGroup, "Monochromatic TD")
		D.mapTabs.addTab(RGBimageGroup, "RGB Pattern")
		#D.mapTabs.addTab(resultMapGroup, "Matching Result")

		######
		#dock#
		######        
		imageDock = QtGui.QDockWidget("", self)
		imageDock.setAllowedAreas(QtCore.Qt.BottomDockWidgetArea)
		imageDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
		imageDock.setWidget(D.mapTabs)
		self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, imageDock)


	##///////////////////////////////////////////////////////////////KEYBOARD ICON//////////////##
	def switchToKeyboard(self):
		self.statusBar().showMessage('In keyboard mode now')
		
		kw = cv.NamedWindow("keyboard window") ##this window must be selected for keyboard mode to work...can we fix that?
		
		D.quiting = False ##False until 'q' makes it True
		
		while D.quiting != True:
			
			raw = cv.WaitKey(5000) ##only works with CV window open
			D.key_press = raw & 255
			if D.key_press != 255:
				self.checkKeyPress()
		print "Press the x in the uppper left corner to exit keyboard mode."
		

	##////////////////////////////////////////////////////////////////CAMERA ICON///////////////##
	def saveImage(self):
		pressed = True ##bool thats only used within this function to generate and save new pictures
						
		cap = cv.CaptureFromCAM(-1)
		D.imgNum += 1 ##updated after every captured image so all subsequent files are related by number

		cv.NamedWindow('frame')

		##saves image from frame, resizes, and displays it in place of the live image 
		##once we get the live image up, it should return to live image if we dont want to analyze the rendered pic...add button for this
		while (pressed):
			frame = cv.QueryFrame(cap)
			cv.ShowImage('frame',frame) ######### it would be great if this didnt have to pop up
			D.originalImage = "imgs/last" + str(D.imgNum) + ".png"
			cv.SaveImage(D.originalImage, frame) ##original-sized captured image saved as filename

			self.statusBar().showMessage('Saving image...')	

			self.resizeCameraShot()## call line 1172 to resize original to D.imgX, D.imgY dimensions, saves to filenameA

			pressed = False

			D.displayImage = D.imageNameA
			self.cameraFeedPanel() ## call cameraFeed to hide liveFeed and render our resized image whose name is now accessable with D.displayImage

			cv.DestroyWindow('frame') ##closes the frame window of the captured image

		#print "Picture has been saved."


	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##//////////////////////////////////////////////////////////////////////////BP ICON//////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	def brightnessPattern(self): ##calls generateBP and matching
		##uses imageNameABW, saves imageNameBP
		self.statusBar().showMessage('Binary brightness pattern generating')
				
		##converted to BW when it was resized (ABW vvvvvvv), send it to be BPed
		self.generateBinaryBrightnessPattern(D.imageNameABW) ## does not need to be global because it is being sent explicitly, returns D.img
		##generates a binaryBP image and a text file that represents its values
		
		##function returns a changed D.img, save it here
		D.imageNameBP = "imgs/last" + str(D.imgNum) + "BBP.png"
		cv2.imwrite(D.imageNameBP,D.img) ##opened with cv2.imread, saved with cv2.imwrite

		#save two dimensional array to a file
		D.binArrayTxt = 'imgs/imgBinaryArray' + str(D.imgNum) + 'BBP.txt'
		np.savetxt(D.binArrayTxt, D.imgBinaryArray, fmt='%d')
		
		##call cameraFeed to refresh img
		D.displayImage = D.imageNameBP
		self.cameraFeedPanel()## call cameraFeed to hide liveFeed and render our resized BP image whose name is now accessable with filename

		self.matchIndexing() ##BP not displayed until after this is closed..and thats not ok
		

	def generateBinaryBrightnessPattern(self,imageName): ##uses a BW image and returns the incoming image's BP
		sumOfValues = 0 ## accumulator for grey-scale values of each pixel in BW version of captured image
		sumOfSubsection = 0 ## accumulator for grey-scale values of each pixel in 20 x 20 sections of BW version of captured image

		D.img = cv2.imread(imageName) ## open the just-saved BW
		length, width, depth = D.img.shape		 ## length, width, color channels (we just re-seized, we should know the size..)
		D.columns = width/D.gridValue ##columns of squares in the BPimage ##16 when length is 320 and grids are 20
		D.rows = length/D.gridValue
		
		##reading ALL pixel values, and averaging their value
		for y in xrange(length):
			for x in xrange(width):
				px = D.img[y,x]
				pxInfo = str(px).strip('[]')
				splitPXinfo = pxInfo.split(" ") ##grey-scale image returns RGB values that are the same (62, 62, 62..for example), we only need one
				BWvalue = int(splitPXinfo[0])
				sumOfValues += BWvalue
				x += 1
			y += 1

		avg = sumOfValues/(length * width) ##average grey values of entire image
				
		##using D.gridValue/D.gridValue, divide the image into subsections
		##the grid loop to traverse 20 x 20 sections
		xcount = 0
		ycount = 0
		numOfColumns = (width/D.gridValue)
		
		while(((xcount + D.gridValue) <= width) and (ycount + D.gridValue <= length)):
			for a in range(numOfColumns):
				for y in range(D.gridValue) :
					for x in range(D.gridValue):
						px = D.img[(y+ycount),(x+xcount)]
						pxInfo = str(px).strip('[]')
						splitPXinfo = pxInfo.split(" ")
						BWvalue = int(splitPXinfo[0]) ##repeat accumulation process of grey-scale values for subsections
						sumOfSubsection += BWvalue
				avgSub = sumOfSubsection/(D.gridValue * D.gridValue)
				if avgSub > avg:
					for y in range(D.gridValue) :
						for x in range(D.gridValue):
							D.img[y+ycount,x+xcount] = 255 ##if the average of this square is higher than the image's average, color it all black
				else:
					for y in range(D.gridValue):
						for x in range(D.gridValue):
							D.img[y+ycount,x+xcount] = 0 ##if the average of this square is lower than the image's average, color it all white
				xcount += D.gridValue
				sumOfSubsection = 0
			ycount += D.gridValue
			xcount = 0

		n = 0
		a = 0

		D.imgBinaryArray = [[0 for v in xrange(D.columns)] for w in xrange(D.rows)] ##initiating two-dimensional array that will hold 0s and 1s
		binaryFromImageList = [26,12,31] ##initiating the array that will hold the 0s and 1s before dumping them in ^the two-dimensional one...use your brain to make this one step and cut out this middle man
		##CONSIDER: binaryFromImageList = [0 for i in xrange(traversalInt)] ##initiating the array that will hold the 0s and 1s before dumping them in ^the two-dimensional one...use your brain to make this one step and cut out this middle man

		#traverse the middle values of the BP image and add a 1 or a 0 to the imgList as appropriate
		for y in xrange(D.rows):
			for x in xrange(D.columns):
				middleY = (D.gridValue/2)+(D.gridValue*y) ## the grids are 20 x 20, and we are looking for the middle value at multiples of 10 x 10 (gridValue/2)..we could have looked at a corner, but I chose the middle
				middleX = (D.gridValue/2)+(D.gridValue*x)
				
				if ((middleX < width) and (middleY < length)):
					px = D.img[middleY,middleX]
					pxInfo = str(px).strip('[]')
					splitPXinfo = pxInfo.split(" ")
					BWvalue = int(splitPXinfo[0])
				
					if (BWvalue == 0):
						binaryFromImageList.insert(D.n, 0) ##add 0 to "list"
					if (BWvalue == 255):
						binaryFromImageList.insert(D.n, 1) ##add 1 to "list"
					D.n += 1
				else:
					break

		#move the imgList contents to a two dimensionalArray
		for b in xrange(D.rows):
			for c in xrange(D.columns):
				D.imgBinaryArray[b][c] =  int(binaryFromImageList[a])
				a += 1
				if (a >= n):
					break


	def matchIndexing(self):
		##from the freshly generated ("imgs...BP") generates two-dimensional binary array
		##compares the binary array to the saved mapBinaryArray (D.mapArrayTxt) and generates a third 2-D binary array with values representing likeness scores
		##abcdefghijk  m mv n opqr tuvwxy z <-- variables used
		D.n = 0
		a = 0
		matchingIndex = 0 ##holds the number of matches from each time the image is compared to a part of the map

		imgToBinary = cv2.imread(D.imageNameBP) ##capture image's BP
		length, width, depth = imgToBinary.shape
		traversalInt = D.columns * D.rows ##total squares in the BPimage 

		imgBinaryArray = [[0 for v in xrange(D.columns)] for w in xrange(D.rows)] ##initiating two-dimensional array that will hold 0s and 1s
		binaryFromImageList = [26,12,31] ##initiating the array that will hold the 0s and 1s before dumping them in ^the two-dimensional one...use your brain to make this one step and cut out this middle man
		##CONSIDER: binaryFromImageList = [0 for i in xrange(traversalInt)] ##initiating the array that will hold the 0s and 1s before dumping them in ^the two-dimensional one...use your brain to make this one step and cut out this middle man

		#traverse the middle values of the BP image and add a 1 or a 0 to the imgList as appropriate
		for y in xrange(D.rows):
			for x in xrange(D.columns):
				middleY = (D.gridValue/2)+(D.gridValue*y) ## the grids are 20 x 20, and we are looking for the middle value at multiples of 10 x 10 (gridValue/2)..we could have looked at a corner, but I chose the middle
				middleX = (D.gridValue/2)+(D.gridValue*x)
				
				if ((middleX < width) and (middleY < length)):
					px = imgToBinary[middleY,middleX]
					pxInfo = str(px).strip('[]')
					splitPXinfo = pxInfo.split(" ")
					BWvalue = int(splitPXinfo[0])
				
					if (BWvalue == 0):
						binaryFromImageList.insert(D.n, 0) ##add 0 to "list" at position D.n
					if (BWvalue == 255):
						binaryFromImageList.insert(D.n, 1) ##add 1 to "list" at position D.n
					D.n += 1
				else:
					break

		#move the imgList contents to a two dimensionalArray
		for b in xrange(D.rows):
			for c in xrange(D.columns):
				imgBinaryArray[b][c] =  int(binaryFromImageList[a])
				a += 1
				if (a >= D.n):
					break
		#save two dimensional array to a file
		D.binArrayTxt = 'imgs/imgBinaryArray' + str(D.imgNum) + 'BBP.txt'
		np.savetxt(D.binArrayTxt, imgBinaryArray, fmt='%d')

		D.n = 0

		imgArrayWidth = D.columns
		imgArrayLength = D.rows

		mapArray = open(D.mapArrayTxtbBP)
		mapArraySize = os.path.getsize(D.mapArrayTxtbBP)
		mapArrayWithSpaces = len(mapArray.readline())
		mapArrayWidth = mapArrayWithSpaces/2
		mapArrayLength = mapArraySize/mapArrayWithSpaces

		newMapArray = [[0 for t in xrange(mapArrayWidth)] for u in xrange(mapArrayLength)]

		mapArray = open(D.mapArrayTxtbBP)

		D.n = 0
		
		#save to two dimensional array
		for f in xrange(mapArrayLength):
			for g in xrange(mapArrayWidth):
				mapArray.seek(D.n)
				newMapArray[f][g] = mapArray.read(1)
				D.n += 2

		verticalBound = mapArrayLength-imgArrayLength
		horizontalBound = mapArrayWidth - imgArrayWidth
		matchingScoreArray = [[0 for i in xrange(horizontalBound+1)] for h in xrange(verticalBound+1)] ##this array will hold the scores

		m = 0
		mv = 0

		#compute matching indexes and load onto array
		while (mv <= verticalBound):
			while (m <= horizontalBound):
				for j in xrange(imgArrayLength):
					for k in xrange(imgArrayWidth):
						mapValue = newMapArray[j+mv][k+m]
						stripMapValue = mapValue.strip()
						intMapValue = int(stripMapValue)
						matchingIndex += abs(imgBinaryArray[j][k] - intMapValue) ##if they are different, the index will increase by one
				matchingScoreArray[mv][m] = matchingIndex
				matchingIndex = 0
				m += 1
			m = 0
			mv += 1
		##save the matching array
		matchingScoreTxt = 'imgs/matchingScoreArray' + str(D.imgNum) + 'BBP.txt'
		np.savetxt(matchingScoreTxt, matchingScoreArray, fmt='%3d') ##the 3 in '%3d' signals that the numbers must have at least three digits, important when reading numbers below and over 100
		D.lengthOfScores = 3
		D.matchingScoresTxt = matchingScoreTxt
		self.plotMostLikely()


	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##////////////////////////////////////////////////////////////////////////FANCY BP ICON//////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	def fancyBrightnessPattern(self): ##calls generateFancyBP and fancyMatching
		##uses imageNameABW, saves imageNameFBP
		self.statusBar().showMessage('Generating monochromatic brightness pattern')
				
		##converted to BW when it was resized,send it to be fancyBPed
		self.generateMonochromaticBrightnessPattern(D.imageNameABW) ## does not need to be global because it is being sent explicitly, returns D.img
		
		##function returns a changed D.imgF, save it here
		D.imageNameFBP = "imgs/last" + str(D.imgNum) + "MBP.png"
		cv2.imwrite(D.imageNameFBP,D.imgF) ##opened with cv2.imread, saved with cv2.imwrite
		
		#because generateMonochromaticBrightnessPattern is a shared function, save the txt version here (TD does not need it saved)
		numArrayTxt = 'imgs/imgMonochromaticArray' + str(D.imgNum) + 'MBP.txt'
		np.savetxt(numArrayTxt, D.imgValuesArray, fmt='%3d')
		
		##call cameraFeed to refresh img
		D.displayImage = D.imageNameFBP
		self.cameraFeedPanel()## call cameraFeed to hide liveFeed and render our resized BP image whose name is now accessable with filename

		self.fancyMatchIndexing() ##BP not displayed until after this is closed..and thats not ok


	def fancyMatchIndexing(self):
		D.n = 0
		fancyMatchingIndex = 0

		imgArrayWidth = D.columns ##same values but the variable names aren't omni-descriptive
		imgArrayLength = D.rows

		mapArray = open(D.mapArrayTxtmBP)
		mapArraySize = os.path.getsize(D.mapArrayTxtmBP)
		mapArrayWithSpaces = len(mapArray.readline())
		mapArrayWidth = mapArrayWithSpaces/4
		mapArrayLength = mapArraySize/mapArrayWithSpaces

		newMapArray = [[0 for t in xrange(mapArrayWidth)] for u in xrange(mapArrayLength)]
		mapArray = open(D.mapArrayTxtmBP)

		for f in xrange(mapArrayLength):
			for g in xrange(mapArrayWidth):
				mapArray.seek(D.n)
				newMapArray[f][g] = mapArray.read(3)
				D.n += 4

		verticalBound = mapArrayLength-imgArrayLength
		horizontalBound = mapArrayWidth - imgArrayWidth
		fancyMatchingScoreArray = [[0 for i in xrange(horizontalBound+1)] for h in xrange(verticalBound+1)] ##this array will hold the scores

		m = 0
		mv = 0
		#compute matching indexes and load onto array
		while (mv <= verticalBound):
			while (m <= horizontalBound):
				for j in xrange(imgArrayLength):
					for k in xrange(imgArrayWidth):
						mapValue = newMapArray[j+mv][k+m]
						stripMapValue = mapValue.strip()
						intMapValue = int(stripMapValue)
						fancyMatchingIndex += abs(D.imgValuesArray[j][k] - intMapValue)
				fancyMatchingScoreArray[mv][m] = fancyMatchingIndex
				fancyMatchingIndex = 0
				m += 1
			m = 0
			mv += 1

		##save the matching array
		fancyMatchingScoreTxt = 'imgs/matchingScoreArray' + str(D.imgNum) + 'MBP.txt'
		np.savetxt(fancyMatchingScoreTxt, fancyMatchingScoreArray, fmt='%5d')
		D.lengthOfScores = 5
		D.matchingScoresTxt = fancyMatchingScoreTxt
		self.plotMostLikely()


	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##////////////////////////////////////////////////////////////////////////FALLING PIXELS ICON////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	def topDownBP(self): ##calls generateBP and matching
		##uses imageNameABW, saves imageNameTD
		self.statusBar().showMessage('Generating binary brightness pattern using top-down approach')
		
		##converted to BW when it was resized ( ABW vvvvvvvvvvvvv ), send it to be BPed
		self.generateMonochromaticBrightnessPattern(D.imageNameABW) ## does not need to be global because it is being sent explicitly, returns D.img ##does not need to be sent because it is global...
		##returns a FBPed image and a text file representing its values

		##use the returned text file to do TD analysis, generate a binary array of results, save, and recolorize image accordingly
		self.analyzeTDandTXT()

		#save recolored image
		D.imageNameTD = 'imgs/last' + str(D.imgNum) + 'BTD.png'
		cv2.imwrite(D.imageNameTD,D.imgF)

		#save txt file representing recolored image
		imgBTDTxt = 'imgs/imgBinaryArray' + str(D.imgNum) + 'BTD.txt'
		np.savetxt(imgBTDTxt, D.tdImgBinaryArray, fmt='%d')

		D.displayImage = D.imageNameTD
		self.cameraFeedPanel()

		self.matchIndexingTD() ##BP not displayed until after this is closed..and thats not ok


	def analyzeTDandTXT(self): ##uses D.imgValuesArray from FBPgenerator to do a top-down analysis of the incoming image and return a binaryTD image and accompanying text file
		tdYblocks = D.rows - 1 ##expecting 25
		#print "tdYblocks: ", tdYblocks
		traversalInt = D.columns * tdYblocks ##total squares in the BPimage 

		D.tdImgBinaryArray = [[0 for v in xrange(D.columns)] for w in xrange(tdYblocks)] ##initiating two-dimensional array that will hold 0s and 1s
		binaryFromImageList = [0 for i in xrange(traversalInt)] ##initiating the array that will hold the 0s and 1s before dumping them in ^the two-dimensional one...use your brain to make this one step and cut out this middle man

		D.n = 0
		#traverse the numeric array values of the monochromatic BP image, compare to value above it and add a 1 or a 0 to the imgList as appropriate
		for ys in xrange(tdYblocks):
			for xs in xrange(D.columns):
				topBWvalue = D.imgValuesArray[ys][xs]
				BWvalue = D.imgValuesArray[ys+1][xs]
				if (BWvalue > topBWvalue): ##255 is white and 0 is black
					binaryFromImageList.insert(D.n, 1) ##add 1 to "list" to signify top value is darker
				if (BWvalue <= topBWvalue):
					binaryFromImageList.insert(D.n, 0) ##add 0 to "list" to signify top value is lighter (or the same)
				D.n += 1

		a = 0
		#move the imgList contents to a two dimensionalArray
		for bd in xrange(tdYblocks):
			for cd in xrange(D.columns):
				D.tdImgBinaryArray[bd][cd] =  int(binaryFromImageList[a])
				a += 1
				if (a >= D.n): ##n should be less than traversal int
					break
		
		xcount = 0
		ycount = D.gridValue
		#recolor
		for yb in range(tdYblocks) : #25
			for xz in range(D.columns): #79
				recolorValue = D.tdImgBinaryArray[yb][xz]
				#recolor image according to tdIMgBinaryArray
				for yf in range(D.gridValue): ##0 to 49 @ 1250, 1250, max in 1299, 1299
					for xf in range(D.gridValue):
						if (recolorValue == 0):
							D.imgF[(yf+ycount),(xf+xcount)] = 0 
						else:
							D.imgF[(yf+ycount),(xf+xcount)] = 255
				xcount += D.gridValue
			ycount += D.gridValue
			xcount = 0


	def matchIndexingTD(self):	
		D.n = 0
		matchingIndex = 0 ##holds the number of matches from each time the image is compared to a part of the map

		imgArrayWidth = D.columns
		imgArrayLength = D.rows

		mapArray = open(D.mapArrayTxtbTD)
		mapArraySize = os.path.getsize(D.mapArrayTxtbTD)
		mapArrayWithSpaces = len(mapArray.readline())
		mapArrayWidth = mapArrayWithSpaces/2
		mapArrayLength = mapArraySize/mapArrayWithSpaces

		newMapArray = [[0 for ttd in xrange(mapArrayWidth)] for utd in xrange(mapArrayLength)]

		mapArray = open(D.mapArrayTxtbTD)

		D.n = 0

		for ftd in xrange(mapArrayLength):
			for gtd in xrange(mapArrayWidth):
				mapArray.seek(D.n)
				newMapArray[ftd][gtd] = mapArray.read(1)
				D.n += 2

		verticalBound = mapArrayLength-imgArrayLength
		horizontalBound = mapArrayWidth - imgArrayWidth
		matchingScoreArray = [[0 for itd in xrange(horizontalBound+1)] for htd in xrange(verticalBound+1)] ##this array will hold the scores

		m = 0
		mv = 0
		#calculate matching index,, load onto array
		while (mv <= verticalBound):
			while (m <= horizontalBound):
				for jtd in xrange(imgArrayLength-1): #0 to 10
					for ktd in xrange(imgArrayWidth): ##0 to 15
						mapValue = newMapArray[jtd+mv][ktd+m]
						stripMapValue = mapValue.strip()
						intMapValue = int(stripMapValue)
						matchingIndex += abs(D.tdImgBinaryArray[jtd][ktd] - intMapValue)
				matchingScoreArray[mv][m] = matchingIndex
				matchingIndex = 0
				m += 1
			m = 0
			mv += 1

		##save the matching array
		matchingScoreTDTxt = 'imgs/matchingScoreArray' + str(D.imgNum) + 'BTD.txt'
		np.savetxt(matchingScoreTDTxt, matchingScoreArray, fmt='%3d') ##the 3 in '%3d' signals that the numbers must have at least three digits, important when reading numbers below and over 100
		D.lengthOfScores = 3
		D.matchingScoresTxt = matchingScoreTDTxt
		self.plotMostLikely()


	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##/////////////////////////////////////////////////////////////////////////TOP-DOWN FANCY ICON///////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	def fancyTopDownBP(self): ##calls generateFancyBP and fancyMatching
		#uses imageNameABW, saves imageNameFBP
		self.statusBar().showMessage('Generating monochromatic brightness pattern using top-down apprach')
		#print "Will analyze a monochromatic BP using a numeric top-down approach"
		
		##converted to BW when it was resized,send it to be fancyBPed
		self.generateMonochromaticBrightnessPattern(D.imageNameABW) ## does not need to be global because it is being sent explicitly, returns D.img ##'LL6last1aBW.png')
		##function returns a changed D.imgF and txt file representing its values
		
		##use the returned text file to do TD analysis, generate a binary array of results, and recolorize		
		self.analyzeFTDandTXT()

		D.imageNameMTD = 'imgs/last' + str(D.imgNum) + 'MTD.png'
		cv2.imwrite(D.imageNameMTD,D.imgF)

		##save txt file here
		imgMTDTxt = 'imgs/imgMonochromaticArray' + str(D.imgNum) + 'MTD.txt'
		np.savetxt(imgMTDTxt, D.ftdImgBinaryArray, fmt='%3d')

		D.displayImage = D.imageNameMTD
		self.cameraFeedPanel()

		self.fancyMatchIndexingTD() ##BP not displayed until after this is closed..and thats not ok


	def analyzeFTDandTXT(self):
		tdYblocks = D.rows - 1 ##expecting 25
		traversalInt = D.columns * tdYblocks ##total squares in the BPimage 

		D.ftdImgBinaryArray = [[0 for v in xrange(D.columns)] for w in xrange(tdYblocks)] ##initiating two-dimensional array that will hold 0s and 1s
		binaryFromImageList = [0 for i in xrange(traversalInt)] ##initiating the array that will hold the 0s and 1s before dumping them in ^the two-dimensional one...use your brain to make this one step and cut out this middle man

		D.n = 0
		#traverse the numeric array values of the monochromatic BP image, compare to value above it and add a 1 or a 0 to the imgList as appropriate
		for ys in xrange(tdYblocks):
			for xs in xrange(D.columns):
				topBWvalue = D.imgValuesArray[ys][xs]
				BWvalue = D.imgValuesArray[ys+1][xs]
				score = ((topBWvalue - BWvalue)/2) + 128
				binaryFromImageList.insert(D.n, score) ##add 0 to "list" to signify top value is lighter (or the same)
				D.n += 1
		
		a = 0
		#move the imgList contents to a two dimensionalArray
		for bd in xrange(tdYblocks):
			for cd in xrange(D.columns):
				D.ftdImgBinaryArray[bd][cd] =  int(binaryFromImageList[a])
				a += 1
				if (a >= D.n): ##n should be less than traversal int
					break

		xcount = 0
		ycount = D.gridValue
		for yb in range(tdYblocks) : #25
			for xz in range(D.columns): #79
				recolorValue = D.ftdImgBinaryArray[yb][xz]
				#recolor image according to tdMapBinaryArray
				for yf in range(D.gridValue): ##0 to 49 @ 1250, 1250, max in 1299, 1299
					for xf in range(D.gridValue):
						D.imgF[(yf+ycount),(xf+xcount)] = recolorValue
				xcount += D.gridValue
			ycount += D.gridValue
			xcount = 0


	def fancyMatchIndexingTD(self):	
		D.n = 0
		matchingIndex = 0 ##holds the number of matches from each time the image is compared to a part of the map

		imgArrayWidth = D.columns
		imgArrayLength = D.rows

		mapArray = open(D.mapArrayTxtmTD)
		mapArraySize = os.path.getsize(D.mapArrayTxtmTD)
		mapArrayWithSpaces = len(mapArray.readline())
		mapArrayWidth = mapArrayWithSpaces/4
		mapArrayLength = mapArraySize/mapArrayWithSpaces

		newMapArray = [[0 for ttd in xrange(mapArrayWidth)] for utd in xrange(mapArrayLength)]

		mapArray = open(D.mapArrayTxtmTD)

		D.n = 0

		for ftd in xrange(mapArrayLength):
			for gtd in xrange(mapArrayWidth):
				mapArray.seek(D.n)
				newMapArray[ftd][gtd] = mapArray.read(3)
				D.n += 4

		verticalBound = mapArrayLength-imgArrayLength 
		horizontalBound = mapArrayWidth - imgArrayWidth
		matchingScoreArray = [[0 for itd in xrange(horizontalBound+1)] for htd in xrange(verticalBound+1)] ##this array will hold the scores

		m = 0
		mv = 0

		while (mv <= verticalBound):
			while (m <= horizontalBound):
				for jtd in xrange(imgArrayLength-1): #0 to 10
					for ktd in xrange(imgArrayWidth): ##0 to 15
						mapValue = newMapArray[jtd+mv][ktd+m]
						stripMapValue = mapValue.strip()
						if (stripMapValue == ''):
							break
						intMapValue = int(stripMapValue)
						matchingIndex += abs(D.ftdImgBinaryArray[jtd][ktd] - intMapValue)
				matchingScoreArray[mv][m] = matchingIndex
				matchingIndex = 0
				m += 1
			m = 0
			mv += 1

		##save the matching array
		matchingScoreTDTxt = 'imgs/matchingScoreArray' + str(D.imgNum) + 'MTD.txt'
		np.savetxt(matchingScoreTDTxt, matchingScoreArray, fmt='%4d') ##the 4 in '%4d' signals that the numbers must have at least three digits, important when reading numbers below and over 100
		D.lengthOfScores = 4
		D.matchingScoresTxt = matchingScoreTDTxt
		self.plotMostLikely()
	

	def generateMonochromaticBrightnessPattern(self,imageName): ##uses the incoming BW image and returns the incoming image's BP and a text file that represents the sub-section's grey-scale values
		sumOfValues = 0 ## accumulator for grey-scale values of each pixel in BW version of captured image
		sumOfSubsection = 0 ## accumulator for grey-scale values of each pixel in 20 x 20 sections of BW version of captured image
		n = 0

		#the list where the up-to-three digit values will be stored
		imageValuesList = [826, 826, 826]

		D.imgF = cv2.imread(imageName) ## open the ^ just-saved BWthat your function is being sent
		length, width, depth = D.imgF.shape		 ## length, width, color channels (we just re-seized, we should know the size..)
		D.xBlocks = width/D.gridValue ##columns of squares in the BPimage ##16 when length is 320 and grids are 20
		D.rows = length/D.gridValue		
		
		##using D.gridValue/D.gridValue, divide the image into subsections
		##the grid loop to traverse 20 x 20 sections
		xcount = 0
		ycount = 0
		numOfColumns = (width/D.gridValue)
		
		while(((xcount + D.gridValue) <= width) and (ycount + D.gridValue <= length)):
			for a in range(numOfColumns):
				for y in range(D.gridValue) :
					for x in range(D.gridValue):
						px = D.imgF[(y+ycount),(x+xcount)]
						pxInfo = str(px).strip('[]')
						splitPXinfo = pxInfo.split(" ")
						BWvalue = int(splitPXinfo[0]) ##repeat accumulation process of grey-scale values for subsections
						sumOfSubsection += BWvalue
				avgSub = sumOfSubsection/(D.gridValue * D.gridValue)

				##add average sub to the list
				imageValuesList.insert(n, avgSub)
				n += 1

				for y in range(D.gridValue) :
					for x in range(D.gridValue):
						D.imgF[y+ycount,x+xcount] = avgSub ##if the average of this square is higher than the image's average, color it all black
				
				xcount += D.gridValue
				sumOfSubsection = 0
			ycount += D.gridValue
			xcount = 0

		a = 0
		##copy list onto two-dimensional array
		D.imgValuesArray = [[0 for v in xrange(D.columns)] for w in xrange(D.rows)]
		for b in xrange(D.rows):
			for c in xrange(D.columns):
				D.imgValuesArray[b][c] =  int(imageValuesList[a])
				a += 1
				if (a >= n):
					break

	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////RGB ICON////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	##///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////##
	def bgrBP(self): ##calls __ and  ___
		#uses __ saves __
		self.statusBar().showMessage('Generating RGB pattern')
		##use resized, not BW image
		img = cv2.imread(D.imageNameA)
		length, width, depth = img.shape ## length, width, color channels (we just re-seized, we should know the size..)

		sumOfR = 0
		sumOfG = 0
		sumOfB = 0
		
		xcount = 0
		ycount = 0
		n = 0
		x = 0
		numOfColumns = (width/D.gridValue)
		#recolor
		while(((xcount + D.gridValue) <= width) and (ycount + D.gridValue <= length)):
			for a in range(numOfColumns):
				for y in range(D.gridValue) :
					for x in range(D.gridValue):
						px = img[(y+ycount),(x+xcount)]
						pxInfo = str(px).strip('[]')
						splitPXinfo = pxInfo.split(" ")
						if (splitPXinfo[n] == ''):
							n += 1
						if (splitPXinfo[n] == ''):
							n += 1
						Bvalue = int(splitPXinfo[n])
						if (splitPXinfo[n+1] == ''):
							n += 1 
						if (splitPXinfo[n+1] == ''):
							n += 1 
						Gvalue = int(splitPXinfo[n+1])
						if (splitPXinfo[n+2] == ''):
							n += 1
						if (splitPXinfo[n+2] == ''):
							n += 1
						Rvalue = int(splitPXinfo[n+2])
						n = 0
						sumOfR += Rvalue
						sumOfG += Gvalue
						sumOfB += Bvalue
				avgRsub = sumOfR/(D.gridValue * D.gridValue)
				avgGsub = sumOfG/(D.gridValue * D.gridValue)
				avgBsub = sumOfB/(D.gridValue * D.gridValue)

				for y in range(D.gridValue) :
					for x in range(D.gridValue):
						img[y+ycount,x+xcount] = [avgBsub, avgGsub, avgRsub] #bgr rather than RGB?
				xcount += D.gridValue
				sumOfR = 0
				sumOfG = 0
				sumOfB = 0
			ycount += D.gridValue
			xcount = 0

		bgrbpName = 'imgs/last' + str(D.imgNum) + 'RGBBP.jpg'
		cv2.imwrite(bgrbpName, img) ##save bgrBP

		##saving three numbers to a list
		columns = width/D.gridValue #width(columns)
		rows = length/D.gridValue 

		##actual two-D array..perhaps a 3d array would work?
		arrayBBP = [[0 for v in xrange(columns*3)] for w in xrange(rows)]

		traversalInt = columns * rows
		#print traversalInt #2236
		imageArray = [26,12,31] ##list of zeros and ones generated from image
		#record values to list
		count = 0
		z = 0
		for y in xrange(rows):
			for x in xrange(columns):
				middleY = (D.gridValue/2)+(D.gridValue*y)
				middleX = (D.gridValue/2)+(D.gridValue*x)
				
				if ((middleX < width) and (middleY < length)):
					px = img[middleY,middleX]
					pxInfo = str(px).strip('[]')
					splitPXinfo = pxInfo.split(" ")
					if (splitPXinfo[n] == ''):
						n += 1
					if (splitPXinfo[n] == ''):
						n += 1
					Bvalue = int(splitPXinfo[n])
					if (splitPXinfo[n+1] == ''):
						n += 1 
					if (splitPXinfo[n+1] == ''):
						n += 1 
					Gvalue = int(splitPXinfo[n+1])
					if (splitPXinfo[n+2] == ''):
						n += 1
					if (splitPXinfo[n+2] == ''):
						n += 1
					Rvalue = int(splitPXinfo[n+2])
					imageArray.insert(z, Bvalue)
					imageArray.insert(z+1, Gvalue)
					imageArray.insert(z+2, Rvalue)
					n = 0
					z += 3
				else:
					break
		a = 0
		#save to two-dimensional array
		D.arrayOne = [[0 for v in xrange(columns*3)] for w in xrange(rows)] ##instantiating?
		for b in xrange(rows):
			for c in xrange(columns*3):
				D.arrayOne[b][c] =  int(imageArray[a])##overwriting
				a += 1
				if (a >= z):
					break

		bgrbpTxt = 'imgs/last' + str(D.imgNum) + 'RGBBParray.txt'
		np.savetxt(bgrbpTxt, D.arrayOne, fmt='%3d') ##because the max is three digit numbers

		D.displayImage = bgrbpName
		self.cameraFeedPanel()

		self.rgbMatchIndexing()

		
	def rgbMatchIndexing(self):	
		D.n = 0
		matchingIndex = 0 ##holds the number of matches from each time the image is compared to a part of the map

		imgArrayWidth = D.columns
		imgArrayLength = D.rows

		mapArray = open(D.mapRGBArray)
		mapArraySize = os.path.getsize(D.mapRGBArray)
		mapArrayWithSpaces = len(mapArray.readline())
		mapArrayWidth = mapArrayWithSpaces/2
		mapArrayLength = mapArraySize/mapArrayWithSpaces

		newMapArray = [[0 for ttd in xrange(mapArrayWidth)] for utd in xrange(mapArrayLength)]

		mapArray = open(D.mapRGBArray)

		#read the saved, map text file
		for ftd in xrange(mapArrayLength):
			for gtd in xrange(mapArrayWidth):
				mapArray.seek(D.n)
				newMapArray[ftd][gtd] = mapArray.read(3)
				D.n += 4

		verticalBound = mapArrayLength-imgArrayLength 
		horizontalBound = (mapArrayWidth/3) - imgArrayWidth
		matchingScoreArray = [[0 for itd in xrange(horizontalBound+1)] for htd in xrange(verticalBound+1)] ##this array will hold the scores

		m = 0
		mv = 0

		while (mv <= verticalBound):
			while (m <= horizontalBound):
				for jtd in xrange(imgArrayLength):
					for ktd in xrange(imgArrayWidth):
						mapValue = newMapArray[jtd+mv][ktd+m]
						stripMapValue = mapValue.strip()
						if (stripMapValue == ''):
							break
						intMapValue = int(stripMapValue)
						matchingIndex += abs(D.arrayOne[jtd][ktd] - intMapValue)
				matchingScoreArray[mv][m] = matchingIndex
				matchingIndex = 0
				m += 1
			m = 0
			mv += 1

		##save the matching array
		matchingScoreRGBtxt = 'imgs/matchingScoreArray' + str(D.imgNum) + 'RGB.txt'
		np.savetxt(matchingScoreRGBtxt, matchingScoreArray, fmt='%5d') ##the 4 in '%4d' signals that the numbers must have at least three digits, important when reading numbers below and over 100
		D.lengthOfScores = 5
		D.matchingScoresTxt = matchingScoreRGBtxt
		self.plotMostLikely()


	def plotMostLikely(self):
		#this is the map that will be printed to in the new window that pops up
		imgI = array(Image.open(D.colorMap))
		plt = figure(figsize=(24,7.5))
		imshow(imgI)
		axis('off')

		bpmap = cv2.imread(D.colorMap)

		length, width, depth = bpmap.shape
		xSquaresInImg = D.imgX/D.gridValue
		xSquaresInMap = width/D.mapGridValue
		var1 = xSquaresInMap - xSquaresInImg 
		ySquaresInImg = D.imgY/D.gridValue
		ySquaresInMap = length/D.mapGridValue
		var2 = ySquaresInMap - ySquaresInImg  
		var3 = var1*var2

		x = [0 for o in xrange(var3)]
		y = [0 for p in xrange(var3)]
		m = 0
		#generating x and y arrays that correspond to the coordinates of points that represent the center of a captured image in relation to the map
		for qtd in xrange(var2):
			for rtd in xrange(var1):
				middleY = (D.mapGridValue/2)+(D.mapGridValue*qtd) + ((ySquaresInImg/2)*D.mapGridValue)
				middleX = (D.mapGridValue/2)+(D.mapGridValue*rtd) + ((xSquaresInImg/2)*D.mapGridValue)
				
				if ((middleX < width) and (middleY < length) and m < var3):
					x[m] = middleX
					y[m] = middleY
					m += 1
				else:
					break
		D.n = 0
		w = 0
		smallest = 100000
		largest = 0
		smallestX = 0
		smallestY = 0
		matchingArraySize = os.path.getsize(D.matchingScoresTxt)

		imgArray = open(D.matchingScoresTxt)
		for z in xrange(var3):

			imgArray.seek(D.n)
			uneditedScore = imgArray.read(D.lengthOfScores) ##read the next lengthOfScores characters together
			if (D.n < matchingArraySize):
				D.n += (D.lengthOfScores + 1) ##move ahead (lengthOfScores+1) spaces (the lengthOfScores characters plus the separating space)
			strippedScore = uneditedScore.strip(' ') ##for numbers less than the min number lengthOfScores long, drop the leading space before converting to an int
			intScore = int(strippedScore)
			if (intScore < smallest):
				smallest = intScore
				smallestX = x[z]
				smallestY = y[z]
			if (intScore > largest):
				largest = intScore

		rangeOfScores = largest - smallest
		interval = rangeOfScores/6
		boundaryList = [(smallest+interval), (smallest+(interval*2)), (smallest+(interval*3)), (smallest+(interval*4)), (smallest+(interval*5))] ##these values decide the color cut offs	

		D.n = 0
		imgArray = open(D.matchingScoresTxt)
		for zz in xrange(var3):

			imgArray.seek(D.n)

			uneditedScore = imgArray.read(D.lengthOfScores) ##read the next three characters together
			if (D.n < matchingArraySize):
				D.n += (D.lengthOfScores + 1) ##move ahead four spaces (the three characters plus the separating space)
			
			if (uneditedScore == ''):
				break
			else:
				strippedScore = uneditedScore.strip(' ') ##for numbers less than min of lengthofScores digits, drop the leading space before converting to an int
				intScore = int(strippedScore)

				#at the given coordinates, the lowest scores are painted red, [next highest: magenta, next highest: blue, next highest: cyan, next highest: green], highest: yellow
				#low means lots of likelihood
				if(intScore < boundaryList[0]):
					plot(x[w],y[w],'r.')
				elif(intScore < boundaryList[1]):
					plot(x[w],y[w],'m.')
				elif(intScore < boundaryList[2]):
					plot(x[w],y[w],'b.')
				elif(intScore < boundaryList[3]):
					plot(x[w],y[w],'c.')
				elif(intScore < boundaryList[4]):
					plot(x[w],y[w],'g.')
				else:
					plot(x[w],y[w],'y.')
				w += 1

		mostLikelyX = [(smallestX - ((D.columns/2)*D.mapGridValue)), (smallestX + (D.columns/2)*D.mapGridValue), (smallestX + (D.columns/2)*D.mapGridValue), (smallestX - (D.columns/2)*D.mapGridValue), (smallestX - ((D.columns/2)*D.mapGridValue))]
		mostLikelyY = [(smallestY - ((D.rows/2)*D.mapGridValue)), (smallestY - (D.rows/2)*D.mapGridValue), (smallestY + (D.rows/2)*D.mapGridValue), (smallestY + (D.rows/2)*D.mapGridValue), (smallestY - ((D.rows/2)*D.mapGridValue))]

		plot(mostLikelyX[:],mostLikelyY[:], 'r')
		##consider replacing with: img = cv2.rectangle(img,(384,0),(510,128),(0,255,0),3)s #//green rectangle at top right rectangle of image

		indentation = width - 250
		#cv2.getTextSize(text, fontFace, fontScale, thickness)
		#cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
		cv2.putText(imgI,"Highest", (indentation,1000), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,0,0), 4) 
		cv2.putText(imgI,"Higher", (indentation,1050), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,0,255), 4)
		cv2.putText(imgI,"High", (indentation,1100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 4) 
		cv2.putText(imgI,"Low", (indentation,1150), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,255), 4) 
		cv2.putText(imgI,"Lower", (indentation,1200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 4) 
		cv2.putText(imgI,"Lowest", (indentation,1250), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)

		##function returns a changed D.img, save it here
		D.imageMostLikely = "imgs/last" + str(D.imgNum) + "mostLikely.png"
		plt.savefig(D.imageMostLikely, dpi=100)#,imgI)

		##crop the center
		pilImg = Image.open(D.imageMostLikely)
		box = (600, 180, 1950, 600) ##from (600, 180) to (1950, 600)
		cropBox = pilImg.crop(box)
		resultsImg = 'imgs/croppedResults' + str(D.imgNum) + '.jpg'
		cropBox.save(resultsImg)

		##call sixth tab with this new info
		#D.resultMap = resultsImg
		#self.mapPanel()

		show() ##opens the window with the color map and the color rhombuses

	##resizes to D.imgX, D.imgY
	def resizeCameraShot(self): ##also generates BW image, since three of our buttons need it
		pilImg = Image.open(D.originalImage)
		smaller = pilImg.resize((D.imgX, D.imgY))
		D.columns = D.imgX/D.gridValue ##columns of squares in the BPimage ##16 when length is 320 and grids are 20
		D.rows = D.imgY/D.gridValue ##rows of squares in the BPimage ##12 when width is 240 and grids are 20
		D.imageNameA = "imgs/last" + str(D.imgNum) + "a.png"
		smaller.save(D.imageNameA)

		imgBW = Image.open(D.imageNameA).convert('L')
		gray()
		D.imageNameABW = "imgs/last" + str(D.imgNum) + "aBW.png"
		imgBW.save(D.imageNameABW)

	#functions for the launcher that all call sendCodeToLauncher
	def moveUp (self):
		self.sendCodeToLauncher("up")
		self.statusBar().showMessage('Up was pressed')

	def moveDown (self):
		self.sendCodeToLauncher("down")
		self.statusBar().showMessage('Down was pressed')

	def moveLeft (self):
		self.sendCodeToLauncher("left")
		self.statusBar().showMessage('Left was pressed')

	def moveRight (self):
		self.sendCodeToLauncher("right")
		self.statusBar().showMessage('Right was pressed')

	def stopMoving (self):
		self.sendCodeToLauncher("stop")
		self.statusBar().showMessage('Stop was pressed')

	def sendCodeToLauncher(self, directive):
		if directive == "up": code = [0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00]
		elif directive == "down": code = [0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00]
		elif directive == "left": code = [0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00]
		elif directive == "right": code = [0x02,0x08,0x00,0x00,0x00,0x00,0x00,0x00]
		elif directive == "fire": code = [0x02,0x10,0x00,0x00,0x00,0x00,0x00,0x00]
		elif directive == "stop": code = [0x02,0x20,0x00,0x00,0x00,0x00,0x00,0x00]
		else:
			print "Error: unrecognized directive in sendCodeToLauncher:", directive
			code = None
		if code != None:
			self.launcher.ctrl_transfer(0x21,0x09,0,0,code)
			
	#verifies keyboard input when in keyboard mode
	def checkKeyPress(self):
		D.key_press
		##the key choices might look odd, but just put your middle finger on 'U'
		if D.key_press == ord('q') or D.key_press == 27:
			self.statusBar().showMessage('Quiting now')
			D.quiting = True
		elif D.key_press == ord('u') or D.key_press == 117:
			self.sendCodeToLauncher("up")
			self.statusBar().showMessage('Up was pressed')
		elif D.key_press == ord('k') or D.key_press == 107:
			self.sendCodeToLauncher("down")
			self.statusBar().showMessage('Down was pressed')
		elif D.key_press == ord('h')  or D.key_press == 104:
			self.sendCodeToLauncher("left")
			self.statusBar().showMessage('Left was pressed')
		elif D.key_press == ord('l')  or D.key_press == 108:
			self.sendCodeToLauncher("right")
			self.statusBar().showMessage('Right was pressed')
		elif D.key_press == ord('\n'):              ##32
			sfelf.sendCodeToLauncher("fire")
		elif D.key_press in [ord('z'), ord(' ')]:   ##122
			self.sendCodeToLauncher("stop")
			self.statusBar().showMessage('Stop was pressed')


def main():
	app = QtGui.QApplication(sys.argv) ## application object necessary
									   ##receives button data
	eighth = IrisEleventh()
	
	sys.exit(app.exec_())
	## mainloop of the application receives events from window,
	## sends to widgets..ends when exit() is called or widget
	## is destroyed. 
	## sys.exit() ensures a clean exit 
	## exec is python keyword exec_ used instead

if __name__ == '__main__':
	main()