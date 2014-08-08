##This program uses the TD approach to score squares monochromatically.
##Uses the dimensions of image fileName and gridValue to figure out time image's ideal size then resizes 
##and transforms it to grey-scale. THey grey scale is recolored monochromatically according to sub grid
##values, then a text file representing the grey-scale values is saved and used to recolor the map
##according to the monochromatic TD approach (Top-down approach: Boxes are colored to the positive difference 
##between their value and the box above it. See line ) The recolored image is saved and resized, then a text
##file representing its values is generated and saved.

import cv2
import numpy as np
from PIL import Image
import os
from pylab import *

fileName = 'mapUnedited.jpg'
mapName = 'mapPerfectSize.jpg'
fileNameBW = 'mapPerfectSizeBW.jpg'
mtdName = 'mapMTD.jpg'
mtdRzd = 'mapMTDrzd.jpg'

numArrayTxt = 'mapMonochromaticArray.txt'
mapMTDtxt = 'mapMonochromaticTD.txt'


gridValue = 50
n = 0
sumOfValues = 0 ## accumulator for grey-scale values of each pixel in BW version of captured image
sumOfSubsection = 0 ## accumulator for grey-scale values of each pixel in 20 x 20 sections of BW version of captured image

imgF = cv2.imread(fileName) ## open the ^ just-saved BWthat your function is being sent
length, width, depth = imgF.shape		 ## length, width, color channels (we just re-seized, we should know the size..)
#perfect dimensions
xBlocks = width/gridValue ##columns of squares in the BPimage ##16 when length is 320 and grids are 20
perfectWidth = xBlocks * gridValue
yBlocks = length/gridValue
perfectLength = yBlocks * gridValue

#resize & save
pilImg = Image.open(fileName)
perfectSize = pilImg.resize((perfectWidth,perfectLength)) 
perfectSize.save(mapName)
#turn BW & save
img = Image.open(mapName).convert('L')
gray()
img.save(fileNameBW)

img = cv2.imread(fileNameBW) ## open the just-saved BW
length, width, depth = img.shape ## length, width, color channels

##using D.gridValue/D.gridValue, divide the image into subsections and repaint to their average
##the grid loop to traverse 20 x 20 sections
xcount = 0
ycount = 0
numOfColumns = (width/gridValue)
numsFromImageList = [826,826,826]

while(((xcount + gridValue) <= width) and (ycount + gridValue <= length)):
	for aa in range(numOfColumns):
		for ya in range(gridValue) :
			for xa in range(gridValue):
				px = img[(ya+ycount),(xa+xcount)]
				pxInfo = str(px).strip('[]')
				splitPXinfo = pxInfo.split(" ")
				BWvalue = int(splitPXinfo[0])
				sumOfSubsection += BWvalue
		avgSub = sumOfSubsection/(gridValue * gridValue)

		numsFromImageList.insert(n, avgSub) ######################insert avgSub to the list and..
		n += 1

		for y in range(gridValue) :
			for x in range(gridValue):
				img[y+ycount,x+xcount] = avgSub ##recolor the image to each block's average ##imgF
		
		xcount += gridValue
		sumOfSubsection = 0
	ycount += gridValue
	xcount = 0

a = 0
##copy list onto two-dimensional array so that it doesnt have to happen in matching array
imgNumsArray = [[0 for v in xrange(xBlocks)] for w in xrange(yBlocks)] ##initiating two-dimensional array that will hold 0s and 1s
#move the imgList contents to a two dimensionalArray
for b in xrange(yBlocks):
	for c in xrange(xBlocks):
		imgNumsArray[b][c] =  int(numsFromImageList[a])
		a += 1
		if (a >= n):
			break
np.savetxt(numArrayTxt, imgNumsArray, fmt='%3d')

tdYblocks = yBlocks - 1 ##expecting 25
traversalInt = xBlocks * tdYblocks ##total squares in the BPimage 

tdMapBinaryArray = [[0 for v in xrange(xBlocks)] for w in xrange(tdYblocks)] ##initiating two-dimensional array that will hold 0s and 1s
binaryFromImageList = [0 for i in xrange(traversalInt)] ##initiating the array that will hold the 0s and 1s before dumping them in ^the two-dimensional one...use your brain to make this one step and cut out this middle man

n = 0
#traverse the numeric array values of the monochromatic BP image, compare to value above it and add a 1 or a 0 to the imgList as appropriate
for ys in xrange(tdYblocks):
	for xs in xrange(xBlocks):
		topBWvalue = imgNumsArray[ys][xs]
		#print "Top: (", x, ", ", y, "): ", topBWvalue
		BWvalue = imgNumsArray[ys+1][xs]
		#print "Bottom: (", x, ", ", y+1, "): ", BWvalue
		score = ((topBWvalue - BWvalue)/2) + 128
		binaryFromImageList.insert(n, score) ##add 0 to "list" to signify top value is lighter (or the same)
		n += 1
a = 0
#move the imgList contents to a two dimensionalArray
for bd in xrange(tdYblocks):
	for cd in xrange(xBlocks):
		tdMapBinaryArray[bd][cd] =  int(binaryFromImageList[a])
		a += 1
		if (a >= n): ##n should be less than traversal int
			break

imgF = cv2.imread(fileName)

xcount = 0
ycount = gridValue
for yb in range(tdYblocks) : #25
	for xz in range(xBlocks): #79
		recolorValue = tdMapBinaryArray[yb][xz]
		#recolor image according to tdMapBinaryArray
		for yf in range(gridValue): 
			for xf in range(gridValue):
				imgF[(yf+ycount),(xf+xcount)] = recolorValue
		xcount += gridValue
	ycount += gridValue
	xcount = 0



cv2.imwrite(mtdName, imgF)

np.savetxt(mapMTDtxt, tdMapBinaryArray, fmt='%3d')

#resize the finished MTD to be displayed in GUI
bbpImg = Image.open(mtdName)
smaller = bbpImg.resize((800,250))
smaller.save(mtdRzd)