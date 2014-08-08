##This program uses an image (mapName), resizes it to its perfect size according to the gridValue, and turns it black
##and white. It then generates a monochromatic brightness pattern, and a text file representing the values. The text
##file is saved and used to generate the binary Top-Down image. (Top-down approach: If the average of the gridValue*gridValue 
##box above a box is lighter, the bottom box is recolored black, otherwise white.) This recolored image is saved and resized
##to be used as the display in the GUI, its values are saved as 0s and 1s in a text file.

import cv2
import numpy as np
from PIL import Image
import os
from pylab import *

mapName = 'mapUnedited.jpg'
mapNameRzd = 'mapPerfectSize.jpg'
mapNameBW = 'mapPerfectSizeBW.jpg'
btdName = 'mapBTD.jpg'
btdRzd = 'mapBTDrzd.jpg'

numArrayTxt = 'mapMonochromaticArray.txt'
mapBTDtxt= 'mapBinaryArrayTD.txt'


gridValue = 50
n = 0
sumOfValues = 0 ## accumulator for grey-scale values of each pixel in BW version of captured image
sumOfSubsection = 0 ## accumulator for grey-scale values of each pixel in 50 x 50 sections of BW version of captured image

imgF = cv2.imread(mapName)
length, width, depth = imgF.shape ## length, width, color channels
#perfect dimensions
xBlocks = width/gridValue ##columns of squares in the BPimage 
perfectWidth = xBlocks * gridValue
yBlocks = length/gridValue
perfectLength = yBlocks * gridValue

#resize & save
pilImg = Image.open(mapName)
perfectSize = pilImg.resize((perfectWidth,perfectLength)) 
perfectSize.save(mapNameRzd)
#turn BW & save
img = Image.open(mapNameRzd).convert('L')
gray()
img.save(mapNameBW)

img = cv2.imread(mapNameBW) ## open the just-saved BW
length, width, depth = img.shape ## length, width, color channels

##using gridValue, divide the image into subsections and repaint to their average
##add average to list and later save to two-dimensional array
xcount = 0
ycount = 0
numOfColumns = (width/gridValue)
##this list will temporarily hold the monochromaticBP values of the image
numsFromImageList = [826,826,826]
##two-dimensional array that will be saved to a text file
imgNumsArray = [[0 for v in xrange(xBlocks)] for w in xrange(yBlocks)] ##initiating two-dimensional array that will hold the values that will be analyzed TD-ly

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

		numsFromImageList.insert(n, avgSub) ################insert avgSub to the list and..
		n += 1
		for y in range(gridValue) :
			for x in range(gridValue):
				img[y+ycount,x+xcount] = avgSub ##recolor the image to each block's average
		xcount += gridValue
		sumOfSubsection = 0
	ycount += gridValue
	xcount = 0
a = 0
##copy list onto two-dimensional array
for b in xrange(yBlocks):
	for c in xrange(xBlocks):
		imgNumsArray[b][c] =  int(numsFromImageList[a])
		a += 1
		if (a >= n):
			break
#save two dimensional array to a file
np.savetxt(numArrayTxt, imgNumsArray, fmt='%3d')


tdYblocks = yBlocks - 1 #because the top row doesnt undergo top-down, it is still grey-scaled
traversalInt = xBlocks * tdYblocks ##total squares in the TDimage

tdMapBinaryArray = [[0 for v in xrange(xBlocks)] for w in xrange(tdYblocks)] ##initiating two-dimensional array that will hold 0s and 1s
binaryFromImageList = [0 for i in xrange(traversalInt)] ##initiating the array that will hold the 0s and 1s before dumping them in ^the two-dimensional one...use your brain to make this one step and cut out this middle man

n = 0
#traverse the numeric array values of the monochromatic BP image, compare to value above it and add a 1 or a 0 to the imgList as appropriate
for ys in xrange(tdYblocks):
	for xs in xrange(xBlocks):
		topBWvalue = imgNumsArray[ys][xs]
		BWvalue = imgNumsArray[ys+1][xs]
		if (BWvalue > topBWvalue):
			binaryFromImageList.insert(n, 1) ##add 1 to "list" to signify top value is darker
		if (BWvalue <= topBWvalue):
			binaryFromImageList.insert(n, 0) ##add 0 to "list" to signify top value is lighter (or the same)
		n += 1
a = 0
#move the imgList contents to a two dimensionalArray
for bd in xrange(tdYblocks):
	for cd in xrange(xBlocks):
		tdMapBinaryArray[bd][cd] =  int(binaryFromImageList[a])
		a += 1
		if (a >= n):
			break

imgF = cv2.imread(mapNameRzd) ##or mapNameBW
xcount = 0
ycount = gridValue
for yb in range(tdYblocks) :
	for xz in range(xBlocks):
		recolorValue = tdMapBinaryArray[yb][xz]
		#recolor image according to tdMapBinaryArray
		for yf in range(gridValue):
			for xf in range(gridValue):
				if (recolorValue == 0): ##255 is white and 0 is black
					imgF[(yf+ycount),(xf+xcount)] = 0 
				else:
					imgF[(yf+ycount),(xf+xcount)] = 255
		xcount += gridValue
	ycount += gridValue
	xcount = 0

cv2.imwrite(btdName, imgF)

np.savetxt(mapBTDtxt, tdMapBinaryArray, fmt='%d')

#resize the finished BBP to be displayed in GUI
bbpImg = Image.open(btdName)
smaller = bbpImg.resize((800,250))
smaller.save(btdRzd)