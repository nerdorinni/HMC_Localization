##This program uses an image mapName, resizes it to its perfect size (saves it), converts it to black and
##white, and saves it to fileNameBW it then takes the BW average of the entire image and recolors gridValue
##x D.gridValue squares 255 if they are lighter than the average and 0 otherwise. It saves the recolored 
##map, generates a txt file representing the binary image, and resized the BBP to be used as the display
##map in the GUI.

from PIL import Image
from numpy import *
from scipy.ndimage import filters
from pylab import *
import os
import cv2
import numpy as np

gridValue = 50
n = 0
sumOfValues = 0
sumOfSubsection = 0

mapName = 'mapUnedited.jpg'			##original image
mapNameRzd = 'mapPerfectSize.jpg'	##resized image
mapNameBW = 'mapPerfectSizeBW.jpg'	##resized, black and white image
bbpName = 'mapBBP.jpg'				##recolored BBP image
bbpRzd = 'mapBBPrzd.jpg'			##resized, recolored BBP image

mapBBPtxt = 'mapBinaryArrayBP.txt'

img = cv2.imread(mapName)
length, width, depth = img.shape
#perfect size dimensions
largestWholeLength = length/gridValue
perfectLength = largestWholeLength*gridValue
largestWholeWidth = width/gridValue
perfectWidth = largestWholeWidth*gridValue
##resize
pilImg = Image.open(mapName)
perfectSize = pilImg.resize((perfectWidth,perfectLength)) 
perfectSize.save(mapNameRzd)
#black and white
img = Image.open(mapNameRzd).convert('L')
gray()
img.save(mapNameBW)

img = cv2.imread(mapNameBW) ## open the just-saved BW
length, width, depth = img.shape ## length, width, color channels

##reading ALL pixel values, and averaging their value
for y in xrange(length):
	for x in xrange(width):
		px = img[y,x]
		pxInfo = str(px).strip('[]')
		splitPXinfo = pxInfo.split(" ")
		BWvalue = int(splitPXinfo[0])
		sumOfValues += BWvalue		
avg = sumOfValues/(length * width)

##using gridValue, divide the image into subsections
##the grid loop
xcount = 0
ycount = 0
numOfColumns = (width/gridValue)

while(((xcount + gridValue) <= width) and (ycount + gridValue <= length)):
	for a in range(numOfColumns):
		for y in range(gridValue):
			for x in range(gridValue):
				px = img[(y+ycount),(x+xcount)]
				pxInfo = str(px).strip('[]')
				splitPXinfo = pxInfo.split(" ")
				BWvalue = int(splitPXinfo[0]) ##grey-scale, RGB values are three repeating numbers
				sumOfSubsection += BWvalue			
		#recolor black if darker, white otherwise
		avgSub = sumOfSubsection/(gridValue * gridValue)
		if avgSub < avg:								##or nested for loops, then an if statement (which one is more efficient?)
			for y in range(gridValue) :
				for x in range(gridValue):
					img[y+ycount,x+xcount] = 0 #black
		else:
			for y in range(gridValue) :
				for x in range(gridValue):
					img[y+ycount,x+xcount] = 255 #white
		xcount += gridValue
		sumOfSubsection = 0
	ycount += gridValue
	xcount = 0

cv2.imwrite(bbpName, img)

##converting to txt
img = cv2.imread(bbpName)
length, width, depth = img.shape
columns = width/gridValue
rows = length/gridValue 
arrayBBP = [[0 for v in xrange(columns)] for w in xrange(rows)] ##instantiating two dimensional array

traversalInt = columns * rows
imageArray = [26,12,31] ##will hold list of zeros and ones generated from image
#generate list from image
for y in xrange(rows):
	for x in xrange(columns):
		middleY = (gridValue/2)+(gridValue*y)
		middleX = (gridValue/2)+(gridValue*x)
		
		if ((middleX < width) and (middleY < length)):
			px = img[middleY,middleX]
			pxInfo = str(px).strip('[]')
			splitPXinfo = pxInfo.split(" ")
			BWvalue = int(splitPXinfo[0])
		
			if (BWvalue == 0):
				imageArray.insert(n, 0)
			if (BWvalue == 255):
				imageArray.insert(n, 1)
			n += 1
		else:
			break
a = 0
#transfer list to two dimensional array
for b in xrange(rows):
	for c in xrange(columns):
		arrayBBP[b][c] =  int(imageArray[a]) ##overwriting
		a += 1
		if (a >= n):
			break
#save two-dimensional array
np.savetxt(mapBBPtxt, arrayBBP, fmt='%d')

#resize the finished BBP to be displayed in GUI
bbpImg = Image.open(bbpName)
smaller = bbpImg.resize((800,250))
smaller.save(bbpRzd)
