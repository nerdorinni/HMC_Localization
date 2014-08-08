##This program resizes and converts a panoramic image (mapUnedited.jpg) to black and white with 
##the largest optimal size possible (mapPerfectSize.jpg), then recolors it to a monochromatic 
##brightness pattern, generates and saves an array representing the BP values, and resizes the 
##MBP to be used as the display map in the GUI


from PIL import Image
from numpy import *
from scipy.ndimage import filters
from pylab import *
import os
import cv2
import numpy as np

gridValue = 50
n = 0
sumOfSubsection = 0

imgName = 'mapUnedited.jpg'
mapName = 'mapPerfectSize.jpg'
mapName = 'mapPerfectSizeBW.jpg'
mbpName = 'mapMBP.jpg'
mbpRzd = 'mapMBPrzd.jpg'

mapMBPtxt = 'mapMonochromaticBP.txt' 

img = cv2.imread(imgName)
length, width, depth = img.shape
#perfect size dimensions
largestWholeLength = length/gridValue
perfectLength = largestWholeLength*gridValue
largestWholeWidth = width/gridValue
perfectWidth = largestWholeWidth*gridValue
#resize
pilImg = Image.open(imgName)
perfectSize = pilImg.resize((perfectWidth,perfectLength)) ##resize
perfectSize.save(mapName)
#black and white
pilImg = Image.open(mapName).convert('L')
pilImg.save(mapName)

##convert to MBP
img = cv2.imread(mapName) ## open the just-saved BW, perfectsized map image
length, width, depth = img.shape		 ## length, width, color channels (we just re-seized, we should know the size..)

xcount = 0
ycount = 0
numOfColumns = (width/gridValue)

while(((xcount + gridValue) <= width) and (ycount + gridValue <= length)):
	for a in range(numOfColumns):
		for y in range(gridValue) :
			for x in range(gridValue):
				px = img[(y+ycount),(x+xcount)]
				pxInfo = str(px).strip('[]')
				splitPXinfo = pxInfo.split(" ")
				BWvalue = int(splitPXinfo[0]) ##grey scale values are represented in RGB with three repeating values, we only need one
				sumOfSubsection += BWvalue ##sum of all pixels in a 50 x 50 square
		avgSub = sumOfSubsection/(gridValue * gridValue)

		for y in range(gridValue) :
			for x in range(gridValue):
				img[y+ycount,x+xcount] = avgSub ##recolor the square to its average
		xcount += gridValue
		sumOfSubsection = 0
	ycount += gridValue
	xcount = 0

cv2.imwrite(mbpName, img) ##save MBP

columns = width/gridValue
rows = length/gridValue 

##two-D array that will hold the MBP numerical values
arrayMBP = [[0 for v in xrange(columns)] for w in xrange(rows)]

traversalInt = columns * rows
imageArray = [26,12,31] ##temporary list

for y in xrange(rows):
	for x in xrange(columns):
		middleY = (gridValue/2)+(gridValue*y)
		middleX = (gridValue/2)+(gridValue*x)
		
		if ((middleX < width) and (middleY < length)):
			px = img[middleY,middleX]
			pxInfo = str(px).strip('[]')
			splitPXinfo = pxInfo.split(" ")
			BWvalue = int(splitPXinfo[0])
			imageArray.insert(n, BWvalue) #record the square's value
			n += 1
		else:
			break
a = 0

for b in xrange(rows):
	for c in xrange(columns):
		arrayMBP[b][c] =  int(imageArray[a])##overwriting
		a += 1
		if (a >= n):
			break

np.savetxt(mapMBPtxt, arrayMBP, fmt='%3d') ##3 because the max is three digit numbers

mbpImg = Image.open(mbpName)
smaller = mbpImg.resize((800,250))
smaller.save(mbpRzd)
