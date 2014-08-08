##This program resizes and converts a panoramic image (mapUnedited.jpg) to the largest optimal 
##size possible (mapPerfectSize.jpg), then recolors it to an RGB average pattern, generates 
##and saves an array representing the RGB values, and resizes the rgbP to be used as the display 
##map in the GUI


from PIL import Image
from numpy import *
from scipy.ndimage import filters
from pylab import *
import os
import cv2
import numpy as np

gridValue = 50
n = 0
sumOfR = 0
sumOfG = 0
sumOfB = 0

##open original file, resize according to the grid value and save 
imgName = 'mapUnedited.jpg'
mapName = 'mapPerfectSize.jpg'
bgrbpName = 'mapRGBBP.jpg'
bbpRzd = 'mapRGBBPrzd.jpg'


img = cv2.imread(imgName)
length, width, depth = img.shape
#perfect size dimensions
largestWholeLength = length/gridValue
perfectLength = largestWholeLength*gridValue
largestWholeWidth = width/gridValue
perfectWidth = largestWholeWidth*gridValue

##resize
pilImg = Image.open(imgName)
perfectSize = pilImg.resize((perfectWidth,perfectLength)) 
perfectSize.save(mapName)

##convert to BGR-BP and save new image
img = cv2.imread(mapName) ## open the just-saved perfectsized map image
length, width, depth = img.shape ## length, width, color channels

xcount = 0
ycount = 0
n = 0
x = 0
numOfColumns = (width/gridValue)

while(((xcount + gridValue) <= width) and (ycount + gridValue <= length)):
	for a in range(numOfColumns):
		for y in range(gridValue) :
			for x in range(gridValue):
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
		avgRsub = sumOfR/(gridValue * gridValue)
		avgGsub = sumOfG/(gridValue * gridValue)
		avgBsub = sumOfB/(gridValue * gridValue)

		for y in range(gridValue) :
			for x in range(gridValue):
				img[y+ycount,x+xcount] = [avgBsub, avgGsub, avgRsub] #bgr rather than RGB?
		xcount += gridValue
		sumOfR = 0
		sumOfG = 0
		sumOfB = 0
	ycount += gridValue
	xcount = 0

cv2.imwrite(bgrbpName, img) ##save bgrBP

##saving three numbers to a list
var1 = width/gridValue #width(columns)
#print var1
var2 = length/gridValue 
#print var2 

##actual two-D array..perhaps a 3d array would work?
arrayBBP = [[0 for v in xrange(var1*3)] for w in xrange(var2)]

traversalInt = var1 * var2
#print traversalInt
imageArray = [26,12,31] ##list of R, G, and B values from image blocks

#accumulate block colors onto list
count = 0
z = 0
for y in xrange(var2):
	for x in xrange(var1):
		middleY = (gridValue/2)+(gridValue*y)
		middleX = (gridValue/2)+(gridValue*x)
		
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

#save list to two-dimensional array
arrayOne = [[0 for v in xrange(var1*3)] for w in xrange(var2)] ##instantiating?
for b in xrange(var2):
	for c in xrange(var1*3):
		#print imageArray[a]
		arrayOne[b][c] =  int(imageArray[a])##overwriting
		a += 1
		if (a >= z):
			break
np.savetxt('mapRGBarray.txt', arrayOne, fmt='%3d') ##because the max is three digit numbers

#resize map to be displayed in GUI
rgbbpImg = Image.open(bgrbpName)
smaller = rgbbpImg.resize((800,250))
smaller.save(bbpRzd)
