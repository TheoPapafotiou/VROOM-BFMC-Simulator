import cv2
import numpy as np
from scipy.ndimage.interpolation import shift
def thresholding(img):

    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lowerWhite = np.array([0,0,0])
    upperWhite = np.array([0,179,120])
    
    maskWhite = cv2.inRange(imgHsv, lowerWhite, upperWhite)

    return maskWhite

def warpImg(img, points, w, h, inv=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])

  
    matrix_inv = cv2.getPerspectiveTransform(pts2, pts1)

    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    imgWarp = cv2.warpPerspective(img, matrix, (w,h))

    return imgWarp, matrix, matrix_inv
    
def nothing(a):
    pass
 
def initializeTrackbars(intialTracbarVals, wT=640, hT=480):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0],wT//2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2],wT//2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], hT, nothing)
 
def valTrackbars(wT=640, hT=480):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                      (widthBottom , heightBottom ), (wT-widthBottom, heightBottom)])
    return points
 
def drawPoints(img,points):
    for x in range(4):
        cv2.circle(img,(int(points[x][0]),int(points[x][1])),15,(0,0,255),cv2.FILLED)
    return img
    
def getHistogram(img,minPer=0.1,display= False,region=1):
 
    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0]//region:,:], axis=0)
 


    midpoint = np.int(histValues.shape[0]/2)
    #print(histValues)
    maxValue = np.max(histValues)
    minValue = minPer*maxValue
    
    leftHist = histValues[:midpoint]
    rightHist = histValues[midpoint:]
    
    maxValueLeft = np.max(leftHist)
    minValueLeft = minPer*maxValueLeft
    
    maxValueRight = np.max(rightHist)
    minValueRight = minPer*maxValueRight

    indexArrayLeft = np.where(leftHist >= minValueLeft)

    basePointLeft = int(np.average(indexArrayLeft))

    indexArrayRight = np.array(np.where(rightHist >= minValueRight)) + midpoint

    basePointRight = int(np.average(indexArrayRight)) 

    #print(basePointLeft, basePointRight)
	
	# Compute the left and right max pixels
    
    if region == 1:
        leftx_ = np.argmax(histValues[:midpoint])
        rightx_ = np.argmax(histValues[midpoint:]) + midpoint

        if histValues[rightx_] == 0 and histValues[leftx_] == 0:
            basePoint = midpoint
            if display:
                imgHist = np.zeros((img.shape[0],img.shape[1],3),np.uint8)
                for x,intensity in enumerate(histValues):
                    cv2.line(imgHist,(x,img.shape[0]),(x,int(img.shape[0]-intensity//255//region)),(255,0,255),1)
                return basePoint,imgHist

        elif histValues[rightx_] == 0:
            histValues = shift(histValues, midpoint - leftx_, cval=0)

        elif histValues[leftx_] == 0:
            histValues = shift(histValues, rightx_ - midpoint, cval=0)
            #print(histValues)
            #print(rightx_ - midpoint)
            #print(rightx_)

    leftx_ = np.argmax(histValues[:midpoint])
    rightx_ = np.argmax(histValues[midpoint:]) + midpoint


    left_sum1 = np.sum(histValues[leftx_:midpoint])
    left_sum2 = np.sum(histValues[:leftx_])


    right_sum1 = np.sum(histValues[rightx_:])
    right_sum2 = np.sum(histValues[midpoint:rightx_])



    #print((basePointRight+basePointLeft)/2.0)
    indexArray = np.where(histValues > 0)

    basePoint = int(np.average(indexArray))

    #print(basePoint)
    #print(leftx_)
    #print(rightx_)
    if display:
        imgHist = np.zeros((img.shape[0],img.shape[1],3),np.uint8)
        for x,intensity in enumerate(histValues):
            cv2.line(imgHist,(x,img.shape[0]),(x,int(img.shape[0]-intensity//255//region)),(255,0,255),1)
        cv2.circle(imgHist,(basePoint,img.shape[0]),20,(0,255,255),cv2.FILLED)
        return basePoint,imgHist
 
    return basePoint
 
def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

def test(img):
    rows = img.shape[0]
    columns = img.shape[1]
    for i in range(1, rows):
        rev_row = img[i][::-1]
        last_white = len(rev_row) - np.argmax(rev_row) - 1
        for j in range(1, columns):
            if img[i,j-1] == 255 and j < last_white :
                img[i,j] = 255

    return img 


