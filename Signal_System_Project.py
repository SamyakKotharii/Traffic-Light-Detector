import os
import cv2 as cv
import numpy as np

def detect(filepath, file):
    #To read input Image
    font = cv.FONT_HERSHEY_COMPLEX
    img = cv.imread(filepath+file)
    oimg = img
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    #For color range
    lowerRed1 = np.array([0,100,100])
    upperRed1 = np.array([20,255,255])
    lowerRed2 = np.array([160,100,100])
    upperRed2 = np.array([180,255,255])
    lowerGreen = np.array([40,50,50])
    upperGreen = np.array([90,255,255])
    lowerYellow = np.array([15,150,150])
    upperYellow = np.array([35,255,255])

    #Masking
    maskr1 = cv.inRange(hsv, lowerRed1, upperRed1)
    maskr2 = cv.inRange(hsv, lowerRed2, upperRed2)
    maskg = cv.inRange(hsv, lowerGreen, upperGreen)
    masky = cv.inRange(hsv, lowerYellow, upperYellow)
    maskr = cv.add(maskr1, maskr2)
    
    #Size of Image
    sizeofimg = img.shape

    #Hough Circle Transform Method
    red_circles = cv.HoughCircles(maskr, cv.HOUGH_GRADIENT, 1, 80,
                               param1=50, param2=10, minRadius=0, maxRadius=30)

    green_circles = cv.HoughCircles(maskg, cv.HOUGH_GRADIENT, 1, 60,
                                 param1=50, param2=10, minRadius=0, maxRadius=30)

    yellow_circles = cv.HoughCircles(masky, cv.HOUGH_GRADIENT, 1, 30,
                                 param1=50, param2=5, minRadius=0, maxRadius=30)

    #To detect Traffic light and Printing Circle on detected light and printing action on Image
    radius = 5
    b = 4.0 / 10
    if red_circles is not None:
        red_circles = np.uint16(np.around(red_circles))

        for i in red_circles[0, :]:
            if i[0] > sizeofimg[1] or i[1] > sizeofimg[0]or i[1] > sizeofimg[0]*b:
                continue

            h, s = 0.0, 0.0
            for m in range(-radius, radius):
                for n in range(-radius, radius):

                    if (i[1]+m) >= sizeofimg[0] or (i[0]+n) >= sizeofimg[1]:
                        continue
                    h += maskr[i[1]+m, i[0]+n]
                    s += 1
            if h / s > 50:
                cv.circle(oimg, (i[0], i[1]), i[2]+10, (0, 0, 255), 2)
                cv.circle(maskr, (i[0], i[1]), i[2]+30, (255, 255, 255), 2)
                cv.putText(oimg,'Stop',(i[0], i[1]), font, 1,(255,0,0),2,cv.LINE_AA)

    if green_circles is not None:
        green_circles = np.uint16(np.around(green_circles))

        for i in green_circles[0, :]:
            if i[0] > sizeofimg[1] or i[1] > sizeofimg[0] or i[1] > sizeofimg[0]*b:
                continue

            h, s = 0.0, 0.0
            for m in range(-radius, radius):
                for n in range(-radius, radius):

                    if (i[1]+m) >= sizeofimg[0] or (i[0]+n) >= sizeofimg[1]:
                        continue
                    h += maskg[i[1]+m, i[0]+n]
                    s += 1
            if h / s > 100:
                cv.circle(oimg, (i[0], i[1]), i[2]+10, (0, 0, 255), 2)
                cv.circle(maskg, (i[0], i[1]), i[2]+30, (255, 255, 255), 2)
                cv.putText(oimg,'Go',(i[0], i[1]), font, 1,(255,0,0),2,cv.LINE_AA)

    if yellow_circles is not None:
        yellow_circles = np.uint16(np.around(yellow_circles))

        for i in yellow_circles[0, :]:
            if i[0] > sizeofimg[1] or i[1] > sizeofimg[0] or i[1] > sizeofimg[0]*b:
                continue

            h, s = 0.0, 0.0
            for m in range(-radius, radius):
                for n in range(-radius, radius):

                    if (i[1]+m) >= sizeofimg[0] or (i[0]+n) >= sizeofimg[1]:
                        continue
                    h += masky[i[1]+m, i[0]+n]
                    s += 1
            if h/s > 50:
                cv.circle(oimg, (i[0], i[1]), i[2]+10, (0, 0, 255), 2)
                cv.circle(masky, (i[0], i[1]), i[2]+30, (255, 255, 255), 2)
                cv.putText(oimg,'Slow Down',(i[0], i[1]), font, 1,(255,0,0),2,cv.LINE_AA)
    #Output
    cv.imshow('Output Message', oimg)
    cv.imwrite(pathofImg+'//result//'+file, oimg)
    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == '__main__':
    pathofImg = os.path.abspath('..')+'//light//'
    for x in os.listdir(pathofImg):
        print(x)
        if x.endswith('.jpg') or x.endswith('.JPG'):
            detect(pathofImg, x)

