import cv2
import numpy
import math
import colorsys
import time
from enum import Enum

class GripPipeline:
    """
    An OpenCV pipeline generated by GRIP.
    """
    
    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__hsv_threshold_hue = [32.0, 85.0]
        self.__hsv_threshold_saturation = [98.0, 255.0]
        self.__hsv_threshold_value = [80.0, 255.0]

        self.hsv_threshold_output = None

        self.__blur_input = self.hsv_threshold_output
        self.__blur_type = BlurType.Median_Filter
        self.__blur_radius = 6.9

        self.blur_output = None

        self.__find_contours_input = self.blur_output
        self.__find_contours_external_only = False

        self.find_contours_output = None


    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step HSV_Threshold0:
        self.__hsv_threshold_input = source0
        (self.hsv_threshold_output) = self.__hsv_threshold(self.__hsv_threshold_input, self.__hsv_threshold_hue, self.__hsv_threshold_saturation, self.__hsv_threshold_value)

        # Step Blur0:
        self.__blur_input = self.hsv_threshold_output
        (self.blur_output) = self.__blur(self.__blur_input, self.__blur_type, self.__blur_radius)

        # Step Find_Contours0:
        self.__find_contours_input = self.blur_output
        (self.find_contours_output) = self.__find_contours(self.__find_contours_input, self.__find_contours_external_only)


    @staticmethod
    def __hsv_threshold(input, hue, sat, val):
        """Segment an image based on hue, saturation, and value ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max value.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
        return cv2.inRange(out, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))

    @staticmethod
    def __blur(src, type, radius):
        """Softens an image using one of several filters.
        Args:
            src: The source mat (numpy.ndarray).
            type: The blurType to perform represented as an int.
            radius: The radius for the blur as a float.
        Returns:
            A numpy.ndarray that has been blurred.
        """
        if(type is BlurType.Box_Blur):
            ksize = int(2 * round(radius) + 1)
            return cv2.blur(src, (ksize, ksize))
        elif(type is BlurType.Gaussian_Blur):
            ksize = int(6 * round(radius) + 1)
            return cv2.GaussianBlur(src, (ksize, ksize), round(radius))
        elif(type is BlurType.Median_Filter):
            ksize = int(2 * round(radius) + 1)
            return cv2.medianBlur(src, ksize)
        else:
            return cv2.bilateralFilter(src, -1, round(radius), round(radius))

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if(external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
        return contours


BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')

gLine  = GripPipeline()

cap = cv2.VideoCapture(0)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(4,4))

xVals = [10000, -10000]
yVals = [10000, -10000]

corner1 = [xVals[0], yVals[0]]
corner2 = [xVals[1], yVals[1]]

def minMax(bRect):
    if(bRect[0] < xVals[0]) :
        xVals[0] = bRect[0]
    if(bRect[1] < yVals[0]) :
        yVals[0] = bRect[1]
    if(bRect[2] > xVals[1]) :
        xVals[1] = bRect[2]
    if(bRect[3] > yVals[1]) :
        yVals[1] = bRect[3]
    
    corner1 = [xVals[0], yVals[0]]
    corner2 = [xVals[1], yVals[1]]

while(True):
    # Capture frame-by-frame
    ret, src = cap.read()
    startTime = time.time()
    image = src

    xVals = [10000, -10000]
    yVals = [10000, -10000]

    gLine.process(image)

    contours = gLine.find_contours_output

    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)

    for i, c in enumerate(contours):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        boundRect[i] = cv2.boundingRect(contours_poly[i])

    i = 0
    for contour in contours:
        r, g, b = colorsys.hsv_to_rgb(i,1,1)
        cv2.drawContours(image, [contour], 0, (b * 255, g * 255, r * 255), 2)
        i = i + 0.1
        if i > 1 :
            i = i - 1
    #postDilate = cv2.dilate(postErode, kernel, iterations=1)

    c = 0
    for i in range(len(contours)):
        r, g, b = colorsys.hsv_to_rgb(c,1,1)
        color = (255 * b, 255 * g, 255 * r)
        c = c + 0.1
        if c > 1:
            c = 0
        cv2.rectangle(image, (int(boundRect[i][0]), int(boundRect[i][1])), \
          (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), color, 2)
        
        minMax([int(boundRect[i][0]), int(boundRect[i][1]), int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])])
        
    if(len(contours) > 0):
        cv2.rectangle(image, (xVals[0], yVals[0]), (xVals[1], yVals[1]), (0, 255, 0), thickness=3)
    # Display the resulting frame
    cv2.imshow('frame',image)
    print(time.time() - startTime)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()