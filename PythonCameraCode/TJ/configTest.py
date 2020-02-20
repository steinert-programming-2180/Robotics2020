import cv2
import numpy as np
import math
import colorsys
import time
import json
from enum import Enum
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer

class ContourTests:
    """
    An OpenCV pipeline generated by GRIP.
    """
    
    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__hsv_threshold_hue = [38.84892086330935, 104.74402730375427]
        self.__hsv_threshold_saturation = [0.0, 255.0]
        self.__hsv_threshold_value = [49.94621811955906, 255.0]

        self.hsv_threshold_output = None

        self.__blur_input = self.hsv_threshold_output
        self.__blur_type = BlurType.Box_Blur
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
            input: A BGR np.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max value.
        Returns:
            A black and white np.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
        return cv2.inRange(out, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))

    @staticmethod
    def __blur(src, type, radius):
        """Softens an image using one of several filters.
        Args:
            src: The source mat (np.ndarray).
            type: The blurType to perform represented as an int.
            radius: The radius for the blur as a float.
        Returns:
            A np.ndarray that has been blurred.
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
            input: A np.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of np.ndarray where each one represents a contour.
        """
        if(external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        im2, contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
        return contours


BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')

configFile = "/boot/frc.json"
gLine = ContourTests()
camera = UsbCamera("CammyBoi", 0)
img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
camera.setConfigJson(json.dumps(json.load(open(configFile, "rt", encoding="utf-8"))))
cs = CameraServer.getInstance()
cs.addCamera(camera)
cap = cs.getVideo()
kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(4,4))
networkStream = cs.putVideo("StreamyBoi", 320, 240)

if __name__ == "__main__":
    
    while(True):
        # Capture frame-by-frame
        (ret, src) = cap.grabFrame(img)
        startTime = time.time()

        gLine.process(src)
        image = src
        contours = gLine.find_contours_output

        c1, c2 = None, None

        if len(contours) > 0:
            
            sortedContours = sorted(contours, key=cv2.contourArea, reverse=True)

            c1 = sortedContours[0]

            cv2.drawContours(image, [c1], 0, (0,255,0), 3)

            if len(contours) > 1:
                c2 = sortedContours[1]
                cv2.drawContours(image, [c2], 0, (0,255,0), 3)


        print(time.time() - startTime)

        networkStream.putFrame(image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break