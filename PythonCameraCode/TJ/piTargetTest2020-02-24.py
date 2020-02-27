import cv2
import numpy as np
import math
import colorsys
import time
from enum import Enum
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink, CvSource, VideoMode

class FixingMistakes:
    """
    An OpenCV pipeline generated by GRIP.
    """
    
    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__hsv_threshold_hue = [59.81587611266919, 83.37920461389646]
        self.__hsv_threshold_saturation = [151.47148632892322, 254.7363828862536]
        self.__hsv_threshold_value = [31.28797058320366, 245.0109921659375]

        self.hsv_threshold_output = None

        self.__cv_dilate_src = self.hsv_threshold_output
        self.__cv_dilate_kernel = None
        self.__cv_dilate_anchor = (-1, -1)
        self.__cv_dilate_iterations = 2.0
        self.__cv_dilate_bordertype = cv2.BORDER_CONSTANT
        self.__cv_dilate_bordervalue = (-1)

        self.cv_dilate_output = None

        self.__cv_erode_src = self.cv_dilate_output
        self.__cv_erode_kernel = None
        self.__cv_erode_anchor = (-1, -1)
        self.__cv_erode_iterations = 2.0
        self.__cv_erode_bordertype = cv2.BORDER_CONSTANT
        self.__cv_erode_bordervalue = (-1)

        self.cv_erode_output = None


        self.__mask_mask = self.cv_erode_output

        self.mask_output = None

        self.__find_contours_input = self.cv_erode_output
        self.__find_contours_external_only = False

        self.find_contours_output = None

        self.__filter_contours_0_contours = self.find_contours_output
        self.__filter_contours_0_min_area = 10.0
        self.__filter_contours_0_min_perimeter = 0.0
        self.__filter_contours_0_min_width = 0.0
        self.__filter_contours_0_max_width = 1000.0
        self.__filter_contours_0_min_height = 0.0
        self.__filter_contours_0_max_height = 1000.0
        self.__filter_contours_0_solidity = [0.0, 100]
        self.__filter_contours_0_max_vertices = 1000.0
        self.__filter_contours_0_min_vertices = 0.0
        self.__filter_contours_0_min_ratio = 0.0
        self.__filter_contours_0_max_ratio = 1000.0

        self.filter_contours_0_output = None

        self.__convex_hulls_contours = self.filter_contours_0_output

        self.convex_hulls_output = None

        self.__filter_contours_1_contours = self.convex_hulls_output
        self.__filter_contours_1_min_area = 0.0
        self.__filter_contours_1_min_perimeter = 0.0
        self.__filter_contours_1_min_width = 0.0
        self.__filter_contours_1_max_width = 1000.0
        self.__filter_contours_1_min_height = 0.0
        self.__filter_contours_1_max_height = 1000.0
        self.__filter_contours_1_solidity = [0, 100]
        self.__filter_contours_1_max_vertices = 20.0
        self.__filter_contours_1_min_vertices = 3.0
        self.__filter_contours_1_min_ratio = 0.0
        self.__filter_contours_1_max_ratio = 1000.0

        self.filter_contours_1_output = None


    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step HSV_Threshold0:
        self.__hsv_threshold_input = source0
        (self.hsv_threshold_output) = self.__hsv_threshold(self.__hsv_threshold_input, self.__hsv_threshold_hue, self.__hsv_threshold_saturation, self.__hsv_threshold_value)

        # Step CV_dilate0:
        self.__cv_dilate_src = self.hsv_threshold_output
        (self.cv_dilate_output) = self.__cv_dilate(self.__cv_dilate_src, self.__cv_dilate_kernel, self.__cv_dilate_anchor, self.__cv_dilate_iterations, self.__cv_dilate_bordertype, self.__cv_dilate_bordervalue)

        # Step CV_erode0:
        self.__cv_erode_src = self.cv_dilate_output
        (self.cv_erode_output) = self.__cv_erode(self.__cv_erode_src, self.__cv_erode_kernel, self.__cv_erode_anchor, self.__cv_erode_iterations, self.__cv_erode_bordertype, self.__cv_erode_bordervalue)

        # Step Mask0:
        self.__mask_input = source0
        self.__mask_mask = self.cv_erode_output
        (self.mask_output) = self.__mask(self.__mask_input, self.__mask_mask)

        # Step Find_Contours0:
        self.__find_contours_input = self.cv_erode_output
        (self.find_contours_output) = self.__find_contours(self.__find_contours_input, self.__find_contours_external_only)

        # Step Filter_Contours0:
        self.__filter_contours_0_contours = self.find_contours_output
        (self.filter_contours_0_output) = self.__filter_contours(self.__filter_contours_0_contours, self.__filter_contours_0_min_area, self.__filter_contours_0_min_perimeter, self.__filter_contours_0_min_width, self.__filter_contours_0_max_width, self.__filter_contours_0_min_height, self.__filter_contours_0_max_height, self.__filter_contours_0_solidity, self.__filter_contours_0_max_vertices, self.__filter_contours_0_min_vertices, self.__filter_contours_0_min_ratio, self.__filter_contours_0_max_ratio)

        # Step Convex_Hulls0:
        self.__convex_hulls_contours = self.filter_contours_0_output
        (self.convex_hulls_output) = self.__convex_hulls(self.__convex_hulls_contours)

        # Step Filter_Contours1:
        self.__filter_contours_1_contours = self.convex_hulls_output
        (self.filter_contours_1_output) = self.__filter_contours(self.__filter_contours_1_contours, self.__filter_contours_1_min_area, self.__filter_contours_1_min_perimeter, self.__filter_contours_1_min_width, self.__filter_contours_1_max_width, self.__filter_contours_1_min_height, self.__filter_contours_1_max_height, self.__filter_contours_1_solidity, self.__filter_contours_1_max_vertices, self.__filter_contours_1_min_vertices, self.__filter_contours_1_min_ratio, self.__filter_contours_1_max_ratio)


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
    def __cv_dilate(src, kernel, anchor, iterations, border_type, border_value):
        """Expands area of higher value in an image.
        Args:
           src: A np.ndarray.
           kernel: The kernel for dilation. A np.ndarray.
           iterations: the number of times to dilate.
           border_type: Opencv enum that represents a border type.
           border_value: value to be used for a constant border.
        Returns:
            A np.ndarray after dilation.
        """
        return cv2.dilate(src, kernel, anchor, iterations = (int) (iterations +0.5),
                            borderType = border_type, borderValue = border_value)

    @staticmethod
    def __cv_erode(src, kernel, anchor, iterations, border_type, border_value):
        """Expands area of lower value in an image.
        Args:
           src: A np.ndarray.
           kernel: The kernel for erosion. A np.ndarray.
           iterations: the number of times to erode.
           border_type: Opencv enum that represents a border type.
           border_value: value to be used for a constant border.
        Returns:
            A np.ndarray after erosion.
        """
        return cv2.erode(src, kernel, anchor, iterations = (int) (iterations +0.5),
                            borderType = border_type, borderValue = border_value)

    @staticmethod
    def __mask(input, mask):
        """Filter out an area of an image using a binary mask.
        Args:
            input: A three channel np.ndarray.
            mask: A black and white np.ndarray.
        Returns:
            A three channel np.ndarray.
        """
        return cv2.bitwise_and(input, input, mask=mask)

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
        _, contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
        return contours

    @staticmethod
    def __convex_hulls(input_contours):
        """Computes the convex hulls of contours.
        Args:
            input_contours: A list of np.ndarray that each represent a contour.
        Returns:
            A list of np.ndarray that each represent a contour.
        """
        output = []
        for contour in input_contours:
            output.append(cv2.convexHull(contour))
        return output

    @staticmethod
    def __filter_contours(input_contours, min_area, min_perimeter, min_width, max_width,
                        min_height, max_height, solidity, max_vertex_count, min_vertex_count,
                        min_ratio, max_ratio):
        """Filters out contours that do not meet certain criteria.
        Args:
            input_contours: Contours as a list of np.ndarray.
            min_area: The minimum area of a contour that will be kept.
            min_perimeter: The minimum perimeter of a contour that will be kept.
            min_width: Minimum width of a contour.
            max_width: MaxWidth maximum width.
            min_height: Minimum height.
            max_height: Maximimum height.
            solidity: The minimum and maximum solidity of a contour.
            min_vertex_count: Minimum vertex Count of the contours.
            max_vertex_count: Maximum vertex Count.
            min_ratio: Minimum ratio of width to height.
            max_ratio: Maximum ratio of width to height.
        Returns:
            Contours as a list of np.ndarray.
        """
        output = []
        for contour in input_contours:
            x,y,w,h = cv2.boundingRect(contour)
            if (w < min_width or w > max_width):
                continue
            if (h < min_height or h > max_height):
                continue
            area = cv2.contourArea(contour)
            if (area < min_area):
                continue
            if (cv2.arcLength(contour, True) < min_perimeter):
                continue
            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            if (solid < solidity[0] or solid > solidity[1]):
                continue
            if (len(contour) < min_vertex_count or len(contour) > max_vertex_count):
                continue
            ratio = (float)(w) / h
            if (ratio < min_ratio or ratio > max_ratio):
                continue
            output.append(contour)
        return output




def filter(polys):
    output = []

    for p in polys :
        if len(p) == 4:
            output.append(p)

    # if len(output) == 0 and len(polys) > 0:
    #     output.append(max(polys, key = cv2.contourArea))

    return output



def findCameraPoints (contour):
    highest = [0,100000] 
    secondHighest = [0,1000000]
    for i in contour.tolist(): #Finds two highest first
        # print(i)
        if i[0][1] < highest[1]:
            secondHighest = highest
            highest = i[0]
        elif i[0][1] < secondHighest[1]:
            secondHighest = i[0]

    highPoints = [highest, secondHighest]
    
    lowPoints = [i[0] for i in contour.tolist()]
    for i in highPoints:
        lowPoints.remove(i)
            
    out = []

    if highPoints[0][0] < highPoints[1][0]:
        out.append(highPoints[0])
        out.append(highPoints[1])
    else:
        out.append(highPoints[1])
        out.append(highPoints[0])
    
    if lowPoints[0][0] < lowPoints[1][0]:
        out.append(lowPoints[0])
        out.append(lowPoints[1])
    else:
        out.append(lowPoints[1])
        out.append(lowPoints[0])

    print(out)

    return out

def compute_output_values(rvec, tvec):
    #Compute the necessary output distance and angles
    x = tvec[0][0]
    z = tvec[2][0]
    # distance in the horizontal plane between camera and target
    distance = math.sqrt(x**2 + z**2)
    # horizontal angle between camera center line and target
    angle1 = math.atan2(x, z)
    rot, _ = cv2.Rodrigues(rvec)
    rot_inv = rot.transpose()
    pzero_world = np.matmul(rot_inv, -tvec)
    angle2 = math.atan2(pzero_world[0][0], pzero_world[2][0])
    return distance, angle1, angle2


worldCoordinates = [[-19.645, 0, 0], #Top left
                    [19.645, 0, 0],  #Top right
                    [-9.8125, -17, 0], #Bottom left
                    [9.8125, -17, 0]] #Bottom right
cameraMatrix = [[654.5772870367346, 0.0, 337.0507139506593], [0.0, 656.1587737528665, 250.33959129066534], [0.0, 0.0, 1.0]]
dist = [[0.08347161242882054, -0.4187163193375646, 0.007898104661949604, -0.001084625132784283, -0.2415468504089658]]
gLine = FixingMistakes()

maxPics = 30
val = 35

blank_image = np.zeros((480,640,3), np.uint8)
epsilon = 25

'''
while True:
    imgloc = "CameraCalibration\\2020Target\\my_photo-{imgNo}.jpg".format(imgNo = val)
    #imgloc = "CameraCalibration\\2020Target\\10sqrt2.jpg"
    img = cv2.imread(imgloc)
    pipeline.process(img)

    processedImg = pipeline.mask_output
    cv2.imshow('mask', processedImg)
    contours = pipeline.filter_contours_1_output

    approxPolys = []
    for c in contours:
        approxPolys.append(cv2.approxPolyDP(c, epsilon, True))
    

    filteredPolys = filter(approxPolys)

    poly_img = np.zeros((480,640,3), np.uint8)
    
    imgCoords = []

    if len(filteredPolys) > 0:
        c = max(filteredPolys, key = cv2.contourArea)
        cv2.drawContours(img, [c], -1, (255, 255, 255), 1)
        imgCoords = findCameraPoints(c)

        cv2.circle(img, tuple(imgCoords[0]), 2, (255, 0, 0), 1)   #top left - blue
        cv2.circle(img, tuple(imgCoords[1]), 2, (0, 0, 255), 1)   #top right - red
        cv2.circle(img, tuple(imgCoords[2]), 2, (255, 0, 255), 1) #bottom left - magenta
        cv2.circle(img, tuple(imgCoords[3]), 2, (0, 255, 255), 1) #bottom right - yellow

        retval, rvec, tvec = cv2.solvePnP(np.array(worldCoordinates, dtype=np.float32), 
                                np.array(imgCoords, dtype=np.float32), 
                                np.array(cameraMatrix, dtype=np.float32), 
                                np.array(dist, dtype=np.float32))
        distance, angle1, angle2 = compute_output_values(rvec, tvec)

        print(distance, angle1, angle2)

    cv2.imshow('image', img)
    ch = cv2.waitKey(0)

    if(ch == ord(' ')):
        break

cv2.destroyAllWindows()
'''

def doNothing():
    return True

camera = UsbCamera("CammyBoi", 0)
camera.setExposureManual(0)
#camera.setConfigJson(json.dumps(json.load(open(configFile, "rt", encoding="utf-8"))))
vidSink = CvSink("Camera")
vidSink.setSource(camera)

vidSource = CvSource("Processed", VideoMode.PixelFormat.kMJPEG, 640, 480, 30)
networkStream = MjpegServer("Stream", 1181)
networkStream.setSource(vidSource)
img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

while(True):
    ret, src = vidSink.grabFrame(img)
    startTime = time.time()

    gLine.process(src)

    processedImg = gLine.mask_output
    contours = gLine.filter_contours_1_output

    approxPolys = []
    for c in contours:
        approxPolys.append(cv2.approxPolyDP(c, epsilon, True))


    filteredPolys = filter(approxPolys)

    poly_img = np.zeros((480,640,3), np.uint8)

    imgCoords = []

    if len(filteredPolys) > 0:
        c = max(filteredPolys, key = cv2.contourArea)
        cv2.drawContours(img, [c], -1, (255, 255, 255), 1)
        imgCoords = findCameraPoints(c)

        cv2.circle(img, tuple(imgCoords[0]), 2, (255, 0, 0), 1)   #top left - blue
        cv2.circle(img, tuple(imgCoords[1]), 2, (0, 0, 255), 1)   #top right - red
        cv2.circle(img, tuple(imgCoords[2]), 2, (255, 0, 255), 1) #bottom left - magenta
        cv2.circle(img, tuple(imgCoords[3]), 2, (0, 255, 255), 1) #bottom right - yellow

        retval, rvec, tvec = cv2.solvePnP(np.array(worldCoordinates, dtype=np.float32), 
                            np.array(imgCoords, dtype=np.float32), 
                            np.array(cameraMatrix, dtype=np.float32), 
                            np.array(dist, dtype=np.float32))
        distance, angle1, angle2 = compute_output_values(rvec, tvec)

        print(distance, angle1, angle2)
        
        print("time: ", time.time() - startTime)

        vidSource.putFrame(processedImg)