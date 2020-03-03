import os as oz
from os import path as os

import shutil
import cv2
import numpy as np


allFiles = oz.listdir('CameraCalibration\\2020Checkerboards')

counter = 0
for img in allFiles:
    shutil.move(os.join("CameraCalibration\\2020Checkerboards", img),
                os.join("CameraCalibration\\2020Checkerboards","img" + str(counter) + ".jpg"))

    counter += 1