import os as oz
from os import path as os

import shutil
import cv2
import numpy as np

allFiles = oz.listdir('CameraCalibration\\2020Checkerboards')

for img in allFiles:
    _, height, width = cv2.imread(os.join("CameraCalibration\\2020Checkerboards", img)).shape
    if not height == 480:
        oz.unlink(os.join("CameraCalibration\\2020Checkerboards", img))

allFiles = oz.listdir('CameraCalibration\\2020Checkerboards')

counter = 1
for img in allFiles:
    shutil.move(os.join("CameraCalibration\\2020Checkerboards", img),
                os.join("CameraCalibration\\2020Checkerboards","ver2img" + str(counter) + ".jpg"))