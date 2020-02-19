import cv2
import numpy
import math
import colorsys
import time
from enum import Enum

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, src = cap.read()

    cv2.imshow('frame',src)
    #print(time.time() - startTime)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()