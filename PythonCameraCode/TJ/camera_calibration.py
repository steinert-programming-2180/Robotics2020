#!/usr/bin/env python3

import cv2
import numpy
import os.path
import math


class CameraCalibration(object):
    """Calibrates a distorted camera"""

    def __init__(self):
        # number of inside corners in the chessboard (height and width)
        self.checkerboard_height = 6
        self.checkerboard_width = 9

        # size of chessboard square in physical units
        self.square_size = 15.0/16

        self.shape = None
        return

    def calibrateCamera(self, output_dir=None):
        '''Calculates the distortion co-efficients'''

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = numpy.zeros((self.checkerboard_height * self.checkerboard_width, 3), numpy.float32)
        objp[:, :2] = numpy.mgrid[0:self.checkerboard_width, 0:self.checkerboard_height].T.reshape(-1, 2)

        # calibrate coordinates to the physical size of the square
        objp *= self.square_size

        # Arrays to store object points and image points from all the images.
        objpoints = []          # 3d point in real world space
        imgpoints = []          # 2d points in image plane.


        for i in range(1, 30):
            
            #if i in range(38,41) or i in range(45,49) or i == 51 or i == 54 or i in range(58, 60):
            #    continue

            imgLoc = "CameraCalibration\\2020Checkerboards\\img{imgNo}.jpg".format(imgNo = i)
            
            print('Processing file', imgLoc)
            img = cv2.imread(imgLoc)

            if img is None:
                print("ERROR: Unable to read file", imgLoc)
                continue
            self.shape = img.shape

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.checkerboard_width, self.checkerboard_height), None)

            # If found, add object points, image points (after refining them)
            if ret:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (self.checkerboard_width, self.checkerboard_height), corners2, ret)
                
                font = cv2.FONT_HERSHEY_SIMPLEX
                
                cv2.imshow('img', img)
                if output_dir:
                    fullname = os.path.join(output_dir, os.path.basename(imgLoc))
                    cv2.imwrite(fullname, img)

                cv2.waitKey(500)
            else:
                print(imgLoc, 'failed')

        cv2.destroyAllWindows()

        if not objpoints:
            print("No useful images. Quitting...")
            return None

        print('Found {} useful images'.format(len(objpoints)))
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        return (ret, mtx.tolist(), dist.tolist(), rvecs, tvecs)


if __name__ == '__main__':
    import argparse
    import json
    import sys

    parser = argparse.ArgumentParser(description='Calibration utility')
    parser.add_argument('--length', '-l', type=int, default=6, help='Length of checkerboard (number of corners)')
    parser.add_argument('--width', '-w', type=int, default=9, help='Width of checkerboard (number of corners)')
    parser.add_argument('--size', '-s', type=float, default=15.0/16, help='Size of square')
    parser.add_argument('--output', '-o', nargs=1, help="Save the distortion constants to json file")
    parser.add_argument('--output-images', nargs=1, help="Save processed images to directory")

    args = parser.parse_args()



    calibrate = CameraCalibration()
    calibrate.checkerboard_width = args.width
    calibrate.checkerboard_height = args.length
    calibrate.square_size = args.size

    output_dir = args.output_images[0] if args.output_images else None

    ret, mtx, dist, rvecs, tvecs = calibrate.calibrateCamera(output_dir)
    print('reprojection error =', ret)
    print('image center = ({:.2f}, {:.2f})'.format(mtx[0][2], mtx[1][2]))

    fov_x = math.degrees(2.0 * math.atan(calibrate.shape[1] / 2.0 / mtx[0][0]))
    fov_y = math.degrees(2.0 * math.atan(calibrate.shape[0] / 2.0 / mtx[1][1]))
    print('FOV = ({:.2f}, {:.2f}) degrees'.format(fov_x, fov_y))

    print('mtx =', mtx)
    print('dist =', dist)

    # If the command line argument was specified, save matrices to a file in JSON format
    if args.output:
        # Writing JSON data
        with open(args.output[0], 'w') as f:
            json.dump({"camera_matrix": mtx, "distortion": dist}, f)
