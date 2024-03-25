import numpy as np
import cv2
import glob
import yaml

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# square size of the chessboard
square_length = 0.02

# board size
board_size = (7,6)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((board_size[1]*board_size[0],3), np.float32)
objp[:,:2] = np.mgrid[0:board_size[0],0:board_size[1]].T.reshape(-1,2)
objp = objp * square_length

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Read the captured images from record.py
images = glob.glob('/calib_images/*.jpg')

# count the number of images used for calibration
count = 0

# detecting the chessboard from the images
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, board_size, None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        # here you can change the number of images for calibration
        if count < 20:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            # Draw and display the corners
            cv2.drawChessboardCorners(img, board_size, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)
            count += 1
        else:
            break

cv2.destroyAllWindows()

# Calibrating the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )

calib_data = {
    'camera_matrix':np.asarray(mtx).tolist(),
    'distortion_coefficients':np.asarray(dist).tolist()
}

with open(r'calib_data.yaml', 'w') as file:
    yaml.dump(calib_data, file)