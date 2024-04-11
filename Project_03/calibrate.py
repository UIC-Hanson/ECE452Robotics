import cv2
import glob
import numpy as np
import yaml

def load_images(image_path):
    """Load images from a specified directory."""
    return glob.glob(image_path)

def find_corners(images, board_size, square_length, criteria, max_images=20):
    """Find and visualize chessboard corners in images."""
    objp = np.zeros((board_size[1] * board_size[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_length

    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    image_size = None

    count = 0
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = gray.shape[::-1]

        ret, corners = cv2.findChessboardCorners(gray, board_size, None)
        if ret:
            if count < max_images:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                cv2.drawChessboardCorners(img, board_size, corners2, ret)
                cv2.imshow('img', img)
                cv2.waitKey(500)
                count += 1
            else:
                break

    cv2.destroyAllWindows()
    return objpoints, imgpoints, image_size

def calibrate_camera(objpoints, imgpoints, image_size):
    """Calibrate the camera given object points and image points."""
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)
    return ret, mtx, dist, rvecs, tvecs

def save_calibration_data(mtx, dist, filename='calib_data.yaml'):
    """Save the calibration data to a file."""
    calib_data = {'camera_matrix': np.asarray(mtx).tolist(),
                  'distortion_coefficients': np.asarray(dist).tolist()}
    with open(filename, 'w') as file:
        yaml.dump(calib_data, file)

def main():
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    square_length = 0.02  # Square size of the chessboard
    board_size = (7, 6)  # Board size
    images_path = './calib_images/*.jpg'
    
    images = load_images(images_path)
    objpoints, imgpoints, image_size = find_corners(images, board_size, square_length, criteria)
    
    if objpoints and imgpoints and image_size:
        ret, mtx, dist, rvecs, tvecs = calibrate_camera(objpoints, imgpoints, image_size)
        print("Calibration successful.")
        save_calibration_data(mtx, dist)
    else:
        print("Calibration was not successful. Make sure there are images and detected points.")

if __name__ == '__main__':
    main()
