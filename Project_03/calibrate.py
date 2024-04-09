import cv2
import glob
import numpy as np
import yaml
import argparse
from pycoral.adapters import common
from pycoral.utils.edgetpu import make_interpreter
from PIL import Image

# Add argparse to handle command-line arguments
parser = argparse.ArgumentParser(description='Camera calibration with Coral TPU preprocessing.')
parser.add_argument('--model', help='Path to the TensorFlow Lite model file.', required=True)
args = parser.parse_args()

interpreter = make_interpreter(args.model)
interpreter.allocate_tensors()

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
        for fname in images:
        # Preprocess the image using the Coral TPU model
        enhanced_image = preprocess_image_with_model(fname)
        # Convert the PIL image back to an OpenCV image if necessary
        img = np.array(enhanced_image)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

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

def preprocess_image_with_model(image_path):
    image = Image.open(image_path)
    common.set_resized_input(interpreter, image.size, lambda size: image.resize(size, Image.ANTIALIAS))
    interpreter.invoke()
    # Example: Get the enhanced image from the model's output (Adjust based on your model)
    output_data = interpreter.get_tensor(common.output_tensor(interpreter, 0))
    # Process the output_data to get an enhanced image. This step depends on your model's specifics.
    enhanced_image = Image.fromarray(output_data)
    return enhanced_image

def main():
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    square_length = 0.02  # Square size of the chessboard
    board_size = (7, 6)  # Board size
    images_path = '/home/452Lab/ECE452Robotics/Project_03/calib_images/*.jpg'
    
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
