import cv2
import os
from vilib import Vilib
#from picamera.array import PiRGBArray
#from picamera import PiCamera

count = 0

path = 'calib_images/'

if not os.path.exists(path):
    os.makedirs(path)

# Start the camera capture using Vilib
Vilib.camera_start(vflip=False, hflip=False)

while True:

    # Read a frame from the camera
    frame = Vilib.take_photo(path)

    # Check if frame is valid
    if frame is None:
        print("Error: Failed to capture frame")
        break

    # Display the frame
    Vilib.display(frame)

    # Press 'q' to stop the recording
    k = cv2.waitKey(2) & 0xFF
    if k == ord('q'):
        break

    # Save every 10th frame
    if count % 10 == 0:
        cv2.imwrite(path + str(count) + '.jpg', frame)

    count += 1

# Release the camera
Vilib.camera_close()

# Close all OpenCV windows
cv2.destroyAllWindows()
