import cv2
import os
from vilib import Vilib

count = 0

path = 'calib_images/'

if not os.path.exists(path):
    os.makedirs(path)

# Start the camera capture using Vilib
Vilib.camera_start(vflip=False, hflip=False)
Vilib.display(local=True, web=True)

while True:
    # Read a frame from the camera
    frame = Vilib.take_photo()
    
    # Display the frame
    Vilib.display_frame(frame)

    # Press 'q' to stop the recording
    k = cv2.waitKey(2) & 0xFF
    if k == ord('q'):
        break

    # Save every 10th frame
    if count % 10 == 0:
        cv2.imwrite(path + str(count) + '.jpg', frame)

    count += 1

# Release the camera
Vilib.camera_stop()

# Close all OpenCV windows
cv2.destroyAllWindows()
