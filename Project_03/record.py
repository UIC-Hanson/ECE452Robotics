import cv2
import os

count = 0

path = 'calib_images/'

if not os.path.exists(path):
    os.makedirs(path)

cap = cv2.VideoCapture(cv2.CAP_V4L)

while cap.isOpened():
    # Read and display each frame
    ret, frame = cap.read()
    if ret:
        cv2.imshow('Current frame', frame)
        k = cv2.waitKey(2) & 0xFF
        # press 'q' to stop the recording
        if k == ord('q'):
           break
        if count%10 == 0:
            cv2.imwrite(path+str(count)+'.jpg',frame)
        count += 1

# close the camera
cap.release()
  
# close all the opened windows
cv2.destroyAllWindows()