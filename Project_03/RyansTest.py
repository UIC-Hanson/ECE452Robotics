import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()
    
while True:
    
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    resize_frame = cv2.resize(frame, (360, 480))
    resize_gray  = cv2.resize(gray, (360, 480))
    
    cv2.imshow('Frame', resize_frame)
    cv2.imshow('Gray', resize_gray)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()