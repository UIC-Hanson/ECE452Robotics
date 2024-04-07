from picamera2 import MappedArray, Picamera2, Preview
  
vid = Picamera2.VideoCapture(0)

def main():
    picam2 = Picamera2()
    picam2.start_preview(Preview.DRM)
 
    picam2.start()

    while True:
        a = 1;
