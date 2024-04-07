from picamera2 import MappedArray, Picamera2, Preview
  
def main():
    picam2 = Picamera2()
    picam2.start_preview(Preview.DRM)
 
    picam2.start()

    while True:
        a = 1;

if __name__ == '__main__':
    main()