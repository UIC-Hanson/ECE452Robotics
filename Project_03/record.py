from pycoral.adapters import common
from pycoral.adapters import detect
from pycoral.utils.edgetpu import make_interpreter
import cv2
import os


model_path = os.path.expanduser('/home/452Lab/ECE452Robotics/CoralModels/ssd_mobilenet_v1_coco_quant_postprocess_edgetpu.tflite')


# Initialize Coral TPU interpreter with your TensorFlow Lite model
interpreter = make_interpreter(model_path)
interpreter.allocate_tensors()

count = 0
cap = cv2.VideoCapture(cv2.CAP_V4L)
path = 'calib_images/'

if not os.path.exists(path):
    os.makedirs(path)

while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        # For Coral TPU inference: Convert frame to the input format expected by the model
        input_image = cv2.resize(frame, common.input_size(interpreter))
        _, scale = common.set_resized_input(interpreter, input_image.shape[:2], lambda size: cv2.resize(input_image, size))
        interpreter.invoke()
        
        # Get object detection results
        results = detect.get_objects(interpreter, score_threshold=0.5, image_scale=scale)
        
        # Example processing results: Drawing bounding boxes around detected objects
        for obj in results:
            bbox = obj.bbox
            cv2.rectangle(frame, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (0, 255, 0), 2)

        cv2.imshow('Current frame', frame)
        if cv2.waitKey(2) & 0xFF == ord('q'):
            break
        if count % 10 == 0:
            cv2.imwrite(os.path.join(path, f"{count}.jpg"), frame)
        count += 1

cap.release()
cv2.destroyAllWindows()