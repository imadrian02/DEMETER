import time
import numpy as np
import threading
import cv2
import serial
from ultralytics import YOLO
from supervision import ByteTrack, BoundingBoxAnnotator, LabelAnnotator, Detections
from greenonbrown import GreenOnBrown
from xygantry import *
from videocapture import *

start_time = time.time()

model = YOLO(r'/home/pi/DEMETER/model/best.pt')
detector = GreenOnBrown(algorithm='exg')
tracker = ByteTrack()

box_annotator = BoundingBoxAnnotator()
label_annotator = LabelAnnotator()

cap = VideoCapture(0)
xygantry = XYGantry()

ser = serial.Serial(r'/dev/ttyACM0', 115200)

# Declare weed_frame as global variable
global weed_frame

def e_stop():
    while True:
        response = ser.readline().decode().strip()
        if response == "ESTOP":
            print("[WARNING] Auto stop activated, plug and unplug mcu")
        

def get_centroid(x1, y1, x2, y2):
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    return cx, cy

def display_frame():
    while True:
        # Access the global variable weed_frame
        cv2.imshow("Output", weed_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        
estop_thread = threading.Thread(target=e_stop)
display_thread = threading.Thread(target=display_frame)
estop_thread.start()

try:
    while True:
        
        frame = cap.read()
        frame = frame[0:480, 0:640]

        results = model(frame, conf=0.8, verbose=False)
        crop_frame = results[0].plot()
        crop_boxes = results[0].boxes.xyxy.cpu().numpy()

        history = []

        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        for crop_box in crop_boxes:
            x1, y1, x2, y2 = crop_box
            m = 12
            cv2.rectangle(mask, (int(x1) - m, int(y1) - m), (int(x2) + m, int(y2) + m), (255, 255, 255), -1)

        masked_frame = frame.copy()
        masked_frame[mask == 255] = [0, 0, 0]

        results = detector.inference(masked_frame)

        detections = Detections.from_azure_analyze_image(results)
        detections = tracker.update_with_detections(detections)

        labels = [f"#{tracker_id}" for class_id, tracker_id in zip(detections.class_id, detections.tracker_id)]

        # Assign the result to the global variable
        weed_frame = box_annotator.annotate(frame.copy(), detections=detections)
        weed_frame = label_annotator.annotate(weed_frame, detections=detections, labels=labels)

        display_thread = threading.Thread(target=display_frame)
        display_thread.start()

        for tracker_id, xyxy in zip(detections.tracker_id, detections.xyxy):
            x1, y1, x2, y2 = xyxy.tolist()
            x, y = get_centroid(x1, y1, x2, y2)
            xygantry.job_update(target=tracker_id, x=x, y=y, treat_time=2)    
            xygantry.treat()
            print()
            history.append(tracker_id)

        print(f"[INFO] Treated {len(history)} weeds in {int(time.time() - start_time)} seconds")
        history.clear()
        xygantry.hard_reset()
        print()


except KeyboardInterrupt:
    pass

xygantry.hard_reset()
cv2.destroyAllWindows()
