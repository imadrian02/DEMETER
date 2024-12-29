import cv2
import numpy as np

from ultralytics import YOLO
from supervision import ByteTrack, BoundingBoxAnnotator, LabelAnnotator, Detections
from greenonbrown import GreenOnBrown


model = YOLO(r'C:\Users\adria\DEMETER\model\crop.pt')
detector = GreenOnBrown(algorithm='exg')
tracker = ByteTrack()

box_annotator = BoundingBoxAnnotator()
label_annotator = LabelAnnotator()

frame = cv2.imread(r'photo_5_2024-05-04_10-15-56.jpg')

results = model(frame, conf=0.001, verbose=False)
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

weed_frame = box_annotator.annotate(frame.copy(), detections=detections)
weed_frame = label_annotator.annotate(weed_frame, detections=detections, labels=labels)

cv2.imshow("output", weed_frame)

# # Concatenate frames horizontally
# combined_frame = np.hstack((weed_frame, crop_frame))
#
# # Display the combined frame
# cv2.imshow("Combined Output", combined_frame)

key = cv2.waitKey(0) & 0xFF

