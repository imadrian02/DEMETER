import cv2
import numpy as np
import math
from ultralytics import YOLO
from supervision import ByteTrack, BoundingBoxAnnotator, LabelAnnotator, Detections
from algorithms import exg, exg_standardised, exg_standardised_hue, hsv, exgr, gndvi, maxg
from imutils import grab_contours

class GreenOnBrown:
    def __init__(self, algorithm='exg'):
        self.algorithm = algorithm

    def inference(self, image, exgMin=22, exgMax=250, hueMin=30, hueMax=90, brightnessMin=5, brightnessMax=200,
                  saturationMin=30, saturationMax=255, minArea=10, show_display=False, invert_hue=False):
        threshedAlready = False
        if self.algorithm == 'exg':
            output = exg(image)
        elif self.algorithm == 'exgr':
            output = exgr(image)
        elif self.algorithm == 'maxg':
            output = maxg(image)
        elif self.algorithm == 'nexg':
            output = exg_standardised(image)
        elif self.algorithm == 'exhsv':
            output = exg_standardised_hue(image, hueMin=hueMin, hueMax=hueMax, brightnessMin=brightnessMin,
                                          brightnessMax=brightnessMax, saturationMin=saturationMin,
                                          saturationMax=saturationMax, invert_hue=invert_hue)
        elif self.algorithm == 'hsv':
            output, threshedAlready = hsv(image, hueMin=hueMin, hueMax=hueMax, brightnessMin=brightnessMin,
                                          brightnessMax=brightnessMax, saturationMin=saturationMin,
                                          saturationMax=saturationMax, invert_hue=invert_hue)
        elif self.algorithm == 'gndvi':
            output = gndvi(image)
        else:
            output = exg(image)
            print('[WARNING] DEFAULTED TO EXG')

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.weedCenters = []
        self.boxes = []

        if not threshedAlready:
            output = np.where(output > exgMin, output, 0)
            output = np.where(output > exgMax, 0, output)
            output = np.uint8(np.abs(output))

            thresholdOut = cv2.adaptiveThreshold(output, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 2)
            thresholdOut = cv2.morphologyEx(thresholdOut, cv2.MORPH_CLOSE, kernel, iterations=1)
        else:
            thresholdOut = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel, iterations=5)

        self.cnts = cv2.findContours(thresholdOut.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.cnts = grab_contours(self.cnts)

        for c in self.cnts:
            if 25 < cv2.contourArea(c) < 5000:
                startX, startY, boxW, boxH = cv2.boundingRect(c)
                endX = startX + boxW
                endY = startY + boxH
                self.boxes.append([startX, startY, boxW, boxH])
                centerX = int(startX + (boxW / 2))
                centerY = int(startY + (boxH / 2))
                self.weedCenters.append([centerX, centerY])

        detections = []
        for box, _ in zip(self.boxes, self.weedCenters):
            x, y, w, h = box
            confidence = 1.0
            detections.append({"boundingBox": {"x": x, "y": y, "w": w, "h": h},
                               "tags": [{"name": "weed", "confidence": confidence}]})

        azure_result = {"objectsResult": {"values": detections}}

        if show_display:
            cv2.imshow("Algorithm Output", output)
            cv2.imshow("Threshold Binary", thresholdOut)
            cv2.waitKey(1)

        return azure_result, thresholdOut, output

class WeedDetectionPipeline:
    def __init__(self, model_path, algorithm='exg'):
        self.model = YOLO(model_path)
        self.detector = GreenOnBrown(algorithm=algorithm)
        self.tracker = ByteTrack()
        self.box_annotator = BoundingBoxAnnotator()
        self.label_annotator = LabelAnnotator()

    def process_frame(self, frame):
        # Run YOLO detection
        results = self.model(frame, conf=0.01, verbose=False)
        crop_boxes = results[0].boxes.xyxy.cpu().numpy()

        # Mask detected regions
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        for crop_box in crop_boxes:
            x1, y1, x2, y2 = crop_box
            m = 20
            cv2.rectangle(mask, (int(x1) - m, int(y1) - m), (int(x2) + m, int(y2) + m), (255, 255, 255), -1)

        masked_frame = frame.copy()
        masked_frame[mask == 255] = [0, 0, 0]

        # Run GreenOnBrown detection
        detection_results, thresholdOut, algorithm_output = self.detector.inference(masked_frame, show_display=False)

        # Parse results
        detections = Detections.from_azure_analyze_image(detection_results)
        detections = self.tracker.update_with_detections(detections)

        labels = [f"#{tracker_id}" for class_id, tracker_id in zip(detections.class_id, detections.tracker_id)]

        # Annotate the frame
        weed_frame = self.box_annotator.annotate(frame.copy(), detections=detections)
        weed_frame = self.label_annotator.annotate(weed_frame, detections=detections, labels=labels)

        return frame, masked_frame, algorithm_output, thresholdOut, weed_frame

# Example usage
if __name__ == "__main__":
    pipeline = WeedDetectionPipeline(model_path='C:\\Users\\adria\\DEMETER\\model\\best.pt', algorithm='exg')

    frame = cv2.imread(r'C:\Users\adria\DEMETER\test2.jpg')
    if frame is None:
        print("[ERROR] Failed to load frame.")
    else:
        original, masked, algo_output, threshold, weed = pipeline.process_frame(frame)

        cv2.imshow("Original Frame", original)
        cv2.imshow("Masked Frame", masked)
        #cv2.imshow("Algorithm Output", algo_output)
        cv2.imshow("Threshold Binary", threshold)
        cv2.imshow("Weed Frame", weed)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
