import cv2
import numpy as np
from ultralytics import YOLO
from supervision import ByteTrack, BoundingBoxAnnotator, LabelAnnotator, Detections

class WeedDetectionPipeline:
    def __init__(self, model_path, algorithm='exg'):
        self.model = YOLO(model_path)
        self.tracker = ByteTrack()
        self.box_annotator = BoundingBoxAnnotator()
        self.label_annotator = LabelAnnotator()

    def process_frame_with_steps(self, frame):
        # 1. Original Image
        original_frame = frame.copy()

        # 2. Grayscale Conversion
        grayscale_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 3. Binary Thresholding
        _, binary_frame = cv2.threshold(grayscale_frame, 120, 255, cv2.THRESH_BINARY_INV)

        # 4. Morphological Processing
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        morph_frame = cv2.morphologyEx(binary_frame, cv2.MORPH_CLOSE, kernel, iterations=2)

        return original_frame, grayscale_frame, binary_frame, morph_frame

if __name__ == "__main__":
    # Load image
    image_path = '/mnt/data/image.png'
    frame = cv2.imread(r'C:\Users\adria\DEMETER\test2.jpg')

    if frame is None:
        print("[ERROR] Failed to load image.")
    else:
        # Initialize pipeline
        pipeline = WeedDetectionPipeline(model_path='C:\\Users\\adria\\DEMETER\\model\\best.pt')

        # Process frame with all steps
        original, grayscale, binary, morph = pipeline.process_frame_with_steps(frame)

        # Display all results
        cv2.imshow("Original Image", original)
        cv2.imshow("Grayscale Image", grayscale)
        cv2.imshow("Binary Image", binary)
        cv2.imshow("Morphological Processing", morph)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
