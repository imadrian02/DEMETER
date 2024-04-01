from algorithms import exg, exg_standardised, exg_standardised_hue, hsv, exgr, gndvi, maxg
from imutils import grab_contours
import numpy as np
import cv2


class GreenOnBrown:
    def __init__(self, algorithm='exg', label_file='models/labels.txt'):
        self.algorithm = algorithm

    def inference(self,
                  image,
                  exgMin=30,
                  exgMax=250,
                  hueMin=30,
                  hueMax=90,
                  brightnessMin=5,
                  brightnessMax=200,
                  saturationMin=30,
                  saturationMax=255,
                  minArea=10,
                  show_display=False,
                  algorithm='exg',
                  invert_hue=False):
        '''
        Uses a provided algorithm and contour detection to determine green objects in the image. Min and Max
        thresholds are provided.
        :param image: input image to be analysed
        :param exgMin: minimum exG threshold value
        :param exgMax: maximum exG threshold value
        :param hueMin: minimum hue threshold value
        :param hueMax: maximum hue threshold value
        :param brightnessMin: minimum brightness threshold value
        :param brightnessMax: maximum brightness threshold value
        :param saturationMin: minimum saturation threshold value
        :param saturationMax: maximum saturation threshold value
        :param minArea: minimum area for the detection - used to filter out small detections
        :param show_display: True: show windows; False: operates in headless mode
        :param algorithm: the algorithm to use. Defaults to ExG if not correct
        :return: returns the contours, bounding boxes, centroids and the image on which the boxes have been drawn
        '''

        # different algorithm options, add in your algorithm here if you make a new one!
        threshedAlready = False
        if algorithm == 'exg':
            output = exg(image)

        elif algorithm == 'exgr':
            output = exgr(image)

        elif algorithm == 'maxg':
            output = maxg(image)

        elif algorithm == 'nexg':
            output = exg_standardised(image)

        elif algorithm == 'exhsv':
            output = exg_standardised_hue(image, hueMin=hueMin, hueMax=hueMax,
                                          brightnessMin=brightnessMin, brightnessMax=brightnessMax,
                                          saturationMin=saturationMin, saturationMax=saturationMax,
                                          invert_hue=invert_hue)

        elif algorithm == 'hsv':
            output, threshedAlready = hsv(image, hueMin=hueMin, hueMax=hueMax,
                                          brightnessMin=brightnessMin, brightnessMax=brightnessMax,
                                          saturationMin=saturationMin, saturationMax=saturationMax,
                                          invert_hue=invert_hue)

        elif algorithm == 'gndvi':
            output = gndvi(image)

        else:
            output = exg(image)
            print('[WARNING] DEFAULTED TO EXG')

        # run the thresholds provided
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.weedCenters = []
        self.boxes = []

        # if not a binary image, run an adaptive threshold on the area that fits within the thresholded bounds.
        if not threshedAlready:
            output = np.where(output > exgMin, output, 0)
            output = np.where(output > exgMax, 0, output)
            output = np.uint8(np.abs(output))
            if show_display:
                cv2.imshow("HSV Threshold on ExG", output)

            thresholdOut = cv2.adaptiveThreshold(output, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31,
                                                 2)
            thresholdOut = cv2.morphologyEx(thresholdOut, cv2.MORPH_CLOSE, kernel, iterations=1)

        # if already binary, run morphological operations to remove any noise
        if threshedAlready:
            thresholdOut = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel, iterations=5)

        if show_display:
            cv2.imshow("Binary Threshold", thresholdOut)

        # find all the contours on the binary images
        self.cnts = cv2.findContours(thresholdOut.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.cnts = grab_contours(self.cnts)

        # loop over all the detected contours and calculate the centres and bounding boxes
        for c in self.cnts:
            # filter based on total area of contour
            if minArea < cv2.contourArea(c) < 2500:
                # calculate the min bounding box
                startX, startY, boxW, boxH = cv2.boundingRect(c)
                endX = startX + boxW
                endY = startY + boxH

                # save the bounding box
                self.boxes.append([startX, startY, boxW, boxH])
                # compute box center
                centerX = int(startX + (boxW / 2))
                centerY = int(startY + (boxH / 2))
                self.weedCenters.append([centerX, centerY])

        # Convert your detection results into the format expected by from_azure_analyze_image()
        detections = []
        for box, _ in zip(self.boxes, self.weedCenters):
            x, y, w, h = box
            confidence = 1.0  # You may adjust this based on your confidence estimation
            class_id = 0  # Assuming there's only one class for weed detection
            detections.append({
                "boundingBox": {"x": x, "y": y, "w": w, "h": h},
                "tags": [{"name": "weed", "confidence": confidence}]
            })

        azure_result = {"objectsResult": {"values": detections}}

        # Return the result compatible with from_azure_analyze_image()
        return azure_result
