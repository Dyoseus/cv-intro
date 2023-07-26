import cv2
import numpy as np
import matplotlib.pyplot as plt

cap = cv2.VideoCapture('AUV_Vid.mkv')

lineArray = []

# Read until the video is completed
while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret == True:

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        # Apply Hough Line Transform
        lines = cv2.HoughLinesP(edges, 5, np.pi/180, 5, minLineLength=300, maxLineGap=200)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                slope = (y2 - y1) / (x2 - x1)
                print(slope)
                
                '''if x2 - x1 != 0:
                    slope = (y2 - y1) / (x2 - x1)
                    lineArray.append(slope)
                else:
                    x2 = 0.000000000001
                    x1 = 0
                    slope = (y2 - y1) / (x2 - x1)
                    lineArray.append(slope)'''

