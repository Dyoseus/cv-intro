import cv2
import numpy as np
import matplotlib.pyplot as plt
import LincolnLabsSecretSauce  as llss

cap = cv2.VideoCapture('AUV_Vid.mkv')

lineArray = []

# Read until the video is completed
while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret == True:

        lines = llss.detect_lines(frame, 49, 50, 3, 500, 40)

        if lines is not None:
            lanes = llss.detect_lanes(lines)
            #print(lanes)
            lineArray.append(lanes)
                
print(len(lineArray))

#def get_lane_center(lanes):
    

    #return center_intercept, center_slope
                

