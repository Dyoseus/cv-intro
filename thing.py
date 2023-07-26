import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_lines(img, threshold1 = 50, threshold2 = 150, apertureSize = 3, minLineLength = 100, maxLineGap = 10):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # convert to grayscale
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize = apertureSize) # detect edges, gray is image in grayscale, 50 and 150 represent 2 images that have been 
                                                      # threshholded at 2 different levels, apertureSize controls how much light the image gets and how exposed it is
    lines = cv2.HoughLinesP(
                    edges, #described above
                    rho = 1, #1 pixel resolution parameter
                    theta = np.pi/180, # 1 degree resolution parameter
                    threshold = 125, #min number of intersections/votes
                    minLineLength = minLineLength,
                    maxLineGap = maxLineGap,
            ) # detect lines


    return lines

def draw_lines(img, lines, color=(255, 0, 0)):
    try:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), color, 2)
    except TypeError:
        pass
    return img
        

def get_slopes_intercepts(lines):
    slopes = []
    intercepts = []
    for line in lines:
        x1, y1, x2, y2 = line
        slopie = (y2 - y1) / (x2 - x1)
        slopes.append(slopie)
        b = y1 - (slopie * x1)
        intercepts.append(b)
    return slopes, intercepts

def detect_lanes(lines):
    i = 0
    lanes = []
    slopes, x_intercepts = get_slopes_intercepts(lines)

    while i < range(slopes):
        if slopes[i] - slopes[i+1] == range (-0.3, 0.3) or slopes[i+1] - slopes[i] == range (-0.3, 0.3):
            lanes.append([lines[i], lines[i+1]])
            i+=2
        else:
            if np.abs(slopes[i+1] - slopes[i]) < 2 and np.abs(x_intercepts[i+1]-[i]) < 200:
                lanes.append([lines[i], lines[i+1]])
                i+=2
            else:
                i+=1

    return lanes

def draw_lanes(img, lanes):
    return 0




