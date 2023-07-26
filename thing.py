import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_lines(img, threshold1, threshold2, apertureSize, minLineLength, maxLineGap):
    img = cv2.imread(str(img))

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # convert to grayscale
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize) # detect edges

    lines = cv2.HoughLinesP(edges, 5, np.pi/180, 5, minLineLength, maxLineGap) # detect lines

    return lines

def draw_lines(img, lines, color = (255, 0, 0)):
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), color, 2)

        show = plt.imshow(img)
    return show
        

def get_slopes_intercepts(lines):
    slopes = []
    intercepts = []
    for i in range(len(lines)):
        x1, y1, x2, y2 = lines[i]
        slopes.append((y2-y1)/(x2-x1))

    for i in range(len(slopes)):
        x1, y1, x2, y2 = lines[i]
        slopie = slopes[i]
        b = y1-(slopie*x1)
        intercepts.append(b)

        return intercepts
    
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
    cv2.line




