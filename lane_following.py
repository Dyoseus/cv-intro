import cv2
import numpy as np
import matplotlib.pyplot as plt
import LincolnLabsSecretSauce  as llss

vid = cv2.VideoCapture('AUV_Vid.mkv')


# Read until the video is completed
def GetLaneList(vid):
    while vid.isOpened():
        # Capture frame-by-frame
        ret, frame = vid.read()
        if ret == True:

            lines = llss.detect_lines(frame, 49, 50, 3, 500, 40)

            if lines is not None:
                llss.draw_lines(frame, lines)

    return frame
                
def get_lane_center(img, lanes):
    """Find the center of the lane closest to the middle of a list of given lanes
    args: 
        img (image path or np.ndarray): img that lanes come from
        lanes (list): list of lanes [[[x, x, x, x], [x, x, x, x]], ...]
    return: 
        (list): [centerSlope, centerIntercept]
        """
    

    if not isinstance(img, np.ndarray):
        img = cv2.imread(img)
    # find slopes and intercepts of all lines in the lanes
    cenSlopes = []
    cenInters = []
    if lanes is not None:
        for lane in lanes:
            slope, intercept = llss.get_slopes_intercepts(img, lane)
            cenSl = 1/((1/slope[0] + 1/slope[1])/2)
            cenInt = (intercept[0] + intercept[1])/2
            cenSlopes.append(cenSl)
            cenInters.append(cenInt)
        cenInter = sorted(cenInters, key=lambda x: abs(img[1]/2 - x))[0]
        index = cenInters.index(cenInter)
        slInters = [(cenSlopes)[index], (cenInters)[index]]
    return slInters

def GetRecDirection(slInters):
    return 0


print(GetLaneList(vid))