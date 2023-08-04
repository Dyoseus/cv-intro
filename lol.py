from dt_apriltags import Detector
import cv2
import numpy as np
import matplotlib.pyplot as plt

img1 = 'IMG_4827.jpg'
img2 = 'IMG_4828.jpg'
img3 = 'IMG_4829.jpg'

at_detector = Detector(families='tag36h11',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)



img = cv2.imread(img1)
gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

tags = at_detector.detect(gray_image, True, camera_params=None, tag_size=0.1)


tag_id = tags.tag_id()  # Access the tag_id of the first element in the list
print(f"Detected tag ID: {str(tag_id)}")  # Convert tag_id to a string before printing
