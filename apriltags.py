from dt_apriltags import Detector
import cv2
import numpy as np
import matplotlib.pyplot as plt


video = cv2.VideoCapture('AprilTagTest.mkv')

ret, frame = video.read()
i = 0
cameraMatrix = np.array([ 1060.71, 0, 960, 0, 1060.71, 540, 0, 0, 1]).reshape((3,3))
camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )
at_detector = Detector(families='tag36h11',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)
l = []

while ret:
    i += 1
    if(i%10 == 0 and i < 90):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        tags = at_detector.detect(img, True, camera_params, tag_size = 0.1)
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
                cv2.circle(color_img, (int(tag.center[0].item()),int(tag.center[1].item())), 50, (0, 0, 255), 2)
            l.append((tag.pose_t, tag.pose_R))
        #output_video.write(processed_frame)
    ret, frame = video.read()
print(l)
video.release()
#output_video.release()