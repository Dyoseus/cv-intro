from dt_apriltags import Detector
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
import sys
import signal

from pid import PID

cameraMatrix = np.array([1060.71, 0, 960, 0, 1060.71, 540, 0, 0, 1]).reshape((3, 3))
camera_params = (cameraMatrix[0, 0], cameraMatrix[1, 1], cameraMatrix[0, 2], cameraMatrix[1, 2])
at_detector = Detector(families='tag36h11',
                           nthreads=1,
                           quad_decimate=1.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)

def getTag1(frame, detected_tags):
    print("start get tag")
    # video.set(cv2.CAP_PROP_POS_FRAMES, frameNumber)
    
    print("video read")
    
    # detected_tags = []
    print("finish camera params/before ret")
    
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(img, True, camera_params, tag_size=0.1)
    for tag in tags:
        x, y = int(tag.center[0]), int(tag.center[1])
        detected_tags.append([x, y])
        for idx in range(len(tag.corners)):
            cv2.line(frame, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
            cv2.circle(frame, (x, y), 50, (0, 0, 255), 2)
    print(detected_tags)
        
    return detected_tags


def size(video):
    vcap = cv2.VideoCapture(video)
    
    width = int(vcap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(vcap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    middle_y = width/2
    middle_x = height/2

    return middle_y, middle_x



def main():
    video = '/home/albertle/cv-intro/APT/AprilTagTestMini.mp4'
    #mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    pid = PID(35, 0.0, 10, 2)

    middle_y, middle_x = size(video)    
    x = 0

    try:
        video = cv2.VideoCapture(video)
        while True:
            # Read the current coordinates from the AprilTag detector
            print("Before getTag function")
            ret, frame = video.read()
            detected_tags = []
            if ret:
                detected_tags = getTag1(frame, detected_tags)
            else:
                continue
            print("After getTag function")
            # if not detected_tags:
            #     print("No tags found in frame", x)
            #     break

            # For simplicity, assume only one tag is detected in each frame
            current_x, current_y = detected_tags[0]

            # Calculate error from the desired middle coordinates
            error_y = middle_y - current_y
            error_x = middle_x - current_x

            print("Frame:", x)
            print("Error Y:", error_y)
            print("Error X:", error_x)

            # Update the PID controllers and get the output
            # output_y = pid.update(error_y)
            # output_x = pid.update(error_x)

            # print("Output Y:", output_y)
            # print("Output X:", output_x)

            # Set vertical power using the PID output
            #set_vertical_power(mav, -output_y)  # Negative because of the direction of the thruster

            # Set horizontal power using the PID output
            #set_rc_channel_pwm(mav, 6, pwm=1500 + output_x)

            x=x+1
            
    except KeyboardInterrupt:
        print("Interrupted by user.")
        # finally:
        # Stop the vehicle's movement when the program ends
        # set_vertical_power(mav, 0)
        # set_rc_channel_pwm(mav, 6, pwm=1500)


    
if __name__ == "__main__":
    main()