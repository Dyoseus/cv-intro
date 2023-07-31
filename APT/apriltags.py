from dt_apriltags import Detector
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
import sys
import signal

from pid import PID

def set_rc_channel_pwm(mav, channel_id, pwm=1500):
    """Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    mav.mav.rc_channels_override_send(
        mav.target_system,  # target_system
        mav.target_component,  # target_component
        *rc_channel_values
    )


def set_vertical_power(mav, power=0):
    """Set vertical power
    Args:
        power (int, optional): Vertical power value -100-100
    """
    if power < -100 or power > 100:
        print("Power value out of range. Clipping...")
        power = np.clip(power, -100, 100)

    power = int(power)

    set_rc_channel_pwm(mav, 3, 1500 + power * 5)


def press_to_depth(pressure):
    """Convert pressure to depth
    Args:
        pressure (float): Pressure in hPa
    Returns:
        float: Depth in water in meters
    """
    rho = 1029  # density of fresh water in kg/m^3
    g = 9.81  # gravity in m/s^2
    pressure_at_sea_level = 1013.25  # pressure at sea level in hPa
    # multiply by 100 to convert hPa to Pa
    return (pressure - pressure_at_sea_level) * 100 / (rho * g)


def readframes(video):
    video = cv2.VideoCapture(f'{video}')

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
#video.release()
#output_video.release()

def size(video):
    vcap = cv2.VideoCapture(f'{video}')
    width  = vcap.get(3)  # float `width`
    height = vcap.get(4)  

    middle_y = width/2
    middle_x = height/2

    return middle_y, middle_x


def main():

    mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    pid = PID(45, 0.0, 10.0, 2)
    video = 'AprilTagTest.mkv'
    middle_y, middle_x = size(video)
    readframes(video)

    

    while True:
        # Read the current coordinates from the AprilTag detector
        current_y, current_x = readframes(video)

        # Calculate error from the desired middle coordinates
        error_y = middle_y - current_y
        error_x = middle_x - current_x

        # Update the PID controllers and get the output
        output_y = pid.update(error_y)
        output_x = pid.update(error_x)

        # Set vertical power using the PID output
        set_vertical_power(mav, -output_y)  # Negative because of the direction of the thruster

        # Set horizontal power using the PID output
        set_rc_channel_pwm(mav, 6, pwm=1500 + output_x)

    
