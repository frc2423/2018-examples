# Import the camera server
from cscore import CameraServer

# Import OpenCV and NumPy
import cv2
import numpy as np
import math
from enum import Enum
from grip_pipeline import GripPipeline

from networktables import NetworkTables
from networktables.util import ntproperty

# Connect to the robot
NetworkTables.initialize(server='127.0.0.1')
sd = NetworkTables.getTable("vision")


def find_largest_blob(blobs):
    largest_blob = None

    for blob in blobs:
        if largest_blob is None or largest_blob.size < blob.size:
            largest_blob = blob

    return largest_blob

def get_distance_from_center(img, blob):
    width, height, _ = img.shape
    x, y = blob.pt
    return x - width / 2, y - height / 2

pipeline = GripPipeline()

def main():
    cs = CameraServer.getInstance()
    cs.enableLogging()

    # Capture from the first USB Camera on the system
    camera = cs.startAutomaticCapture()
    camera.setResolution(320, 240)

    # Get a CvSink. This will capture images from the camera
    cvSink = cs.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = cs.putVideo("Name", 320, 240)

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    while True:


        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, img = cvSink.grabFrame(img)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError());
            # skip the rest of the current iteration
            continue

        #
        # Insert your image processing logic here!
        #
        pipeline.process(img)

        # (optional) send some image back to the dashboard
        outputStream.putFrame(pipeline.key_points_output)

        # send information over networktables
        largest_blob = find_largest_blob(pipeline.find_blobs_output)

        if largest_blob is not None:
            sd.putBoolean('target_found', True)
            target_x, target_y = get_distance_from_center(pipeline.key_points_output, largest_blob)
            sd.putNumber('target_x', target_x)
            sd.putNumber('target_y', target_y)
            sd.putNumber('target_size', largest_blob.size)
        else:
            sd.putBoolean('target_found', False)

