#!/usr/bin/env python3.7
"""
    This is used to test the grip vision pipeline code off the robot
"""

import cv2
import numpy as np
from grip_pipeline import GripPipeline


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

cv2.namedWindow("preview")
vc = cv2.VideoCapture(0)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

while rval:
    pipeline.process(frame)

    cv2.imshow("preview", pipeline.key_points_output)

    largest_blob = find_largest_blob(pipeline.find_blobs_output)
    if largest_blob is not None:
        print('blob:', largest_blob.size, largest_blob.pt)
        print('distance from center: ', get_distance_from_center(pipeline.key_points_output, largest_blob))

    rval, frame = vc.read()
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break

cv2.destroyWindow("preview")
vc.release()
