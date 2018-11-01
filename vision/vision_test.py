#!/usr/bin/env python3.7
"""
    This is used to test the grip vision pipeline code off the robot
"""

import cv2
from grip_pipeline import GripPipeline


pipeline = GripPipeline()

cv2.namedWindow("preview")
vc = cv2.VideoCapture(0)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

while rval:
    pipeline.process(frame)
    print(pipeline.cv_resize_output)
    cv2.imshow("preview", pipeline.mask_output)
    rval, frame = vc.read()
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break

cv2.destroyWindow("preview")
vc.release()