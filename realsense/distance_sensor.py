import pyrealsense2 as rs
from realsense_depth import *
import cv2 as cv
import numpy as np

cpoint = (0, 0)

def get_position(event, x, y, args, params):
    global cpoint
    cpoint = (x, y)

# Initialize realsense
dc = DepthCamera()
print("Connected")

cv.namedWindow("Color frame", cv.WINDOW_AUTOSIZE)
cv.setMouseCallback("Color frame", get_position)

try:
    while True:
        ret, depth_frame, color_frame = dc.get_frame()

        # Draw a circle around the point to be measured
        center_point = (240, 320)
        point_distance = depth_frame[center_point[1], center_point[0]]
        cursor_distance = depth_frame[cpoint[1], cpoint[0]]
        cv.circle(color_frame, center_point, 4, (0,0,255)) 
        cv.putText(color_frame, str(point_distance)+"mm", (center_point[0]-8, center_point[1]+38), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2) 
        cv.putText(color_frame, str(cursor_distance)+"mm", (cpoint[0]-8, cpoint[1]+8), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2) 

        # Show images
        cv.imshow('Color frame', color_frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            break

finally:

    # Stop streaming
    dc.release()