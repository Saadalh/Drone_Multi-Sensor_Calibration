from realsense_depth import *
import cv2 as cv

dc = DepthCamera()

try:
    while True:
        # Get frame and pose and view the color frame
        ret, depth_frame, color_frame, posit, orient = dc.get_frame()

        cv.imshow("stream", color_frame)
        
        # Get the sensor information and print the print
        t, R = dc.get_sensor_info()
        #print(f"Linear Acceleration (ax, ay, az):\n ({t})")
        #print(f"Rotational Velocity (wx, wy, wz):\n ({R})")
        #print(f"Linear Position (x, y, z):\n ({round(posit[0], 3)}, {round(posit[1], 3)}, {round(posit[2], 3)})")
        #print(f"Orientation (rx, ry, rz):\n ({round(orient.as_euler('xyz')[0], 3)}, {round(orient.as_euler('xyz')[1], 3)}, {round(orient.as_euler('xyz')[2], 3)})")
        if cv.waitKey(100) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            break


finally:
    # Stop streaming
    dc.release()