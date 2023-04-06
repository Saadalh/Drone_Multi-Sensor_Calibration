from realsense_depth import *
import cv2 as cv

dc = DepthCamera()

try:
    while True:
        ret, depth_frame, color_frame = dc.get_frame()
        cv.imshow("stream", color_frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            break
        
        t, R = dc.get_pose()
        print(f"Translation:\n ({t})")
        print(f"Orientation:\n ({R})")
        if cv.waitKey(100) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            break


finally:
    # Stop streaming
    dc.release()