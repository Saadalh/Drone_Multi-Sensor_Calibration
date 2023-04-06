import pyrealsense2 as rs
import numpy as np
import scipy
import cv2 as cv

class DepthCamera:
    def __init__(self):
        # Define pose-related variables
        self.R = np.eye(3)
        self.v = 0
        self.p = 0
        self.rotg = np.zeros((3, 1))
        self.t0 = None
        self.gyro_last = None

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

        # Start streaming
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        if not depth_frame or not color_frame or not accel_frame or not gyro_frame:
            return False, None, None, None, None

        # Extract the color and depth frames, alongside the accelerometer and gyroscope data
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        accel_data = accel_frame.as_motion_frame().get_motion_data()
        gyro_data = gyro_frame.as_motion_frame().get_motion_data()

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # Calculate the time difference
        if self.t0 is None:
            self.t0 = gyro_frame.get_timestamp()
            dt = 0
        else: 
            dt = (gyro_frame.get_timestamp() - self.t0) / 1e6
            self.t0 = gyro_frame.get_timestamp()

        # Get the rotation data
        if self.gyro_last is not None:
            omega = np.array([gyro_data.x, gyro_data.y, gyro_data.z]) - self.gyro_last
            self.R = np.dot(self.R, scipy.linalg.expm(self.cross_matrix(omega) * dt))
            
        self.gyro_last = np.array([gyro_data.x, gyro_data.y, gyro_data.z])
        #scipy.spatial.transform.Rotation.from_

        # Calculate the translation using the accelerometer data
        g = np.array([0, -9.81, 0])
        self.a = np.array([accel_data.x, accel_data.y, accel_data.z])
        self.rotg = self.R.dot(g)

        # calculate the acceleration in the world frame
        a_world = self.R.dot(self.a) - np.array([[0], [0], [9.81]])

        # integrate the acceleration to obtain the velocity
        self.v += a_world * dt

        # integrate the velocity to obtain the position
        self.p += self.v * dt

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))
        
        return True, depth_image, color_image
    
    def get_pose(self):
        return self.a, self.gyro_last
    
    def cross_matrix(self, v):
        return np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
            
    def release(self):
        self.pipeline.stop()