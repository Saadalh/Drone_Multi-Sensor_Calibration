from scipy.spatial.transform import Rotation
import pyrealsense2 as rs
import numpy as np
import scipy
import cv2 as cv

gravity = [0, -9.81, 0]
alpha = 0.95

class DepthCamera:
    def __init__(self, dir_path):
        print("Starting the realsense pipeline...")

        self.dpath = dir_path
        self.caplist = []
        self.color_image = None
        self.depth_image = None
        self.streaming = True
        
        # Define pose-related variables
        self.initial_position = [0,0,0]
        self.initial_orientation = Rotation.from_quat([0,0,0,1])
        self.gt0 = None
        self.at0 = None

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

        print("Pipeline initialized!")

    def stream(self):
        print("Streaming LIVE!")
        while self.streaming:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)

            if not depth_frame or not color_frame or not accel_frame or not gyro_frame:
                continue

            # Extract the color and depth frames, alongside the accelerometer and gyroscope data
            self.depth_image = np.asanyarray(depth_frame.get_data())
            self.color_image = np.asanyarray(color_frame.get_data())
            accel_data = accel_frame.as_motion_frame().get_motion_data()
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()

            cv.imshow("Color Image", self.color_image)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv.applyColorMap(cv.convertScaleAbs(self.depth_image, alpha=0.03), cv.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = self.color_image.shape

            # Calculate the gyro time difference
            if self.gt0 is None:
                self.gt0 = gyro_frame.get_timestamp()/1000
                gdt = 0
            else: 
                gdt = ((gyro_frame.get_timestamp()/1000) - self.gt0)
                self.gt0 = gyro_frame.get_timestamp()/1000

            # Calculate the accel time difference
            if self.at0 is None:
                self.at0 = accel_frame.get_timestamp()/1000
                adt = 0
            else: 
                adt = ((accel_frame.get_timestamp()/1000) - self.at0)
                self.at0 = accel_frame.get_timestamp()/1000

            # Get the current gyro data
            self.gyro_last = np.array([gyro_data.x, gyro_data.y, gyro_data.z])

            # Get the accelerometer data and compensate for gravity
            self.accel_last = np.array([accel_data.x, accel_data.y, accel_data.z])

            # Get the position and orientation
            position, orientation = self.update_position_orientation(adt, gdt)

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv.resize(self.color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((self.color_image, depth_colormap))
                    
    def get_sensor_info(self):
        return self.accel_last, self.gyro_last
    
    def save_frame(self, x, i):
        if i == 0:
            self.caplist = []
        cv.imwrite(f"{self.dpath}/img_{x}{i+1}.jpg", self.color_image)
        self.caplist.append(f"{self.dpath}/img_{x}{i+1}.jpg")
        print(f"Captured frame #{x}{i+1}!")

    def update_position_orientation(self, acc_dt, gyro_dt):
        a = self.accel_last
        w = self.gyro_last

         # Apply complementary filter to estimate orientation and remove gravity component from accelerometer readings
        orientation_acc = Rotation.from_matrix(np.array([[a[1], -a[0], 0], [-a[2], 0, a[0]], [0, -a[2], -a[1]]]))
        # Convert the orientation_acc Rotation object to a rotation matrix
        orientation_acc_matrix = orientation_acc.as_matrix()
        # Multiply the matrix with the scaling factor and add the initial orientation
        orientation_scaled_matrix = (1 - alpha) * orientation_acc_matrix + alpha * self.initial_orientation.as_matrix()
        # Convert the scaled matrix back to a Rotation object
        orientation_scaled = Rotation.from_matrix(orientation_scaled_matrix)
        a_comp = orientation_scaled.inv().apply(a) - gravity
#        print(a)
#        print(a_comp)

        # Integrate accelerometer readings twice to obtain change in position
        delta_p = 0.5 * a_comp * (acc_dt ** 2)
        # Update position
        position = self.initial_position + delta_p
        self.initial_position = position 

        # Integrate gyroscope readings once to obtain change in orientation
        delta_q = Rotation.from_euler('xyz', w * gyro_dt, degrees=True).as_quat()
        # Update orientation
        orientation = Rotation.from_quat(delta_q) * self.initial_orientation
        self.initial_orientation = orientation

        return position, orientation
    
    def get_caplist(self):
        return self.caplist
    
    def release(self, sthread):
        self.streaming = False
        sthread.join()
        self.pipeline.stop()

if __name__ == "__main__":
    dc = DepthCamera("path")
    dc.stream()