# Binosense General Data Message
# This message contains synchronized data from the bionic eye device
# including images, motor states, IMU, GPS, etc.

std_msgs/Header header

# Frame ID and timestamp
uint64 frame_id
uint64 timestamp

# Stereo images (0-3: right, left, both, depth)
# Note: Only valid images are published
sensor_msgs/Image left_image
sensor_msgs/Image right_image
sensor_msgs/Image both_image

# Camera info
sensor_msgs/CameraInfo left_camera_info
sensor_msgs/CameraInfo right_camera_info

# Motor data (6 motors: right_pitch, right_roll, right_yaw, left_pitch, left_roll, left_yaw)
float32[6] motor_data

# IMU data (16 sensors x 4 values, flattened to 64 elements)
float32[64] imu_data

# GPS data
float64[3] gps_data    # longitude, latitude, height
int64 gps_time          # GPS UTC time

# Motion flag
int64 is_moving_fastly
