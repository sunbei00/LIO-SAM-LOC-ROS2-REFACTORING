# Cloud Info
std_msgs/Header header 

int32[] start_ring_index                # start point's index in ring
int32[] end_ring_index                  # end point's index in ring
                                        # because of lidar problem

int32[]  point_col_ind                  # point column index in range image
float32[] point_range                   # point range (distance)

# it means that imu_*_init is available
int64 imu_available
# it means that initial_guess_* is available
int64 odom_available

# Attitude for LOAM initialization
# it is set by imageProjection.cpp - imuDeskewInfo()
# imu orientation at starting time in laser scan
float32 imu_roll_init
float32 imu_pitch_init
float32 imu_yaw_init

# Initial guess from imu pre-integration
# it is set by imageProjection.cpp - odomDeskewInfo()
# orientation at starting time in /odometry/incremental
float32 initial_guess_x
float32 initial_guess_y
float32 initial_guess_z
float32 initial_guess_roll
float32 initial_guess_pitch
float32 initial_guess_yaw

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature
