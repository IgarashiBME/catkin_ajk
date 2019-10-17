#
time stamp
float64 iTOW
uint8 rtk_status
uint8 movingbase_status

uint32 waypoint_seq
float64 waypoint_start_x
float64 waypoint_start_y
float64 waypoint_end_x
float64 waypoint_end_y
float64 own_x
float64 own_y
float64 own_yaw

float64 tf_waypoint_x
float64 tf_waypoint_y
float64 tf_own_x
float64 tf_own_y
float64 cross_track_error

float64 Kp
float64 Ki
float64 Kd
float64 look_ahead_dist
float64 i_control_dist
float64 i_limit

float64 p
float64 i
float64 d
float64 steering_ang #[degree]
float64 linear_x  # [meter/sec]
float64 angular_z # [rad/sec]
