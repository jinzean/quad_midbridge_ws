std_msgs/Header header

uint8 task_mode
uint8 path_type
uint8 priority_mode

geometry_msgs/Point[] centerline_points
float32[] centerline_s

float32 corridor_half_width
float32 corridor_half_height

float32 speed_pref
float32 speed_max
float32 yaw_pref
float32 yaw_rate_pref

float32 risk_level
float32 observation_priority
float32 progress_weight
float32 contour_weight

bool terminal_hold
bool allow_reverse_progress
