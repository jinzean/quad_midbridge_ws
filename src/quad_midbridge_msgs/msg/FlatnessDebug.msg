std_msgs/Header header

# Differential-flatness force mapping diagnostics in PX4 NED/FRD convention.
float32 desired_force_norm
float32 desired_tilt_rad
float32 desired_tilt_deg

# PX4 normalized collective thrust before and after saturation.
float32 collective_thrust_raw
float32 collective_thrust_clamped
float32 hover_thrust_norm_est
float32 nominal_hover_thrust_norm
float32 thrust_scale

# Reference and measured vertical tracking quantities. NED z grows downward.
float32 desired_z
float32 local_z
float32 local_vz
float32 z_error_up
float32 adapt_signal
float32 adapt_dt

# Gate quantities for online hover-thrust adaptation.
float32 ref_speed
float32 actual_speed
float32 lateral_accel
float32 vertical_accel

bool thrust_adaptation_enabled
bool thrust_adaptation_active
bool local_position_valid
bool z_reset_paused
bool thrust_saturated

string adaptation_status
