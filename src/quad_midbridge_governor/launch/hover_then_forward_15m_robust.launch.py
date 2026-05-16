import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mpcc_params = os.path.join(
        get_package_share_directory('quad_midbridge_mpcc'),
        'config',
        'mpcc_solver.yaml'
    )

    governor_params = os.path.join(
        get_package_share_directory('quad_midbridge_governor'),
        'config',
        'governor_v2.yaml'
    )

    return LaunchDescription([
        Node(
            package='quad_midbridge_mpcc',
            executable='mpcc_node',
            name='mpcc_node',
            output='screen',
            parameters=[mpcc_params],
        ),

        Node(
            package='quad_midbridge_governor',
            executable='governor_node',
            name='governor_node',
            output='screen',
            parameters=[
                governor_params,
                {
                    'takeoff_target_height_m': 1.5,
                    'takeoff_release_height_m': 1.10,
                    'takeoff_release_z_error_abs_max': 0.45,
                    'takeoff_release_vz_abs_max': 0.20,
                    'takeoff_release_vxy_abs_max': 0.25,
                    'takeoff_release_stable_time_sec': 0.8,
                    'takeoff_kp_z': 1.8,
                    'takeoff_kd_z': 2.6,
                    'takeoff_accel_z_min': -0.8,
                    'takeoff_accel_z_max': 1.00,
                    'post_takeoff_blend_time_sec': 12.0,
                    'takeoff_kp_xy': 0.60,
                    'takeoff_kd_xy': 0.90,
                    'takeoff_accel_xy_max': 0.45,
                    'height_mode': 'takeoff_relative',
                },
            ],
        ),

        Node(
            package='quad_midbridge_flatness_mapper',
            executable='flatness_mapper_node',
            name='flatness_mapper_node',
            output='screen',
            parameters=[{
                'input_reference_topic': '/midbridge/governed_reference',
                'thrust_scale': 14.4,
                'max_collective_thrust': 0.90,
                    'publish_flatness_debug': True,
                    'flatness_debug_topic': '/midbridge/flatness_debug',
                    'flatness_debug_throttle_sec': 0.10,
                    'enable_thrust_adaptation': True,
                    'hover_thrust_norm_init': 0.68,
                    'hover_thrust_norm_min': 0.45,
                    'hover_thrust_norm_max': 0.78,
                    'hover_thrust_adapt_rate': 0.035,
                    'hover_thrust_error_deadband_m': 0.03,
                    'hover_thrust_error_clip_m': 0.35,
                    'hover_thrust_vz_gain_sec': 0.35,
                    'adapt_max_ref_speed': 0.25,
                    'adapt_max_actual_speed': 0.45,
                    'adapt_max_lateral_accel': 1.25,
                    'adapt_max_vertical_accel': 0.12,
                    'adapt_max_abs_z_error_m': 0.35,
                    'enable_overshoot_thrust_guard': True,
                    'overshoot_guard_z_error_m': 0.45,
                    'overshoot_guard_vz_up_mps': 0.35,
                    'overshoot_guard_thrust_max': 0.58,
                    'adapt_z_reset_pause_sec': 1.0,
                    'local_pos_timeout_sec': 0.5,
            }],
        ),

        Node(
            package='quad_midbridge_attitude_offboard',
            executable='attitude_offboard_node',
            name='attitude_offboard_node',
            output='screen',
            parameters=[{
                'publish_rate_hz': 190.0,
                'ref_timeout_sec': 0.5,
                'hard_timeout_sec': 5.0,
                'hold_last_valid_on_timeout': True,
                'auto_arm': False,
                'auto_set_mode': False,
                'require_valid_reference': True,
            }],
        ),

        # Replaces the old pkill-based LocalIntent switching. The manager keeps a
        # single publisher alive and latches anchors at stage transitions:
        # hover -> forward -> decelerate -> final_hold.
        Node(
            package='quad_midbridge_governor',
            executable='scripted_local_intent_manager_node',
            name='scripted_local_intent_manager_node',
            output='screen',
            parameters=[{
                'pub_rate': 10.0,
                'target_z': -1.5,
                'corridor_half_height': 1.5,
                'corridor_half_width': 1.0,
                'path_spacing': 0.4,
                'hover_duration_sec': 45.0,
                'forward_duration_sec': 67.0,
                'decel_duration_sec': 8.0,
                'hover_num_points': 4,
                'forward_num_points': 8,
                'final_hold_num_points': 4,
                'forward_speed_pref': 0.06,
                'forward_speed_max': 0.10,
                'forward_progress_weight': 0.12,
                'contour_weight': 0.5,
            }],
        ),
    ])
