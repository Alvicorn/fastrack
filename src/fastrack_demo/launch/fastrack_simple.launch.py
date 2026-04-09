from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # params_file = LaunchConfiguration("params_file")
    params_file = LaunchConfiguration("params_file")
    default_tracker_params = {
        'min_lookahead': 0.05,
        'max_lookahead': 0.5,
    }
    default_hybrid_tracking_controller_params = {
        'min_v': 0.0,
        'min_v_hat': 0.0,
        'max_v': 0.5,
        'max_v_hat': 0.5,
        'max_omega': 1.5,
        'max_omega_hat': 1.5,
        'max_a': 0.5,
        'max_alpha': 0.5,
        'dt': 0.01,
        'safety_threshold': 0.05
    }
    default_planner_params = {
        'path_length': 5.0,
        'goal_x': 1.0,
        'goal_y': -0.5
    }

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value="", description="path to parameters file"),
        Node(
            package='fastrack_demo',
            executable='simple_planner',
            name='simple_planner',
            parameters=[params_file] if params_file != "" else [default_planner_params]
        ),
        Node(
            package='fastrack_demo',
            executable='simple_tracker',
            name='simple_tracker',
            parameters=[params_file] if params_file != "" else [default_tracker_params]
        ),
        Node(
            package='fastrack_demo',
            executable='hybrid_tracking_controller',
            name='hybrid_tracking_controller',
            parameters=[params_file] if params_file != "" else [default_hybrid_tracking_controller_params]
        )

    ])
