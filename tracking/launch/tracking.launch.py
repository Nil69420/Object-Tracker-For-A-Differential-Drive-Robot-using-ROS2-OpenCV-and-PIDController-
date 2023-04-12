import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('tracking'),
        'config',
        'detector.yaml'
        )

    object_tracker_cmd = Node(
        package='tracking',
        executable='object_tracker',
        parameters=[{
          'use_sim_time': True
        }, params_file],
        remappings=[
          ('input_image', '/head_front_camera/rgb/image_raw'),
          ('joint_state', '/head_controller/state'),
          ('joint_command', '/head_controller/joint_trajectory')
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add any actions
    ld.add_action(object_tracker_cmd)

    return ld