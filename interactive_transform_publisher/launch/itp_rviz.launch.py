import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription([
    Node(
      package='interactive_transform_publisher',
      executable='interactive_transform_publisher',
      name='interactive_transform_publisher',
      parameters=[{
        'frame_id':'world',
        'child_frame_id':'my_frame',
        'x':0.0,
        'y':0.0,
        'z':0.0,
        'qx':0.0,
        'qy':0.0,
        'qz':0.0,
        'qw':1.0,
        'scale':0.2,
      }],
      output='screen'
    ),
    Node(
        package='rviz2',
        executable='rviz2',
        name='itp_rviz2',
        output='screen',
        arguments=[["-d"],[os.path.join(get_package_share_directory('interactive_transform_publisher'), 'itp.rviz')]]
    ),
  ])
