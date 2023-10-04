import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  pkg_dir = get_package_share_directory('bagtube_server')

  return LaunchDescription([
    Node(
      package='usb_cam',
      executable='usb_cam_node_exe',
      name='usb_cam',
      output='screen',
    ),
    Node(
      package='demo_nodes_cpp',
      executable='talker',
      name='talker',
      output='screen',
    ),
    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', os.path.join(pkg_dir, 'rviz', 'chatter_camera.rviz')],
      output='screen',
    ),

    Node(
      package='bagtube_server',
      executable='bagtube_server',
      name='bagtube_server',
      parameters=[
        os.path.join(pkg_dir, 'params', 'chatter_camera.yaml'),
      ],
      output='screen',
    ),
  ])
