import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  ros2_tutorial_urdf_path =   get_package_share_path('ros2_tutorial_urdf')
  #default_model_path =        ros2_tutorial_urdf_path / 'diff2.urdf.xml'
  default_rviz_config_path =  ros2_tutorial_urdf_path / 'diff2.rviz'

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  urdf_file_name = 'diff2.urdf.xml'
  urdf = os.path.join(
    get_package_share_directory('ros2_tutorial_urdf'),
    urdf_file_name)
  with open(urdf, 'r') as infp:
    robot_desc = infp.read()
  
  rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                  description='Absolute path to rviz config file')
  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='false',
      description='Use simulation (Gazebo) clock if true'),
    Node(
      package=    'robot_state_publisher',
      executable= 'robot_state_publisher',
      name=       'robot_state_publisher',
      output=     'screen',
      parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
      arguments=[urdf]),
    Node(
      package=    'ros2_tutorial_urdf', # Changed package name
      executable= 'diff2_state_publisher',
      name=       'state_publisher',
      output=     'screen'),
    rviz_arg,
    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    ),
  ])