# ===================================================================================
# Description: Launch file for autonomous navigation of TAS car via NAV2.
# Author: Salman Bari
# Date: Oct 29, 2022
# ===================================================================================
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

 # Constants for paths to different files and folders
  package_name = 'tas2-simulator'
  robot_name_in_model = 'tas_car'
  gazebo_models_path = 'models/gazeboModels'
  gazebo_world_file_path = 'models/worlds/basement.world'
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros') 
  pkg_nav2 = FindPackageShare(package='nav2_bringup').find('nav2_bringup')   
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  urdf_file_path = os.path.join(pkg_share, 'models/urdf/tas_car.urdf')
  sdf_file_path = os.path.join(pkg_share, 'models/urdf/tas_car.sdf')
  rviz_config_file_path = os.path.join(pkg_share, 'rviz/navigationTAScar.rviz')
  nav2_config_file_path = os.path.join(pkg_share, 'config/navigation.yaml')
  map_config_file_path = os.path.join(pkg_share, 'maps/LSR_N5_basement.yaml')
  gazebo_world_path = os.path.join(pkg_share, gazebo_world_file_path)
  gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
  
  # Pose where we want to spawn the robot in Gazebo
  spawn_at_x = '-2.0'
  spawn_at_y = '-2.0'
  spawn_at_z = '0.0'
  spawn_at_yaw = '0'

  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')
  world = LaunchConfiguration('world')
  urdf = open(urdf_file_path).read()
  
  # Declare the launch arguments  
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=rviz_config_file_path,
    description='Full path to the RVIZ config file to use')

  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=urdf_file_path, 
    description='Absolute path to robot urdf file')
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=gazebo_world_path,
    description='Full path to the world model file to load')
  
   # Subscribe to the joint states of the robot and publish.    
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': urdf
                }
            ]
        )

  # Start Gazebo 
  start_gazebo_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
      launch_arguments={
      'use_sim_time': use_sim_time,
      'world': world
    }.items())

  # Spawn the robot in Gazebo
  spawn_entity_cmd = Node(
     package='gazebo_ros', 
     executable='spawn_entity.py',
     name='spawn_entity',
     arguments=['-entity', robot_name_in_model, 
                 '-file', sdf_file_path,
                 '-x', spawn_at_x,
                  '-y', spawn_at_y,
                  '-z', spawn_at_z,
                  '-Y', spawn_at_yaw],
                     output='screen')

  # robot localization via EKF
  robot_EKF_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
      parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}])

  # Launch RViz
  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file_path],
    parameters=[{'use_sim_time': use_sim_time}])

  # nav2 launch
  start_nav2_pkg = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
      launch_arguments={
        'use_sim_time': use_sim_time,
        'params_file': nav2_config_file_path,
        'map': map_config_file_path,
        'rviz_config_file': rviz_config_file_path
      }.items())
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_world_cmd)


    # launch the nodes
  # ld.add_action(start_rviz_cmd)
  # ld.add_action(start_gazebo_cmd)
  # ld.add_action(spawn_entity_cmd)
  # ld.add_action(robot_EKF_localization_node)
  # ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_nav2_pkg)
  return ld
