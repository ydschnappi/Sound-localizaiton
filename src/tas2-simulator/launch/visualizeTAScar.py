# ===================================================================================
# Description: Launch file to visualize TAS car in Gazebo and Rviz. 
#                [Note: Its just for visualization. You cannot sent the goal.]
# Author: Salman Bari
# Date: Oct 29, 2022
# ===================================================================================

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  subprocess.run(['killall', 'gzserver'])
  subprocess.run(['killall', 'gzclient'])

  # Constants for paths to different files and folders
  package_name = 'tas2-simulator'
  robot_name_in_model = 'tas_car'
  gazebo_models_path = 'models/gazeboModels'
  gazebo_world_file_path = 'models/worlds/basement.world'
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  urdf_file_path = os.path.join(pkg_share, 'models/urdf/tas_car.urdf')
  sdf_file_path = os.path.join(pkg_share, 'models/urdf/tas_car.sdf')
  rviz_config_file_path = os.path.join(pkg_share, 'rviz/visualizeTAScar.rviz')
  gazebo_world_path = os.path.join(pkg_share, gazebo_world_file_path)
  gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

  # Pose where we want to spawn the robot in Gazebo
  spawn_at_x = '-10.0'
  spawn_at_y = '10.0'
  spawn_at_z = '0.0'
  spawn_at_yaw = '0.00'

  # Launch configuration variables specific to simulation 
  use_sim_time = LaunchConfiguration('use_sim_time')
  world = LaunchConfiguration('world')
  urdf = open(urdf_file_path).read()
  
  
  # Declare the launch arguments 
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=rviz_config_file_path,
    description='Path to the rviz config file')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true')

  declare_gazebo_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=gazebo_world_path,
    description='Full path to the gazebo world model file to load')
  
  # Subscribe to the joint states of the robot and publish.    
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': urdf
                }
               ]
    )

  # Publish the joint states of the robot
  start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    arguments=[urdf_file_path])

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

  # Start the rviz
  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file_path],
    parameters=[{'use_sim_time': use_sim_time}])

  
  # launch descriptions
  ld = LaunchDescription()
  
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_gazebo_world_cmd)


  # launch the nodes
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_joint_state_publisher_cmd)
  ld.add_action(start_gazebo_cmd)
  ld.add_action(spawn_entity_cmd)
  ld.add_action(start_rviz_cmd)
  return ld