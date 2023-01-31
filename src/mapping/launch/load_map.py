import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

	map_file = os.path.join(get_package_share_directory('mapping'), 'config', 'map.уаml')

	return LaunchDescription([
		Node( 
		package='nav2_map_server',
		executable='map_server',
		name='map_server', 
		output='screen',
		parameters=[{'use_sim_time': True},
			    {'yaml_filename': map_file}
			    ]),

		Node( 
		    package='rviz2',
		    executable='rviz2', 
		    name='rviz2',
		    parameters=[{'use_sim_time': True}], 
		    output='screen'),


		Node( 
		    package='nav2_lifecycle_manager',
		    executable='lifecycle_manager',
		    name='lifecycle_manager_mapper',
		    output='screen', 
		    parameters=[{'use_sim_time': True},
				{'autostart': True},
				{'node_names': ['map_server']}])


	            ])
