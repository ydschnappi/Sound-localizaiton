from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        Node(
            package='signal_generator',
            executable='signal_pose_node',
            #parameters=[
            #    {{'signal_x':15.0,'signal_y':17.0,'signal_z':0.0}}
            #]
        ),
        Node(
            package='signal_generator',
            executable='signal_publisher_node',
            output = 'screen',
            # parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='planner',
            executable='planner',
            output = 'screen',
        ),
        Node(
            package='planner',
            executable='visualization',
            output = 'screen',
        ),
        ])
