from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gate_detection',
            
            node_executable='client',
            node_name='client_node'
        ),
        Node(
            package='gate_detection',
           
            node_executable='service',
            node_name='server_node'
        )
    ])