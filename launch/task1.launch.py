from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    return LaunchDescription([ 
        Node( 
            package='com2009_team69_2025', 
            executable='move_eight_shape.py', 
            name='velocity_control_node'
        )
    ])