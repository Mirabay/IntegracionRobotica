from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # lauunch Rviz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        
    )
    # Launch the static transform publisher
    static_tf_node_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        output='screen',
        arguments=[
            '0', '0', '0',
            '0', '0', '0', 
            'world', 'robot_1'
            ],
    )
    static_tf_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        output='screen',
        arguments=[
            '0', '-1.0', '0',
            '0', '0', '0', 
            'robot_1', 'robot_2'
            ],
    )

    
    l_d = LaunchDescription([rviz2_node, static_tf_node_1, static_tf_node_2])
    return l_d