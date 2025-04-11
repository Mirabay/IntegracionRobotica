import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    urdf_file_name = 'continuos_ex.urdf'
    urdf = os.path.join(
        get_package_share_directory('joints_act'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
                            arguments=[urdf]
                            )
    
    joint_state_publisher_node = Node(
                            package='joint_state_publisher_gui',
                            executable='joint_state_publisher_gui',
                            name='joint_state_publisher',
                            output='screen'
                            )

    # Rvis2
    rviz2_node = Node(
                            package='rviz2',
                            executable='rviz2',
                            name='rviz2',
                            output='screen',
                            arguments=['-d', os.path.join(get_package_share_directory('joints_act'), 'rviz', 'joint_state_publisher.rviz')]
                            )
    
    l_d = LaunchDescription([robot_state_pub_node,
                             joint_state_publisher_node,
                             rviz2_node])

    return l_d