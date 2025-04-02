import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package directory
    urdf_file_name = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
        print(robot_description)
    # Create the robot description parameter
    
    robot_state_pub_node= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }],
        arguments=[urdf],
        output='screen'
    )
    joint_state_pub_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    joint_state_publisher_node = Node(
        package='puzzlebot_sim',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )   
    
    rviz2_pub_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('puzzlebot_sim'), 'rviz', 'puzzlebot.rviz')],
        output='screen',
    )
    

    l_d = LaunchDescription([rviz2_pub_node,
                             robot_state_pub_node,
                            #  joint_state_pub_node,
                             joint_state_publisher_node])

    return l_d