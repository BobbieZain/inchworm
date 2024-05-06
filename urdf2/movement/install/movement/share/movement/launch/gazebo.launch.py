import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition,  UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


import xacro


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    pkg_name = 'final_description'
    file_subpath = 'urdf/final.urdf.xacro'
    world_subpath = 'worlds/test_world.world'
    world_path = os.path.join(get_package_share_directory(pkg_name), world_subpath)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world_path}.items(),
        # launch_arguments={
        #    'gz_args': '-r gpu_lidar.sdf'
        # }.items(),
    )
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )
    
    share_dir = get_package_share_directory('final_description')
    show_gui = LaunchConfiguration('gui')
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )


    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        # parameters=[{'name': 'example_robot',
        #             '-topic': '/robot_description'}],
        arguments=['-topic', 'robot_description',
        '-entity', 'my_bot'], output='screen',
    )

    # Gz - ROS Bridge
    bridge1 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (IGN -> ROS2)
            #'/world/empty/model/rrbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        # remappings=[
        #     ('/world/empty/model/rrbot/joint_state', 'joint_states'),
        # ],
        output='screen'
    )

    # Bridge
    
    return LaunchDescription([
        gz_sim,
        gui_arg,
        node_robot_state_publisher,
        spawn,
        bridge1,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
