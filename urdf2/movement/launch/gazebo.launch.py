import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

import xacro


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    pkg_name = 'final_description'
    file_subpath = 'urdf/final.urdf.xacro'
    world_subpath = 'worlds/test_world.world'
    config_subpath = 'config/ros2_controllers.yaml'



    movement_demo_path = os.path.join(get_package_share_directory('movement'))
    description_path = os.path.join(get_package_share_directory(pkg_name))
    world_path = os.path.join(description_path, world_subpath)



    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'ign_args': '-r ' + world_path}.items(),

    )
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )



    rviz_config_file = os.path.join(description_path, 'config', 'display.rviz')
    control_config_file = os.path.join(movement_demo_path,config_subpath)
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[control_config_file],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )


    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )


    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'  # Add this line for debug output
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_raw,
                   '-name', 'inchworm',
                   '-allow_renaming', 'true'],
    )

    # Gz - ROS Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )


    load_joint_state_broadcaster =  ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    # trajectory controller
    load_top_trajectory_controller =  ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'top_controller'],
        output='screen'
    )

    load_mid_trajectory_controller =  ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mid_controller'],
        output='screen'
    )



    load_bot_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'bot_controller'],
        output='screen'
    )

    imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen'
    )
    #point cloud bridge





    return LaunchDescription([
        gz_sim,
        gui_arg,

        bridge,
        joint_state_publisher_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_top_trajectory_controller,load_mid_trajectory_controller,load_bot_trajectory_controller],
            )
        ),       

      

        node_robot_state_publisher,
        spawn,
        rviz_node,
    ])
