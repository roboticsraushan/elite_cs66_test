#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the launch directory
    pkg_my_robot = get_package_share_directory('my_elite_robot_cell_description')
    pkg_cs_sim = get_package_share_directory('eli_cs_robot_simulation_gz')
    pkg_my_robot_control = get_package_share_directory('my_elite_robot_cell_control')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=os.path.join(pkg_cs_sim, 'worlds', 'empty.world'))
    robot_name = LaunchConfiguration('robot_name', default='cs')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Controller configurations
    mobile_base_controller_config = os.path.join(pkg_my_robot_control, 'config', 'mobile_base_controllers.yaml')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_cs_sim, 'worlds', 'empty.world'),
        description='Full path to world file to load')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='cs',
        description='Name of the robot')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz')

    # Get URDF via xacro
    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'mobile_cleaning_robot.urdf.xacro')
    robot_description_content = Command(
        [
            'xacro ',
            urdf_file,
            ' name:=',
            robot_name,
            ' sim_ignition:=true',
            ' use_fake_hardware:=true',
            ' fake_sensor_commands:=true'
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Start Gazebo with our world
    ignition_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen')

    # Spawn the robot in Gazebo
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',  # Increased height to avoid ground penetration
            '-Y', '0.0'   # Yaw orientation
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]  # Ensure sim time is used
    )

    # Robot state publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Joint state publisher
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz
    rviz_config_file = os.path.join(pkg_my_robot, 'rviz', 'view_robot.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # Include CS robot controllers
    cs_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cs_sim, 'launch', 'cs_sim_control.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'load_controllers_only': 'true',
            'use_rviz': 'false'  # Disable RViz in CS launch
        }.items()
    )

    # Load and start mobile base controllers
    controller_manager_cmd = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            mobile_base_controller_config,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Spawn mobile base controllers
    spawn_jsb_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    spawn_mobile_base_controller_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mobile_base_controller'],
        output='screen'
    )

    # Bridge
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            # Updated topics for Ignition visualization
            '/world/empty/model/cs/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose',
            '/world/empty/model/cs/pose_static@geometry_msgs/msg/Pose[ignition.msgs.Pose',
            '/world/empty/model/cs/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Additional topics for mobile base
            '/mobile_base_controller/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/mobile_base_controller/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry'
        ],
        remappings=[
            ('/world/empty/model/cs/joint_states', '/joint_states'),
            ('/mobile_base_controller/cmd_vel', '/cmd_vel'),
            ('/mobile_base_controller/odom', '/odom')
        ],
        output='screen'
    )

    # Create and return launch description
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add nodes to launch description
    ld.add_action(ignition_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    # Add a delay before spawning the robot
    ld.add_action(ExecuteProcess(
        cmd=['sleep', '2.0'],
        output='screen'
    ))
    ld.add_action(spawn_robot_cmd)
    ld.add_action(cs_controllers_launch)
    ld.add_action(controller_manager_cmd)
    ld.add_action(spawn_jsb_cmd)
    ld.add_action(spawn_mobile_base_controller_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(rviz_cmd)  # Add RViz last

    return ld 