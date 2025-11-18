# Import the core class that represents a full launch description (the list of actions)
from launch import LaunchDescription

# IncludeLaunchDescription allows us to nest other launch files (e.g., bring up MoveIt)
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# This tells ROS 2 how to load another launch file written in Python
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Node is used to start a ROS 2 executable as part of the launch
from launch_ros.actions import Node

# get_package_share_directory finds the installed share/ path of a package
# (so we can locate its launch files, config files, meshes, etc.)
from ament_index_python.packages import get_package_share_directory

import os  # for handling file paths

# Every ROS 2 launch file must define this function; the launch system calls it automatically
def generate_launch_description():

    # Path to robot_gdl_description share directory
    robot_desc_share = get_package_share_directory('robot_gdl_description')

    # Path to your URDF/Xacro
    urdf_file = os.path.join(robot_desc_share, 'urdf', 'robot_gdl.urdf.xacro')


    # Example: publish robot_description using that URDF/Xacro
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # If it is xacro:
            'robot_description': Command(['xacro ', urdf_file])
            # If it is a plain URDF, use open(urdf_file).read() instead of Command(...)
        }]
    )


    # Get the absolute path to the ur_robot_driver package's share directory
    pkg_path = get_package_share_directory('ur_robot_driver')

    custom_description = os.path.join(
        robot_desc_share, 'urdf', 'robot_gdl.urdf.xacro'  # adjust filename
    )
    
    ur_robot_driver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, 'launch', 'ur_control.launch.py')
            ),
            launch_arguments={
                'ur_type' : 'ur5e',
                'robot_ip' : '172.17.0.2',            # Modify if different robot ip
                'launch_rviz' : 'false',
                'description_package': 'robot_gdl_description',
                'description_file': 'robot_gdl.urdf.xacro',
            }.items()
        )


    # Get the absolute path to the ur_moveit_config package's share directory
    pkg_path = get_package_share_directory('ur_moveit_config')

    # Create an IncludeLaunchDescription action that will launch MoveIt for the UR5e robot
    moveit_launch = IncludeLaunchDescription(
        # Specify the file to include: ur_moveit.launch.py located inside the package's launch folder
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'ur_moveit.launch.py')
        ),
        # Pass arguments to that launch file (like command-line args)
        launch_arguments={
            'ur_type': 'ur5e',         # tells MoveIt which robot model to load
            'launch_rviz': 'true'      # start RViz visualization automatically
        }.items()
    )

    # Create box obstacle inside MoveIt to plan for collisions with the UR5e base
    add_box = TimerAction(                 # small delay so move_group is ready
        period=2.0,
        actions=[Node(
            package='ur_pick_and_place',
            executable='add_box_obstacle',
            output='screen'
        )]
    )

    # Define your own node (in this case, your go_home_node from ur_pick_and_place)
    go_home = Node(
        package='ur_pick_and_place',  # name of your ROS 2 package
        executable='go_home_node',    # the executable to run (must match the target name in CMakeLists.txt)
        name='go_home_node',          # node name on the ROS graph
        output='screen'               # print log output to the terminal instead of a log file
    )

    pick_and_place = Node(
        package='ur_pick_and_place',
        executable='pick_and_place',
        name='pick_and_place',
        output='screen'
    )

    # --- Trigger "Play" on the UR dashboard client ---
    play_robot = ExecuteProcess(
        cmd=['ros2', 'service', 'call',
             '/dashboard_client/play',
             'std_srvs/srv/Trigger', '{}'],
        output='screen'
    )

    # Return a LaunchDescription that tells ROS 2 to:
    # 1) launch MoveIt (the include)
    # 2) then launch your custom node
    return LaunchDescription([
        robot_state_publisher,
        ur_robot_driver_launch,
        moveit_launch,
        add_box,
        play_robot,
        go_home,
        pick_and_place
        ])
