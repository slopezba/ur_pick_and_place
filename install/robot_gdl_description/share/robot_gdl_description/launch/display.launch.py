from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type", default="ur5e")

    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"),
                "launch",
                "ur_sim_moveit.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "description_package": "robot_gdl_description",
            "description_file": "robot_gdl.urdf.xacro",
        }.items(),
    )

    return LaunchDescription([ur_launch])
