import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch_ros.actions


from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable,DeclareLaunchArgument



def generate_launch_description():

    namespace = LaunchConfiguration("namespace", default="")

    namespace_launch_arg = DeclareLaunchArgument(
		'namespace', default_value=''
	)

    config_file = LaunchConfiguration('params_file')

    config_file_launch = DeclareLaunchArgument(
		'params_file', default_value='params.yaml'
	)

    config_file_path = PathJoinSubstitution(
                [
                    FindPackageShare('uwb_serial'),
                    'config',
                    config_file
                ]
                )

    
    return LaunchDescription([
        namespace_launch_arg,
        config_file_launch,
        launch_ros.actions.Node(
            package='uwb_serial',
            executable='uwb',
            namespace=namespace,
            output='screen',
            parameters=[config_file_path],
            ros_arguments = ['--params-file', config_file_path]
        ),
    ])
