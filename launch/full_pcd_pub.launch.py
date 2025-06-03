from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace of the robot'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config.yaml',
        description='Name of the config file to be used (located in the Params folder)'
    )

    # Get the path to the Params folder of the package
    package_name = 'isaac_sim_pointcloud_full_publisher'  # Replace with your package name
    params_folder = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'Params'
    ])

    # Full path to the configuration file
    config_file_path = PathJoinSubstitution([
        params_folder,
        LaunchConfiguration('config_file')
    ])

    # Node configuration
    lidar_ring_converter_node = Node(
        package=package_name,
        executable='full_pcd_pub',
        parameters=[
            config_file_path,  # Use the resolved config file path
            {'robot_namespace': LaunchConfiguration('robot_namespace')}  # Override robot_namespace
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_namespace_arg,
        config_file_arg,
        lidar_ring_converter_node
    ])
