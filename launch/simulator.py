import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'arcanain_simulator'
    rviz_file_name = "arcanain_simulator.rviz"

    # 引数の宣言（コマンドライン引数で指定可能にする）
    odometry_init_x_arg = DeclareLaunchArgument(
        "init_x", default_value="0.0", description="Initial x position for odometry_pub"
    )
    odometry_init_y_arg = DeclareLaunchArgument(
        "init_y", default_value="0.0", description="Initial y position for odometry_pub"
    )
    odometry_init_th_arg = DeclareLaunchArgument(
        "init_th", default_value="0.0", description="Initial theta for odometry_pub"
    )

    file_path = os.path.expanduser('~/ros2_ws/src/arcanain_simulator/urdf/mobile_robot.urdf.xml')

    with open(file_path, 'r') as file:
        robot_description = file.read()

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", rviz_file_name]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_description_rviz_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_rviz_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='both',
        parameters=[{'joint_state_publisher': robot_description}]
    )

    odometry_pub_node = Node(
        package=package_name,
        executable='odometry_pub',
        output="screen",
        parameters=[
            {"init_x": LaunchConfiguration("init_x")},
            {"init_y": LaunchConfiguration("init_y")},
            {"init_th": LaunchConfiguration("init_th")}
        ]
    )

    obstacle_pub_node = Node(
        package=package_name,
        executable='obstacle_pub',
        output="screen",
    )

    waypoint_pub_node = Node(
        package=package_name,
        executable='waypoint_pub',
        output="screen",
    )

    nodes = [
        odometry_init_x_arg,
        odometry_init_y_arg,
        odometry_init_th_arg,
        rviz_node,
        robot_description_rviz_node,
        joint_state_publisher_rviz_node,
        odometry_pub_node,
        obstacle_pub_node,
        waypoint_pub_node,
    ]

    return LaunchDescription(nodes)
