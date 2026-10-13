from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory("Ust_description")

    xacro_file = os.path.join(share_dir, 'urdf', 'Ust.urdf')
    robot_doc = xacro.parse(open(xacro_file))
    xacro.process_doc(robot_doc)
    robot_description = robot_doc.toxml()
    world_file_path = os.path.join(share_dir, "worlds", "empty.sdf")

    # launch args
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="whether enable sim time or not"
    )
    pause_sim_arg = DeclareLaunchArgument(
        "pause_sim",
        default_value="true",
        description="whether to pause or unpause sim on launch"
    )
    sim_pause_flag = PythonExpression(["'' if '", LaunchConfiguration("pause_sim"), "' == 'true' else '-r'"])

    # launcher server-client separated for ign gz sim
    ign_gz_sim_server_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments = {
            'gz_args': [sim_pause_flag, ' -s -v1 ', world_file_path]
        }.items()
    )
    ign_gz_sim_client_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments = {
            "gz_args": "-g -v1 "
        }.items()
    )
    # publishes TF from joint states
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen"
    )

    # activates joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    # activates your effort controller
    mcp_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mcp_effort_controller"],
        output="screen"
    )

    pip_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pip_effort_controller"],
        output="screen"
    )

    dip_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dip_effort_controller"],
        output="screen"
    )

    # spawn model in gz sim
    spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            '-name', "Ust",
            '-string', robot_description,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        name="gz_spawner_node",
        output="screen"
    )

    #spawn model in rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share_dir, 'rviz', 'ust_config.rviz')],  # optional saved config
        output='screen'
    )
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(pause_sim_arg)
    ld.add_action(ign_gz_sim_server_launcher)
    ld.add_action(ign_gz_sim_client_launcher)
    ld.add_action(spawner_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(mcp_effort_controller_spawner)
    ld.add_action(pip_effort_controller_spawner)
    ld.add_action(dip_effort_controller_spawner)
    return ld