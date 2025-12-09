import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get the path to your package
    pkg_name = 'grafika_ros_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 2. Define the path to your main URDF file
    # Ensure this path is correct relative to the package root
    default_model_path = os.path.join(pkg_share, 'urdf/robots', 'grafika_ros.urdf')
    
    # Path to the serial script (Assuming it's in src/grafika_ros/grafika_ros_description/scripts/)
    # We construct this path relative to the package share if installed, or source if local.
    # For simplicity in this workspace, we point to the source script location:
    script_path = '/home/syntropy/ros2_ws/src/grafika_ros/grafika_ros_description/scripts/serial_driver.py'

    # --- Launch Arguments ---
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Absolute path to robot URDF file'
    )
    
    # New argument to toggle between GUI and Arduino
    use_hardware_arg = DeclareLaunchArgument(
        name='use_hardware',
        default_value='false',
        description='If true, read from Arduino serial. If false, use GUI sliders.'
    )

    # New argument to simulate serial input
    simulate_serial_arg = DeclareLaunchArgument(
        name='simulate_serial',
        default_value='false',
        description='If true, simulate serial data when use_hardware is true.'
    )

    use_hardware = LaunchConfiguration('use_hardware')
    simulate_serial = LaunchConfiguration('simulate_serial')

    # --- Robot State Publisher Node ---
    robot_description = Command(['xacro ', LaunchConfiguration('model')])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # --- Joint State Publisher GUI Node (Simulation) ---
    # Only runs if use_hardware is false
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=UnlessCondition(use_hardware)
    )

    # --- Serial Driver Node (Hardware) ---
    # Only runs if use_hardware is true
    # We execute the python script directly
    serial_driver_process = ExecuteProcess(
        cmd=[sys.executable, script_path, '--use-dummy', simulate_serial],
        output='screen',
        condition=IfCondition(use_hardware)
    )

    # --- RViz Node ---
    rviz_config_dir = os.path.join(pkg_share, 'rviz', 'config.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
    )

    return LaunchDescription([
        model_arg,
        use_hardware_arg,
        simulate_serial_arg,
        joint_state_publisher_gui_node,
        serial_driver_process,
        robot_state_publisher_node,
        rviz_node
    ])