import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get the path to your package
    pkg_name = 'janusseraphim_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 2. Define the path to your main URDF file
    # Ensure this path is correct relative to the package root
    default_model_path = os.path.join(pkg_share, 'urdf/robots', 'janusseraphim.urdf')
    
    # Path to the serial script (Assuming it's in src/janusseraphim/janusseraphim_description/scripts/)
    # We construct this path relative to the package share if installed, or source if local.
    # For simplicity in this workspace, we point to the source script location:
    script_path = '/home/syntropy/ros2_ws/src/janusseraphim/janusseraphim_description/scripts/serial_driver.py'
    sim_script_path = '/home/syntropy/ros2_ws/src/janusseraphim/janusseraphim_description/scripts/serial_sim_node.py'
    serial_writer_path = '/home/syntropy/ros2_ws/src/janusseraphim/janusseraphim_description/scripts/joint_state_serial_writer.py'

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

    # Run a simulated serial publisher node instead of hardware
    use_sim_node_arg = DeclareLaunchArgument(
        name='use_sim_node',
        default_value='false',
        description='If true, run a simulated serial node that publishes joint_states.'
    )

    send_to_robot_arg = DeclareLaunchArgument(
        name='send_to_robot',
        default_value='false',
        description='If true, forward outgoing joint_states to the physical robot over serial.'
    )

    serial_output_port_arg = DeclareLaunchArgument(
        name='serial_output_port',
        default_value='/dev/ttyACM0',
        description='Serial device path for joint_state streaming when send_to_robot is true.'
    )

    # New argument to simulate serial input
    simulate_serial_arg = DeclareLaunchArgument(
        name='simulate_serial',
        default_value='false',
        description='If true, simulate serial data when use_hardware is true.'
    )

    # Interactive mode: disable GUI, let external IK solver publish joint_states
    interactive_mode_arg = DeclareLaunchArgument(
        name='interactive_mode',
        default_value='false',
        description='If true, disable all joint_state publishers (use with interactive_controller.launch.py).'
    )

    use_hardware = LaunchConfiguration('use_hardware')
    use_sim_node = LaunchConfiguration('use_sim_node')
    send_to_robot = LaunchConfiguration('send_to_robot')
    serial_output_port = LaunchConfiguration('serial_output_port')
    simulate_serial = LaunchConfiguration('simulate_serial')
    interactive_mode = LaunchConfiguration('interactive_mode')

    # --- Robot State Publisher Node ---
    robot_description = Command(['xacro ', LaunchConfiguration('model')])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # --- Joint State Publisher GUI Node (Simulation) ---
    # Only runs if neither hardware nor simulated serial node is used AND not interactive mode
    gui_condition = PythonExpression([
        '"', use_hardware, '" != "true" and "', use_sim_node, '" != "true" and "', interactive_mode, '" != "true"'
    ])

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(gui_condition)
    )

    # --- Serial Driver Node (Hardware) ---
    # Only runs if use_hardware is true and sim node is false
    # We execute the python script directly
    hardware_condition = PythonExpression([
        '"', use_hardware, '" == "true" and "', use_sim_node, '" != "true"'
    ])

    serial_driver_process = ExecuteProcess(
        cmd=[sys.executable, script_path, '--use-dummy', simulate_serial],
        output='screen',
        condition=IfCondition(hardware_condition)
    )

    # --- Simulated Serial Node (Publishes joint_states) ---
    sim_serial_process = ExecuteProcess(
        cmd=[sys.executable, sim_script_path],
        output='screen',
        condition=IfCondition(use_sim_node)
    )

    # --- Joint State Serial Writer (GUI/Sim -> hardware) ---
    serial_writer_process = ExecuteProcess(
        cmd=[
            sys.executable,
            serial_writer_path,
            '--port', serial_output_port,
        ],
        output='screen',
        condition=IfCondition(send_to_robot)
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
        use_sim_node_arg,
        send_to_robot_arg,
        serial_output_port_arg,
        simulate_serial_arg,
        interactive_mode_arg,
        joint_state_publisher_gui_node,
        serial_driver_process,
        sim_serial_process,
        serial_writer_process,
        robot_state_publisher_node,
        rviz_node
    ])