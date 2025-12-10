from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for hardware output',
    )
    send_serial_arg = DeclareLaunchArgument(
        'send_serial',
        default_value='true',
        description='Send solved joint angles to serial (hardware)',
    )

    serial_port = LaunchConfiguration('serial_port')
    send_serial = LaunchConfiguration('send_serial')

    return LaunchDescription([
        serial_port_arg,
        send_serial_arg,
        # Interactive marker (broadcasts ee_link_interactive TF, no serial output now)
        Node(
            package='janusseraphim_description',
            executable='interactive_controller.py',
            name='interactive_controller',
            output='screen',
            parameters=[
                {'target_link': 'ee_link'},
                {'reference_frame': 'base_link'},
                {'serial_port': ''},  # disable serial here; IK solver handles it
                {'serial_baud': 115200},
            ],
        ),
        # IK solver: reads ee_link_interactive TF, publishes joint_states
        Node(
            package='janusseraphim_description',
            executable='ik_solver_node.py',
            name='ik_solver_node',
            output='screen',
            parameters=[
                {'target_frame': 'ee_link_interactive'},
                {'reference_frame': 'base_link'},
                {'serial_port': serial_port},
                {'serial_baud': 115200},
                {'send_serial': send_serial},
            ],
        ),
    ])
