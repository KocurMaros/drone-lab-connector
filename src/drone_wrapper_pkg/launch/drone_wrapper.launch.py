from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument('fcu_url', default_value='udp://127.0.0.1:14550@14555')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='/drones/edu11')
    tgt_system_arg = DeclareLaunchArgument('tgt_system', default_value='11')

    topics_yaml = os.path.join(
        '/home/deck/Projects/drone-lab-connector/install/drone_wrapper_pkg/share/drone_wrapper_pkg/config',
        'domain_bridge_topics.yaml'
    )
    topics_0_11_yaml = os.path.join(
        '/home/deck/Projects/drone-lab-connector/install/drone_wrapper_pkg/share/drone_wrapper_pkg/config',
        'domain_bridge_topics_0_to_11.yaml'
    )

    # Copy OS environment for correct setup.py metadata paths
    student_env = os.environ.copy()
    student_env['ROS_DOMAIN_ID'] = '0'

    drone_env = os.environ.copy()
    drone_env['ROS_DOMAIN_ID'] = '11'

    return LaunchDescription([
        fcu_url_arg,
        namespace_arg,
        tgt_system_arg,

        # 1. MAVROS Native Node - ON DRONE DOMAIN 11
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'tgt_system': LaunchConfiguration('tgt_system')
            }],
            # Restore the safety remaps originally present in C++ GUI
            remappings=[
                ('mavros/setpoint_position/local', 'mavros/internal_setpoint_position/local'),
                ('mavros/setpoint_velocity/cmd_vel_unstamped', 'mavros/internal_setpoint_velocity/cmd_vel_unstamped'),
                ('mavros/setpoint_raw/local', 'mavros/_internal/setpoint_raw/local'),
                ('mavros/setpoint_raw/global', 'mavros/_internal/setpoint_raw/global'),
                ('mavros/setpoint_raw/attitude', 'mavros/_internal/setpoint_raw/attitude'),
                ('mavros/setpoint_attitude/attitude', 'mavros/_internal/setpoint_attitude/attitude'),
                ('mavros/setpoint_attitude/cmd_vel', 'mavros/_internal/setpoint_attitude/cmd_vel')
            ],
            env=drone_env,
            output='screen'
        ),

        # Bridge 1: Topics 11 -> 0 ONLY.
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='bridge_topics_11_0',
            arguments=[topics_yaml],
            output='screen'
        ),
        
        # Bridge 2: Topics 0 -> 11 (Commands)
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='bridge_topics_0_11',
            arguments=[topics_0_11_yaml],
            output='screen'
        ),
        
        # Start Validation Node - ON STUDENT DOMAIN 0 
        Node(
            package='drone_wrapper_pkg',
            executable='validation_node',
            output='screen',
            env=student_env
        ),
        
        # Start Arming Sequence Node - (Multidomain Python Script)
        # Avoid parameter-based renaming which breaks multi-node scripts!
        Node(
            package='drone_wrapper_pkg',
            executable='arming_node',
            output='screen',
            parameters=[
                {'mavros_namespace': LaunchConfiguration('namespace')}
            ],
            env=student_env
        )
    ])
