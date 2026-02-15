from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('inverted_pendulum_control')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    world = PathJoinSubstitution([package_share, 'worlds', 'mybot.world'])
    lip_model = PathJoinSubstitution([package_share, 'model', 'lip.urdf'])
    mybot_model = PathJoinSubstitution([package_share, 'model', 'mybot.xacro'])
    pid_params = PathJoinSubstitution([package_share, 'config', 'lip_pid.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_pid = LaunchConfiguration('start_pid')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true',
        }.items(),
    )

    lip_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='lip',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': Command([FindExecutable(name='xacro'), ' ', lip_model]),
            }
        ],
    )

    mybot_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='mybot',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': Command([FindExecutable(name='xacro'), ' ', mybot_model]),
            }
        ],
    )

    spawn_lip = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'lip',
            '-topic', '/lip/robot_description',
            '-robot_namespace', '/lip',
            '-x', '10', '-y', '0', '-z', '0',
        ],
    )

    spawn_mybot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'mybot', '-topic', '/mybot/robot_description', '-robot_namespace', '/mybot'],
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/lip/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
    )

    load_joint_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_effort_controller_j_0',
            '--controller-manager',
            '/lip/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
    )

    lip_control = Node(
        package='inverted_pendulum_control',
        executable='control_pid.py',
        output='screen',
        parameters=[pid_params],
        condition=IfCondition(start_pid),
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_lip,
            on_exit=[
                TimerAction(
                    period=4.0,
                    actions=[
                        load_joint_state_broadcaster,
                        load_joint_effort_controller,
                        lip_control,
                    ],
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('start_pid', default_value='false'),
            gazebo,
            lip_rsp,
            mybot_rsp,
            spawn_lip,
            spawn_mybot,
            start_controllers,
        ]
    )
