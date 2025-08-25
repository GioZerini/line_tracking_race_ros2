from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,   # <-- AGGIUNTO
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command
)

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


pkg_project_bringup = get_package_share_directory("line_tracking_race_bringup")
pkg_project_gazebo = get_package_share_directory("line_tracking_race_gazebo")
pkg_project_description = get_package_share_directory("line_tracking_race_description")

pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

def generate_ros_gz_bridge_description(context, *args, **kwargs):

    # Get launch configuration values
    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context)  # actual value

    ros_gz_bridge_conf_file = f"ros_gz_bridge_{'ros2' if use_ros2_control.lower() == 'true' else 'gazebo'}_control.yaml"

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_project_bringup, "config", ros_gz_bridge_conf_file]),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return [gz_bridge]

def generate_ros2_control_description(context, *args, **kwargs):

    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context) # actual value
    car_ros2_control_file = LaunchConfiguration("car_ros2_control_file")

    ld = LaunchDescription()

    if use_ros2_control.lower() == "true":
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        )
        ld.add_action(joint_state_broadcaster_spawner)

        car_controllers_conf_file = PathJoinSubstitution([pkg_project_bringup,
                                                          "config",
                                                          car_ros2_control_file])

        diff_drive_base_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_base_controller',
                '--param-file',
                car_controllers_conf_file,
                '--controller-ros-args',
                '-r /diff_drive_base_controller/cmd_vel:=/cmd_vel',
                '--controller-ros-args',
                '-r /diff_drive_base_controller/odom:=/odom',
                ],
        )
        ld.add_action(RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster_spawner,
                                                        on_exit=[diff_drive_base_controller_spawner])))

    return [ld]

def generate_launch_description():

    # ---------------- Argomenti esistenti ----------------
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='race_track.world',
        description='The world file'
    )

    car_file_arg = DeclareLaunchArgument(
        'car_file',
        default_value='car.urdf.xacro',
        description='The car model file (xacro/urdf/sdf)'
    )

    car_ros2_control_file_arg = DeclareLaunchArgument(
        'car_ros2_control_file',
        default_value='car_control_diff_drive.yaml',
        description='The car ros2_control configuration yaml file (yaml)'
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Use ros2_control instead of gazebo control plugin'
    )

    x_pos_arg = DeclareLaunchArgument(
        "x_pos", default_value="0.0", description="X position to spawn the robot"
    )
    y_pos_arg = DeclareLaunchArgument(
        "y_pos", default_value="0.0", description="Y position to spawn the robot"
    )
    z_pos_arg = DeclareLaunchArgument(
        "z_pos", default_value="0.45", description="Z position to spawn the robot"
    )
    yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="3.142", description="Yaw orientation to spawn the robot"
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false', description='Open RViz.')

    # ---------------- NUOVI argomenti (planner/controller) ----------------
    strategy_arg = DeclareLaunchArgument('strategy', default_value='nonlinear',
                                         description="Planner strategy: centroid|centerline|nonlinear")
    error_type_arg = DeclareLaunchArgument('error_type', default_value='offset',
                                           description="Legacy error type for compatibility: offset|angle")
    viz_arg = DeclareLaunchArgument('viz', default_value='false', description='Enable visual debug overlays')

    controller_type_arg = DeclareLaunchArgument('controller_type', default_value='nlpf',
                                                description='Controller: pid|nlpf')

    k_psi_arg = DeclareLaunchArgument('k_psi', default_value='2.0',
                                      description='Gain on psi for non-linear controller')

    use_planner_speed_arg = DeclareLaunchArgument('use_planner_speed', default_value='false',
                                                  description='Use /planner/speed_cmd if available')

    px_per_m_x_arg = DeclareLaunchArgument('px_per_m_x', default_value='800.0',
                                           description='Pixels per meter (horizontal) in ROI')
    px_per_m_y_arg = DeclareLaunchArgument('px_per_m_y', default_value='800.0',
                                           description='Pixels per meter (vertical) in ROI')

    # ---------------- Gazebo ----------------
    world_file = PathJoinSubstitution([pkg_project_gazebo, 'worlds', LaunchConfiguration("world_file")])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [world_file, " -r -v 1"],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # Bridge separato (config dipendente dal tipo di controllo)
    gz_bridge = OpaqueFunction(function=generate_ros_gz_bridge_description)

    # ---------------- Robot description ----------------
    urdf_path = PathJoinSubstitution([PathJoinSubstitution([pkg_project_description, "models"]),
                                     "urdf",
                                     LaunchConfiguration("car_file")])

    robot_description_content = Command(['xacro ', urdf_path,
                                         ' use_ros2_control:=', LaunchConfiguration("use_ros2_control")])

    robot_state_publisher = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': ParameterValue(robot_description_content, value_type=str),
                                      }])

    # Spawn robot
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "car",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x_pos"),
            "-y", LaunchConfiguration("y_pos"),
            "-z", LaunchConfiguration("z_pos"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    # ros2_control dopo lo spawn
    ros2_control_description = OpaqueFunction(function=generate_ros2_control_description)
    ros2_control = RegisterEventHandler(OnProcessExit(target_action=spawn_robot_node,
                                                      on_exit=[ros2_control_description]))

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', PathJoinSubstitution([pkg_project_bringup, "config", 'line_tracking_race.rviz'])],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # ---------------- (Opzionale) Risorse Gazebo dal pacchetto description ----------------
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_project_description  # include .../share/line_tracking_race_description
    )

    # ---------------- NODI: Planner + Controller ----------------
    planner = Node(
        package='line_tracking_race_controller',
        executable='planner_node_sel',
        name='planner',
        output='screen',
        parameters=[{
            #'strategy': LaunchConfiguration('strategy'),
            'error_type': LaunchConfiguration('error_type'),
            'viz': LaunchConfiguration('viz'),
            # parametri usati dalla NonLinearStrategy
            #'px_per_m_x': LaunchConfiguration('px_per_m_x'),
            #'px_per_m_y': LaunchConfiguration('px_per_m_y'),
        }],
        remappings=[
            # ('/camera/image_raw', '/my_camera_topic'),  # se usi un topic camera diverso, rimappa qui
        ]
    )

    controller = Node(
        package='line_tracking_race_controller',
        executable='control_node_sel',
        name='control_node',
        output='screen',
        parameters=[{
            #'controller_type': LaunchConfiguration('controller_type'),
            #'k_psi': LaunchConfiguration('k_psi'),
            #'use_planner_speed': LaunchConfiguration('use_planner_speed'),
            'duration': -1,
            'eps_div': 1.0e-3,
            # PID per test A/B:
            'k_p': 1.0, 'k_i': 0.2, 'k_d': 0.2,
        }]
    )
    
    viz = Node(
    	package="line_tracking_race_controller",
    	executable="visualizer",
    	name="viz",
    	output="screen"
    )

    return LaunchDescription(
        [
            # Argomenti esistenti
            world_file_arg,
            car_file_arg,
            x_pos_arg,
            y_pos_arg,
            z_pos_arg,
            yaw_arg,
            use_ros2_control_arg,
            car_ros2_control_file_arg,
            rviz_arg,

            # Nuovi argomenti planner/controller
            strategy_arg,
            error_type_arg,
            viz_arg,
            controller_type_arg,
            k_psi_arg,
            use_planner_speed_arg,
            px_per_m_x_arg,
            px_per_m_y_arg,

            # Gazebo + bridge + robot
            gz_sim,
            gz_bridge,
            set_gz_resource_path,
            robot_state_publisher,
            spawn_robot_node,
            ros2_control,
            rviz,
            viz,

            # Nuovi nodi
            planner,
            controller,
        ]
    )
