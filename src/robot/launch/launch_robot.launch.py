import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():


    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    package_name='robot' # --- CHANGE ME

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    rsp = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	   	get_package_share_directory(package_name),'launch','rsp.launch.py'
		   )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    imu = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	   	get_package_share_directory(package_name),'launch','ros2_mpu6050.launch.py'
		   )])
    )

    # pkg_path = os.path.join(get_package_share_directory(package_name))
    # xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    # robot_description = Command(['xacro ', xacro_file])

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_params_file],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    nav2_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
    nav_node = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	   	    get_package_share_directory(package_name),'launch','navigation_launch.py'
        )]),
        launch_arguments={
            'params_file' : nav2_params_file,
            'use_sim_time': 'false',
            'use_respawn' : 'true',
        }.items()   
    )

    slam_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml')
    slam_node = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	   	    get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py'
        )]),
        launch_arguments={
            'slam_params_file'  : slam_params_file,
            'use_sim_time' : 'false'
        }.items()
    )

    amcl_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'amcl_params.yaml')
    amcl_node = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	   	    get_package_share_directory(package_name),'launch','localization_launch.py'
        )]),
        launch_arguments={
            'map'  : '/home/mosalam/robot_ws/src/robot/maps/robotics_room.yaml',
            'use_sim_time' : 'false',
            'params_file': amcl_params_file
        }.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    rplidar = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	       	get_package_share_directory(package_name),'launch','rplidar.launch.py'
        )])
    )

    lidar_filter = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	   	    get_package_share_directory(package_name),'launch','filter_lidar.launch.py'
		)])
    )

    smart_navigation_node = LaunchDescription([
        Node(
            package=package_name,
            executable='smart_navigation.py',
            name='smart_navigation'
        )
    ])

    # initial_pose_pub = Node(
    #     package='geometry_msgs',
    #     executable='initial_pose_publisher',  # this is your own node or a small script you make
    #     name='initial_pose_publisher',
    #     output='screen',
    #     parameters=[{
    #         'x': 1.0,
    #         'y': 2.0,
    #         'theta': 1.57
    #     }]
    # )

    # delayed_initial_pose = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=amcl_node,
    #         on_start=[initial_pose_pub],
    #     )
    # )

    # # Delay it a few seconds after AMCL starts
    # delayed_pose = TimerAction(
    #     period=3.0,
    #     actions=[initial_pose_publisher]
    # )

    vizanti_node = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	   	    get_package_share_directory('vizanti_server'),'launch','vizanti_server.launch.py'
		)])
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        # imu,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        rplidar,
        lidar_filter,
        # slam_node,
        amcl_node,
        # delayed_initial_pose,
        nav_node,
        # vizanti_node,
        # smart_navigation_node
    ])
