import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='robot' # --- CHANGE ME

    map_file         = os.path.join(get_package_share_directory(package_name), 'maps'  , 'my_map_save.yaml')
    nav2_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml')
    ekf_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')

    rsp = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	   	get_package_share_directory(package_name),'launch','rsp.launch.py'
		   )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
	   PythonLaunchDescriptionSource([os.path.join(
		get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
	   )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
 	    arguments=['-topic', 'robot_description', 
 	    	'-entity', 'prototype_robot'],
			output='screen')

    # twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    # twist_mux = Node(
    #     package="twist_mux",
    #     executable="twist_mux",
    #     parameters=[twist_mux_params, {'use_sim_time': True}],
    #     remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    # )

    nav_node = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	   	get_package_share_directory(package_name),'launch','navigation_launch.py'
		   )]),
        launch_arguments={
            'params_file' : nav2_params_file,
            'use_sim_time': 'true',
            'use_respawn' : 'true',
        }.items()   
    )

    slam_node = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	   	get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py'
		   )]),
        launch_arguments={
            'slam_params_file'  : slam_params_file,
            'use_sim_time' : 'true'
        }.items()
    )

    robot_localization_node = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join(
	   	get_package_share_directory('robot_localization'),'launch','ekf.launch.py'
		   )]),
        launch_arguments={
            'params_file'  : ekf_params_file,
            'use_sim_time' : 'true'
        }.items()
    )



    # Launch them all!
    return LaunchDescription([
        rsp,
        #twist_mux,
        gazebo,
        spawn_entity,
        robot_localization_node,
        nav_node,
        slam_node
    ])
