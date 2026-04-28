from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
	namespace = LaunchConfiguration('namespace')
	# Paths to installed launch files
	vicon_driver_launch = os.path.join(
		get_package_share_directory('mocap4r2_vicon_driver'),
		 'launch', 'mocap4r2_vicon_driver_launch.py')
	map_server_launch = os.path.join(
		get_package_share_directory('mocap4r2_robot_localization'),
		  'launch', 'map_server.launch.py')
	robot_localization_launch = os.path.join(
		get_package_share_directory('mocap4r2_robot_localization'),
		  'launch', 'mocap4r2_robot_localization.launch.py')
	people_launch = os.path.join(
		get_package_share_directory('mocap4r2_people'),
		  'launch', 'mocap4r2_people.launch.py')

	declare_namespace_cmd = DeclareLaunchArgument(
		'namespace',
		default_value='',
		description='Namespace for the nodes'
	)
	declare_rviz_cmd = DeclareLaunchArgument(
		'launch_rviz',
		default_value='True',
		choices=['True', 'False'],
		description='Whether to launch RViz'
	)

	declare_use_map_cmd = DeclareLaunchArgument(
		'use_map',
		default_value='False',
		choices=['True', 'False'],
		description='Whether to launch the map server'
	)
	return LaunchDescription([
		declare_namespace_cmd,
		declare_rviz_cmd,
		declare_use_map_cmd,
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(vicon_driver_launch),
			launch_arguments={'namespace': namespace, 'launch_rviz': LaunchConfiguration('launch_rviz')}.items()
		),
		GroupAction(
			condition=IfCondition(LaunchConfiguration('use_map')),
			actions=[
				IncludeLaunchDescription(
					PythonLaunchDescriptionSource(map_server_launch),
					launch_arguments={'namespace': namespace}.items()
				)
			]
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(robot_localization_launch),
			launch_arguments={'namespace': namespace}.items()
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(people_launch),
			launch_arguments={'namespace': namespace}.items()
		),
	])
