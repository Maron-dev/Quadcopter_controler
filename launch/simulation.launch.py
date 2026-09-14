from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('quadcopter_sim'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    use_rviz = LaunchConfiguration('rviz')
    scenario = LaunchConfiguration('scenario')
    world = LaunchConfiguration('world')
    mission = LaunchConfiguration('mission')
    controller = LaunchConfiguration('controller')
    evaluator = LaunchConfiguration('evaluator')
    report_file = LaunchConfiguration('report_file')
    position_tolerance = LaunchConfiguration('position_tolerance')

    # A scenario selects a matching world and trajectory. The explicit `world`
    # and `mission` arguments remain available for custom combinations.
    default_world = PythonExpression([
        "'", str(share / 'worlds'), "/' + ('obstacle_course.world' if '",
        scenario, "' == 'obstacle_course' else 'empty.world')"
    ])
    default_mission = PythonExpression([
        "'", str(share / 'config'), "/' + ('mission_obstacle_course.yaml' if '",
        scenario, "' == 'obstacle_course' else 'mission.yaml')"
    ])
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='default',
                              choices=['default', 'obstacle_course'],
                              description='World and trajectory set to run'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='World file (overrides the selected scenario)'),
        DeclareLaunchArgument('mission', default_value=default_mission,
                              description='Mission file (overrides the selected scenario)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('controller', default_value='pid',
                              choices=['pid', 'lqr', 'mpc'],
                              description='Flight controller: pid, lqr or nonlinear mpc'),
        DeclareLaunchArgument('evaluator', default_value='true',
                              description='Evaluate control and write a report after the mission'),
        DeclareLaunchArgument('report_file', default_value='/tmp/quadcopter_control_report.txt',
                              description='Path of the control evaluation report'),
        DeclareLaunchArgument('position_tolerance', default_value='0.25',
                              description='Allowed 3D distance from the reference path [m]'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo_share / 'launch' / 'gazebo.launch.py')),
                                 launch_arguments={'world': world, 'gui': LaunchConfiguration('gui')}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_quadcopter', output='screen',
             arguments=['-entity', 'quadcopter', '-file', str(share / 'models' / 'quadcopter' / 'model.sdf')]),
        Node(package='quadcopter_sim', executable='trajectory_node.py', namespace='quadcopter',
             output='screen', parameters=[{'use_sim_time': True, 'mission_file': mission}]),
        Node(package='quadcopter_sim', executable='motor_allocator', name='motor_allocator',
             namespace='quadcopter', output='screen',
             parameters=[str(share / 'config' / 'motor_model.yaml'), {'use_sim_time': True}],
             condition=IfCondition(PythonExpression(["'", controller, "' != 'mpc'"]))),
        Node(package='quadcopter_sim', executable='quadcopter_controller', name='controller',
             namespace='quadcopter', output='screen',
             parameters=[str(share / 'config' / 'controller.yaml'), {'use_sim_time': True}],
             condition=IfCondition(PythonExpression(["'", controller, "' == 'pid'"]))),
        Node(package='quadcopter_sim', executable='lqr_controller', name='controller',
             namespace='quadcopter', output='screen',
             parameters=[str(share / 'config' / 'lqr_controller.yaml'), {'use_sim_time': True}],
             condition=IfCondition(PythonExpression(["'", controller, "' == 'lqr'"]))),
        Node(package='quadcopter_sim', executable='mpc_controller', name='controller',
             namespace='quadcopter', output='screen',
             parameters=[str(share / 'config' / 'mpc_controller.yaml'), {'use_sim_time': True}],
             condition=IfCondition(PythonExpression(["'", controller, "' == 'mpc'"]))),
        Node(package='quadcopter_sim', executable='control_evaluator.py', name='control_evaluator',
             namespace='quadcopter', output='screen',
             parameters=[{'use_sim_time': True, 'mission_file': mission,
                          'controller_name': controller, 'report_file': report_file,
                          'position_tolerance': position_tolerance}],
             condition=IfCondition(evaluator)),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', str(share / 'rviz' / 'quadcopter.rviz')], condition=IfCondition(use_rviz)),
    ])
