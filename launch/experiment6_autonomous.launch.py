"""Launch the three foundation interfaces and one wall-bypass controller."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    controller = LaunchConfiguration('controller').perform(context)
    if controller not in ('reference', 'student'):
        raise ValueError('controller must be reference or student')
    share = Path(get_package_share_directory('experiment6_navigation'))
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share/'launch/experiment6_interfaces.launch.py')),
        launch_arguments={key: LaunchConfiguration(key) for key in
                          ('gui', 'rviz', 'software_rendering', 'robot_x', 'robot_y',
                           'robot_z', 'robot_yaw', 'depth_rate', 'require_depth')}.items())
    executable = 'avoidance_node.py' if controller == 'reference' else 'student_avoidance_node.py'
    parameters = [{'use_sim_time': True,
                   'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
                   'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float)}]
    if controller == 'reference':
        parameters[0]['side'] = LaunchConfiguration('side')
    return [base, Node(package='experiment6_navigation', executable=executable,
                       name='r1_wall_bypass' if controller == 'reference' else 'r1_student_wall_bypass',
                       output='screen', parameters=parameters)]


def generate_launch_description():
    defaults = {'controller':'reference', 'goal_x':'4.2', 'goal_y':'0.0',
                'side':'left', 'gui':'true', 'rviz':'false',
                'software_rendering':'true', 'robot_x':'0.0', 'robot_y':'0.0',
                'robot_z':'0.0', 'robot_yaw':'0.0', 'depth_rate':'10.0',
                'require_depth':'true'}
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()]+[OpaqueFunction(function=setup)])
