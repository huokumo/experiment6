"""Experiment-5 assets + measured pose + planar execution + depth obstacles."""
import math
import os
from pathlib import Path
import shlex
import tempfile
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from experiment6_core.scene import generate_scene


def boolean(context,key):
    value=LaunchConfiguration(key).perform(context).lower()
    if value not in ('true','false','1','0'):
        raise ValueError(f'{key} must be true or false')
    return value in ('true','1')


def setup(context):
    exp5=Path(get_package_share_directory('experiment5_depth'))
    share=Path(get_package_share_directory('experiment6_navigation'))
    prefix=Path(get_package_prefix('experiment6_navigation'))
    if not (prefix/'lib/libr1_planar_motion_system.so').is_file():
        raise RuntimeError('Build with BUILD_FORTRESS_PLUGIN=ON before starting simulation')
    # One generated scene per launch; the original experiment-5 package is read-only.
    runtime=Path(tempfile.mkdtemp(prefix='experiment6_'))
    spawn=tuple(float(LaunchConfiguration(key).perform(context))
                for key in ('robot_x','robot_y','robot_z','robot_yaw'))
    rate=float(LaunchConfiguration('depth_rate').perform(context))
    if not math.isfinite(rate): raise ValueError('depth_rate must be finite')
    gui=boolean(context,'gui')
    world=generate_scene(exp5,runtime/'experiment6.world',spawn=spawn,depth_rate=rate,gui=gui)
    actions=[SetEnvironmentVariable('PYTHONNOUSERSITE','1'),
             SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE',
                '1' if boolean(context,'software_rendering') else '0')]
    for variable,extra in (
        ('IGN_GAZEBO_RESOURCE_PATH',str(exp5/'models')),
        ('GZ_SIM_RESOURCE_PATH',str(exp5/'models')),
        ('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',str(prefix/'lib')),
        ('GZ_SIM_SYSTEM_PLUGIN_PATH',str(prefix/'lib'))):
        actions.append(SetEnvironmentVariable(variable,extra+os.pathsep+os.environ.get(variable,'')))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args':('-r -v 3 ' if gui else '-s -r --headless-rendering -v 3 ')+shlex.quote(str(world)),
                          'gz_version':'6','on_exit_shutdown':'true'}.items()))
    actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',
        name='r1_state_command_bridge',output='screen',
        parameters=[{'config_file':str(share/'config/sim_bridge.yaml'),'use_sim_time':True}]))
    # Camera-only frame override: never apply this override to odometry.
    actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',
        name='r1_depth_bridge',output='screen',
        parameters=[{'config_file':str(exp5/'config/depth_bridge.yaml'),
                     'override_frame_id':'depth_camera_optical_frame','use_sim_time':True}]))
    actions.append(Node(package='robot_state_publisher',executable='robot_state_publisher',
        name='r1_state_publisher',output='screen',parameters=[{'use_sim_time':True,
        'robot_description':ParameterValue((exp5/'description/r1_depth.urdf').read_text(encoding='utf-8'),value_type=str)}]))
    actions.append(Node(package='tf2_ros',executable='static_transform_publisher',
        name='map_to_odom',output='screen',arguments=['--x','0','--y','0','--z','0',
        '--yaw','0','--pitch','0','--roll','0','--frame-id','map','--child-frame-id','odom'],
        parameters=[{'use_sim_time':True}]))
    actions.append(Node(package='experiment6_navigation',executable='pose_node.py',
        name='r1_pose_publisher',output='screen',parameters=[{'use_sim_time':True}]))
    actions.append(Node(package='experiment6_navigation',executable='motion_node.py',
        name='r1_motion_adapter',output='screen',parameters=[str(share/'config/interfaces.yaml'),
        {'use_sim_time':True,'require_depth':ParameterValue(LaunchConfiguration('require_depth'),value_type=bool)}]))
    actions.append(Node(package='experiment6_navigation',executable='depth_node.py',
        name='r1_depth_obstacles',output='screen',parameters=[str(share/'config/interfaces.yaml'),{'use_sim_time':True}]))
    if boolean(context,'rviz'):
        actions.append(Node(package='rviz2',executable='rviz2',name='experiment6_rviz',
            arguments=['-d',str(share/'config/experiment6.rviz')],parameters=[{'use_sim_time':True}]))
    return actions


def generate_launch_description():
    defaults={'robot_x':'0.0','robot_y':'0.0','robot_z':'0.0','robot_yaw':'0.0',
              'depth_rate':'10.0','gui':'true','rviz':'false',
              'software_rendering':'true','require_depth':'false'}
    return LaunchDescription([DeclareLaunchArgument(key,default_value=value)
                              for key,value in defaults.items()]+[OpaqueFunction(function=setup)])
