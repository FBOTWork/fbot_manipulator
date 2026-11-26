"""Bring up the full GPU-accelerated manipulation stack for the wx200 in one launch.

Combines the three pieces that otherwise need their own terminals (see
fbot_manipulator/docs/cuda_planning_setup.md for why each exists):

  1. interbotix_xsarm_moveit xsarm_moveit.launch.py  -- move_group + the hardware/fake interface
  2. isaac_ros_cumotion isaac_ros_cumotion.launch.py  -- the standalone GPU planner node that
     isaac_ros_cumotion_moveit's MoveIt plugin forwards planning requests to
  3. fbot_manipulator manipulator_interface_wx200.launch.py -- the MTC action server, with
     planner_backend:=cumotion

This only covers the ROS side. The CUDA toolkit, the isaac_ros_cumotion*/curobo apt packages,
their pip dependencies, and the five source patches in fbot_manipulator/docs/ are still a
one-time host setup -- see cuda_planning_setup.md. The isaac_ros_cumotion node needs
PATH/LD_LIBRARY_PATH pointed at the CUDA 13 toolkit (same guide, step 3) *before* this launch
file is run, same as if it were started on its own.

Example:
    ros2 launch fbot_manipulator cuda_manipulation.launch.py hardware_type:=fake
    ros2 launch fbot_manipulator cuda_manipulation.launch.py hardware_type:=actual
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'hardware_type',
            default_value='actual',
            choices=('fake', 'actual'),
            description=(
                "Passed straight through to xsarm_moveit.launch.py -- 'fake' for simulation "
                "(no real motors), 'actual' for the real wx200."
            ),
        ),
        DeclareLaunchArgument(
            'planner_backend',
            default_value='cumotion',
            choices=('ompl', 'cumotion'),
            description=(
                "Planning pipeline for manipulation_task_server. This launch file exists for "
                "'cumotion' (the default); 'ompl' is here only so you can A/B against CPU "
                "planning without editing anything -- the isaac_ros_cumotion node below still "
                "starts either way, it's just unused on the 'ompl' path."
            ),
        ),
        DeclareLaunchArgument(
            'cumotion_xrdf_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('fbot_manipulator'), 'config', 'interbotix_arm', 'wx200_gripper.xrdf',
            ]),
            description='XRDF, shared by the standalone cuMotion node and manipulation_task_server.',
        ),
        DeclareLaunchArgument(
            'cumotion_urdf_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('fbot_manipulator'), 'config', 'interbotix_arm', 'wx200_gripper.urdf',
            ]),
            description='URDF, shared by the standalone cuMotion node and manipulation_task_server.',
        ),
        DeclareLaunchArgument(
            'cumotion_tool_frame',
            default_value='wx200/ee_gripper_link',
            description='End-effector link name passed to the cuMotion action server.',
        ),
        DeclareLaunchArgument(
            'cumotion_startup_delay',
            default_value='10.0',
            description=(
                "Seconds to wait after starting isaac_ros_cumotion before starting "
                "fbot_manipulator (ignored for planner_backend:=ompl, which has no cuMotion "
                "node to wait for). The standalone cuMotion node JIT-compiles several CUDA "
                "extensions on its first-ever run (a minute or more; a few seconds once "
                "cached); if manipulation_task_server sends it a planning request before it's "
                "finished, the action call just sits waiting for a server that isn't accepting "
                "goals yet -- indistinguishable from 'planning never finishes'. 10s covers a "
                "warm cache; raise it for the very first run on a given cuRobo/PyTorch install, "
                "or watch isaac_ros_cumotion's own terminal for 'cuMotion is ready for planning "
                "queries!' and launch fbot_manipulator separately instead of using this combined "
                "launch file."
            ),
        ),
    ]

    hardware_type = LaunchConfiguration('hardware_type')
    planner_backend = LaunchConfiguration('planner_backend')
    cumotion_xrdf_file = LaunchConfiguration('cumotion_xrdf_file')
    cumotion_urdf_file = LaunchConfiguration('cumotion_urdf_file')
    cumotion_tool_frame = LaunchConfiguration('cumotion_tool_frame')

    # Fresh Condition instances every call -- Condition objects aren't meant to be shared
    # across multiple actions.
    def _is_cumotion():
        return IfCondition(PythonExpression(["'", planner_backend, "' == 'cumotion'"]))

    def _is_not_cumotion():
        return UnlessCondition(PythonExpression(["'", planner_backend, "' == 'cumotion'"]))

    # 1. move_group + the wx200 hardware/fake interface
    xsarm_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'), 'launch', 'xsarm_moveit.launch.py',
            ])
        ),
        launch_arguments={
            'robot_model': 'wx200',
            'hardware_type': hardware_type,
        }.items(),
    )

    # 2. The standalone GPU planner node. Only needed (and only started) for planner_backend:=cumotion.
    cumotion_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_cumotion'), 'launch', 'isaac_ros_cumotion.launch.py',
            ])
        ),
        launch_arguments={
            'cumotion_planner.urdf_path': cumotion_urdf_file,
            'cumotion_planner.yml_file_path': cumotion_xrdf_file,
            'cumotion_planner.tool_frame': cumotion_tool_frame,
            'cumotion_planner.read_esdf_world': 'false',
            'cumotion_planner.joint_states_topic': '/wx200/joint_states',
        }.items(),
        condition=_is_cumotion(),
    )

    # 3. fbot_manipulator itself (manipulator_interface_node + manipulation_task_server).
    # For planner_backend:=cumotion this must wait for the GPU node above to actually be ready
    # (see cumotion_startup_delay's description) -- started immediately otherwise, since there's
    # no cuMotion node to wait for on the ompl path.
    def _manipulator_interface_include(condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('fbot_manipulator'), 'launch', 'manipulator_interface_wx200.launch.py',
                ])
            ),
            launch_arguments={
                'planner_backend': planner_backend,
                'cumotion_xrdf_file': cumotion_xrdf_file,
                'cumotion_urdf_file': cumotion_urdf_file,
                'cumotion_tool_frame': cumotion_tool_frame,
            }.items(),
            condition=condition,
        )

    manipulator_interface_immediate = _manipulator_interface_include(condition=_is_not_cumotion())

    manipulator_interface_delayed = TimerAction(
        period=LaunchConfiguration('cumotion_startup_delay'),
        actions=[_manipulator_interface_include()],
        condition=_is_cumotion(),
    )

    return LaunchDescription(declared_arguments + [
        xsarm_moveit, cumotion_planner, manipulator_interface_immediate, manipulator_interface_delayed,
    ])
