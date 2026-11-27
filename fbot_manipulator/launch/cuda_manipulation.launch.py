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
one-time host setup -- see cuda_planning_setup.md. This launch file prepends the CUDA 13
toolkit (PATH) and the pip-installed CUDA 12 runtime (LD_LIBRARY_PATH) itself for the
isaac_ros_cumotion node below, so you don't need to export them in the shell first -- without
them, that node's first-ever extension JIT-compile fails immediately with the Ubuntu-packaged
(too old) nvcc, which looks like a hang later (the action call waits forever for a planner node
that already crashed), not like a build error, since the crash happens before anyone sends it
a goal.

Example:
    ros2 launch fbot_manipulator cuda_manipulation.launch.py hardware_type:=fake
    ros2 launch fbot_manipulator cuda_manipulation.launch.py hardware_type:=actual
"""

import os

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
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
            'use_rviz',
            default_value='false',
            choices=('true', 'false'),
            description='Open the MoveIt RViz window from xsarm_moveit (use_moveit_rviz).',
        ),
        DeclareLaunchArgument(
            'use_world_frame',
            default_value='true',
            choices=('true', 'false'),
            description=(
                "Add the interbotix world -> wx200/base_link link. 'true' for the arm alone; "
                "'false' when something else (the micky URDF) already parents wx200/base_link."
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
            'cumotion_venv',
            default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'cumotion_venv']),
            description=(
                "uv venv (python3.10) holding NumPy 2, torch, warp-lang... for the cuMotion node "
                "only, so they don't shadow the system NumPy 1.x the vision nodes need. Create it "
                "with: uv venv --python /usr/bin/python3.10 ~/cumotion_venv && uv pip install "
                "--python ~/cumotion_venv/bin/python numpy==2.2.6 scipy==1.15.3 torch==2.9.1 "
                "warp-lang==1.5.0 trimesh ninja pyyaml. Ignored if the folder doesn't exist."
            ),
        ),
        DeclareLaunchArgument(
            'cumotion_startup_delay',
            default_value='30.0',
            description=(
                "Seconds to wait after starting isaac_ros_cumotion before starting "
                "fbot_manipulator (ignored for planner_backend:=ompl, which has no cuMotion "
                "node to wait for). The standalone cuMotion node JIT-compiles several CUDA "
                "extensions; measured directly on this machine with a *warm* JIT cache (already "
                "compiled once before), 'cuMotion is ready for planning queries!' took 20.4s "
                "from process start -- 30s leaves some margin over that measured value. On the "
                "very first run ever (nothing cached yet) it took well over a minute; raise this "
                "accordingly, or watch isaac_ros_cumotion's own terminal for that exact message "
                "and launch fbot_manipulator separately instead of using this combined launch "
                "file. If manipulation_task_server sends a planning request before the node is "
                "ready, the action call just sits waiting for a server that isn't accepting "
                "goals yet -- indistinguishable from 'planning never finishes'."
            ),
        ),
    ]

    hardware_type = LaunchConfiguration('hardware_type')
    planner_backend = LaunchConfiguration('planner_backend')
    cumotion_xrdf_file = LaunchConfiguration('cumotion_xrdf_file')
    cumotion_urdf_file = LaunchConfiguration('cumotion_urdf_file')
    cumotion_tool_frame = LaunchConfiguration('cumotion_tool_frame')

    # Prepend (not overwrite) so the system's own PATH/LD_LIBRARY_PATH still apply for
    # everything else -- only isaac_ros_cumotion needs these ahead of what's already there.
    # Harmless to set even for planner_backend:=ompl, where they just go unused.
    set_cuda_env = [
        AppendEnvironmentVariable('PATH', '/usr/local/cuda-13.0/bin', prepend=True),
        AppendEnvironmentVariable(
            'LD_LIBRARY_PATH',
            PathJoinSubstitution([
                EnvironmentVariable('HOME'), '.local', 'lib', 'python3.10', 'site-packages',
                'nvidia', 'cuda_runtime', 'lib',
            ]),
            prepend=True,
        ),
        AppendEnvironmentVariable('LD_LIBRARY_PATH', '/usr/local/cuda-13.0/lib64', prepend=True),
    ]

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
            'use_moveit_rviz': LaunchConfiguration('use_rviz'),
            'use_world_frame': LaunchConfiguration('use_world_frame'),
        }.items(),
    )

    # 2. The standalone GPU planner node. Only needed (and only started) for planner_backend:=cumotion.
    # Runs with the uv venv's site-packages first on PYTHONPATH and the user site disabled, so it gets
    # NumPy 2 / torch from the venv while every other node keeps the system NumPy 1.x. The scoped group
    # restores the environment afterwards.
    def _cumotion_planner_setup(context):
        venv = LaunchConfiguration('cumotion_venv').perform(context)
        site = os.path.join(venv, 'lib', 'python3.10', 'site-packages')
        env = []
        if os.path.isdir(site):
            env = [
                SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
                AppendEnvironmentVariable('PYTHONPATH', site, prepend=True),
            ]
        else:
            print(f'[cuda_manipulation] venv {venv} not found: cuMotion uses the system/user Python')
        include = IncludeLaunchDescription(
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
        )
        return [GroupAction(scoped=True, actions=env + [include])]

    cumotion_planner = OpaqueFunction(function=_cumotion_planner_setup, condition=_is_cumotion())

    # 3. fbot_manipulator itself (manipulator_interface_node + manipulation_task_server).
    # For planner_backend:=cumotion this must wait for the GPU node above to actually be ready
    # (see cumotion_startup_delay's description) -- started immediately otherwise, since there's
    # no cuMotion node to wait for on the ompl path.
    # Arguments are resolved here (OpaqueFunction), not when the timer fires: when this launch is
    # included from a scoped group (micky_behavior robot_bringup), the group has already dropped
    # these launch configurations by the time a delayed include would evaluate them.
    def _manipulator_interface_setup(context):
        args = {
            name: LaunchConfiguration(name).perform(context)
            for name in (
                'hardware_type', 'use_world_frame', 'planner_backend',
                'cumotion_xrdf_file', 'cumotion_urdf_file', 'cumotion_tool_frame',
            )
        }
        args['robot_model'] = 'wx200'
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('fbot_manipulator'), 'launch', 'manipulator_interface_wx200.launch.py',
                ])
            ),
            launch_arguments=args.items(),
        )
        if args['planner_backend'] != 'cumotion':
            return [include]
        delay = float(LaunchConfiguration('cumotion_startup_delay').perform(context))
        return [TimerAction(period=delay, actions=[include])]

    manipulator_interface = OpaqueFunction(function=_manipulator_interface_setup)

    return LaunchDescription(declared_arguments + set_cuda_env + [
        xsarm_moveit, cumotion_planner, manipulator_interface,
    ])
