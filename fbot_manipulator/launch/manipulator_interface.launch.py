import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml


def load_yaml(package_name, *path_parts):
    """Load a YAML file from a ROS 2 package share directory."""
    from ament_index_python.packages import get_package_share_directory
    full_path = os.path.join(get_package_share_directory(package_name), *path_parts)
    with open(full_path, "r") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare(package="fbot_manipulator")

    arm_type = LaunchConfiguration("arm_type")
    namespace = LaunchConfiguration("namespace")
    planner_backend = LaunchConfiguration("planner_backend").perform(context)

    # Create the manipulator interface node
    manipulator_interface_node = Node(
        package="fbot_manipulator",
        executable="manipulator_interface_node",
        #namespace=namespace,
        name="manipulator_interface",
        parameters=[
            {"arm_type": arm_type},
        ],
    )

    # MTC config file path
    mtc_config = PathJoinSubstitution([
        pkg_share, "config", arm_type, "mtc_config.yaml"
    ])

    # ---- Load MoveIt configs from xarm_moveit_config ----
    # These are needed so the MTC node can plan and execute independently
    # without requiring a combined launch with xarm_moveit_config.

    # Kinematics (IK solver)
    kinematics_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm6", "kinematics.yaml")

    # OMPL planning pipeline. This is always loaded (it stays registered even when
    # planner_backend:=cumotion, so cuMotion has a CPU fallback pipeline to fall back on).
    ompl_defaults_yaml = load_yaml(
        "xarm_moveit_config", "config", "moveit_configs", "ompl_defaults.yaml")
    ompl_planning_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm6", "ompl_planning.yaml")
    gripper_ompl_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm_gripper", "ompl_planning.yaml")
    ompl_defaults_yaml.update(ompl_planning_yaml)
    ompl_defaults_yaml.update(gripper_ompl_yaml)
    ompl_planning_yaml = ompl_defaults_yaml

    ompl_pipeline_entry = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters":
            "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints",
        "start_state_max_bounds_error": 0.1,
    }
    ompl_pipeline_entry.update(ompl_planning_yaml)

    # ---- Planning pipeline selection: CPU (OMPL, default) or GPU (NVIDIA Isaac ROS cuMotion) ----
    # See fbot_manipulator/README.md -- "GPU-accelerated planning" for the full setup.
    if planner_backend == "ompl":
        planning_pipeline_config = {
            "default_planning_pipeline": "ompl",
            "planning_pipelines": ["ompl"],
            "ompl": ompl_pipeline_entry,
        }
        cumotion_action_server_params = {}
        mtc_planning_pipeline = {"mtc.planning_pipeline": "ompl"}
    elif planner_backend == "cumotion":
        xrdf_file = LaunchConfiguration("cumotion_xrdf_file").perform(context)
        urdf_file = LaunchConfiguration("cumotion_urdf_file").perform(context)
        tool_frame = LaunchConfiguration("cumotion_tool_frame").perform(context)
        if not xrdf_file or not urdf_file or not tool_frame:
            raise RuntimeError(
                "planner_backend:=cumotion requires cumotion_xrdf_file, cumotion_urdf_file and "
                "cumotion_tool_frame to be set (see fbot_manipulator/README.md -- "
                "'GPU-accelerated planning')."
            )
        try:
            cumotion_planning_yaml = load_yaml(
                "isaac_ros_cumotion_moveit", "config", "isaac_ros_cumotion_planning.yaml")
        except Exception as exc:
            raise RuntimeError(
                "planner_backend:=cumotion needs the 'isaac_ros_cumotion_moveit' package "
                "(NVIDIA Isaac ROS cuMotion) installed and sourced -- see "
                "fbot_manipulator/README.md -- 'GPU-accelerated planning'."
            ) from exc

        planning_pipeline_config = {
            "default_planning_pipeline": "isaac_ros_cumotion",
            "planning_pipelines": ["ompl", "isaac_ros_cumotion"],
            "ompl": ompl_pipeline_entry,
            "isaac_ros_cumotion": cumotion_planning_yaml,
        }
        cumotion_action_server_params = {
            "cumotion_action_server.xrdf_file_path": xrdf_file,
            "cumotion_action_server.urdf_file_path": urdf_file,
            "cumotion_action_server.tool_frame": tool_frame,
            # The planning scene (collision objects) already comes from MoveIt here, so the
            # cuMotion server doesn't need its own ESDF world built from a depth camera.
            "cumotion_action_server.read_esdf_world": False,
        }
        mtc_planning_pipeline = {"mtc.planning_pipeline": "isaac_ros_cumotion"}
    else:
        raise RuntimeError(
            f"Unknown planner_backend '{planner_backend}' (expected 'ompl' or 'cumotion').")

    # Joint limits
    joint_limits_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm6", "joint_limits.yaml")
    gripper_joint_limits_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm_gripper", "joint_limits.yaml")
    joint_limits_yaml["joint_limits"].update(gripper_joint_limits_yaml["joint_limits"])

    # Controllers (fake for simulation)
    controllers_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm6", "fake_controllers.yaml")
    gripper_controllers_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm_gripper", "fake_controllers.yaml")
    for name in gripper_controllers_yaml.get("controller_names", []):
        if name not in controllers_yaml["controller_names"]:
            controllers_yaml["controller_names"].append(name)
        controllers_yaml[name] = gripper_controllers_yaml[name]

    moveit_controllers = {
        "moveit_fake_controller_manager": controllers_yaml,
        "moveit_controller_manager":
            "moveit_fake_controller_manager/MoveItFakeControllerManager",
    }

    # Trajectory execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    # Planning scene monitor
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Create the manipulation task server node (MTC-based action server)
    manipulation_task_server = Node(
        package="fbot_manipulator",
        executable="manipulation_task_server",
        name="manipulation_task_server",
        parameters=[
            mtc_config,
            mtc_planning_pipeline,
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_yaml},
            planning_pipeline_config,
            cumotion_action_server_params,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor,
        ],
        output="screen",
    )

    return [manipulator_interface_node, manipulation_task_server]


def generate_launch_description():
    """Generate the launch description."""

    declared_arguments = [
        DeclareLaunchArgument(
            name="arm_type",
            default_value="xarm6",
            description="Type of arm to use (e.g., 'xarm6', 'wx200')",
        ),
        DeclareLaunchArgument(
            name="namespace",
            default_value="fbot_manipulator",
            description="Namespace for the manipulator interface node",
        ),
        DeclareLaunchArgument(
            name="planner_backend",
            default_value="ompl",
            choices=("ompl", "cumotion"),
            description=(
                "MoveIt planning pipeline used by the MTC task server. 'ompl' (default) plans "
                "on CPU. 'cumotion' offloads planning to the GPU via NVIDIA Isaac ROS cuMotion "
                "-- requires isaac_ros_cumotion_moveit installed plus cumotion_xrdf_file / "
                "cumotion_urdf_file / cumotion_tool_frame. See README.md -- "
                "'GPU-accelerated planning'."
            ),
        ),
        DeclareLaunchArgument(
            name="cumotion_xrdf_file",
            default_value="",
            description=(
                "Path to the robot's XRDF file (cuMotion robot description). Required when "
                "planner_backend:=cumotion."
            ),
        ),
        DeclareLaunchArgument(
            name="cumotion_urdf_file",
            default_value="",
            description=(
                "Path to the URDF file passed to the cuMotion action server. Required when "
                "planner_backend:=cumotion."
            ),
        ),
        DeclareLaunchArgument(
            name="cumotion_tool_frame",
            default_value="",
            description=(
                "End-effector link name passed to the cuMotion action server (e.g. 'link_tcp' "
                "for xarm6). Required when planner_backend:=cumotion."
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
