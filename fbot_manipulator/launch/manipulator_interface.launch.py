import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml
import xacro


def load_yaml(package_name, *path_parts):
    """Load a YAML file from a ROS 2 package share directory."""
    from ament_index_python.packages import get_package_share_directory
    try:
        full_path = os.path.join(get_package_share_directory(package_name), *path_parts)
        with open(full_path, "r") as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        return {}


def load_file(package_name, *path_parts):
    """Load a raw text file (like SRDF) from a ROS 2 package share directory."""
    from ament_index_python.packages import get_package_share_directory
    try:
        full_path = os.path.join(get_package_share_directory(package_name), *path_parts)
        with open(full_path, "r") as f:
            return f.read()
    except Exception as e:
        return ""


def launch_setup(context, *args, **kwargs):
    arm_type_str = LaunchConfiguration("arm_type").perform(context)
    namespace_str = LaunchConfiguration("namespace").perform(context)

    pkg_share = FindPackageShare(package="fbot_manipulator").perform(context)

    if arm_type_str in ["openarm", "left_arm", "right_arm"]:
        moveit_config_pkg = "openarm_bimanual_moveit_config"
        description_pkg = "openarm_description"
        
        config_subfolder = "openarm_v1.0"
        srdf_path_parts = ["config", config_subfolder, "openarm_bimanual.srdf"]
        xacro_path_parts = ["urdf", "openarm_bimanual.urdf.xacro"]
    else:
        moveit_config_pkg = f"{arm_type_str}_moveit_config"
        description_pkg = f"{arm_type_str}_description"
        config_subfolder = arm_type_str
        srdf_path_parts = ["config", config_subfolder, f"{arm_type_str}.srdf"]
        xacro_path_parts = ["urdf", f"{arm_type_str}.urdf.xacro"]

    from ament_index_python.packages import get_package_share_directory
    try:
        xacro_file = os.path.join(get_package_share_directory(description_pkg), *xacro_path_parts)
        doc = xacro.parse(open(xacro_file))
        xacro.process_doc(doc)
        robot_description_content = doc.toxml()
    except Exception as e:
        robot_description_content = ""

    robot_description = {"robot_description": robot_description_content}

    srdf_content = load_file(moveit_config_pkg, *srdf_path_parts)
    robot_description_semantic = {"robot_description_semantic": srdf_content}

    kinematics_yaml = load_yaml(moveit_config_pkg, "config", config_subfolder, "kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    joint_limits_yaml = load_yaml(moveit_config_pkg, "config", config_subfolder, "joint_limits.yaml")
    ompl_planning_yaml = load_yaml(moveit_config_pkg, "config", config_subfolder, "ompl_planning.yaml")

    ompl_planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        },
    }
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml(moveit_config_pkg, "config", config_subfolder, "moveit_controllers.yaml")

    moveit_controllers = {
        "moveit_fake_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_fake_controller_manager/MoveItFakeControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    mtc_config_path = os.path.join(pkg_share, "config", arm_type_str, "mtc_config.yaml")

    manipulator_interface_node = Node(
        package="fbot_manipulator",
        executable="manipulator_interface_node",
        name="manipulator_interface",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"arm_type": arm_type_str},
            {"arm_name": "left_arm"},
            {"robot_description_planning": joint_limits_yaml},
            ompl_planning_pipeline_config,
        ],
        output="screen",
    )

    manipulation_task_server = Node(
        package="fbot_manipulator",
        executable="manipulation_task_server",
        name="manipulation_task_server",
        parameters=[
            mtc_config_path, 
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "mtc.arm_group_name": "left_arm",
                "mtc.hand_group_name": "left_gripper",
                "mtc.eef_name": "left_ee",
                "mtc.hand_frame": "openarm_left_hand_tcp",  
                "mtc.ik_frame": "openarm_left_hand_tcp",   
                "mtc.world_frame": "world",
                "mtc.surface_link": "world",
                "mtc.hand_open_state": "open",
                "mtc.hand_closed_state": "closed",
                "mtc.arm_ready_state": "hands_up",
                "mtc.approach_min": 0.05,
                "mtc.approach_max": 0.10,
                "mtc.lift_min": 0.05,
                "mtc.lift_max": 0.15,
                "mtc.grasp_angle_delta": 0.262,
                "mtc.grasp_frame_rpy": [0.0, -1.571, 3.142],
            },
            {"robot_description_planning": joint_limits_yaml},
            ompl_planning_pipeline_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor,
        ],
        output="screen",
    )

    return [
        manipulator_interface_node,
        manipulation_task_server,
    ]


def generate_launch_description():
    """Entry point required by ROS 2 launch system."""
    arm_type_arg = DeclareLaunchArgument(
        name="arm_type",
        default_value="openarm",
        description="Type of arm to use (e.g., 'openarm', 'xarm6', 'wx200')",
    )

    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="fbot_manipulator",
        description="Namespace for the manipulator interface node",
    )

    return LaunchDescription([
        arm_type_arg,
        namespace_arg,
        OpaqueFunction(function=launch_setup),
    ])