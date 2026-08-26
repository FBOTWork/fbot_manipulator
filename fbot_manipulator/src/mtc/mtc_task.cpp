#include "fbot_manipulator/mtc/mtc_task.hpp"
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace fbot_manipulator
{

MtcTask::MtcTask(const std::string& task_name, rclcpp::Node::SharedPtr node)
    : task_name_(task_name), node_(node)
{
    loadConfig();
    setupSolvers();
}

void MtcTask::loadConfigForArm(const std::string& arm_name)
{
    std::string prefix = (arm_name == "right_arm") ? "right" : "left";
    config_.arm_group_name = arm_name;
    config_.hand_group_name = prefix + "_gripper";
    config_.eef_name = prefix + "_ee";
    config_.hand_frame = "openarm_" + prefix + "_hand_tcp";
}

void MtcTask::initTask()
{
    task_.reset();
    task_.stages()->setName(task_name_);
    task_.loadRobotModel(node_);
    
    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("hand", config_.hand_group_name);
    task_.setProperty("hand_grasping_frame", config_.hand_frame);
    task_.setProperty("ik_frame", config_.hand_frame);
}

void MtcTask::setCollisionObjectColor(const std::string& object_id, float r, float g, float b, float a)
{
    moveit_msgs::msg::PlanningScene msg;
    msg.is_diff = true;
    moveit_msgs::msg::ObjectColor color;
    color.id = object_id;
    color.color.r = r; color.color.g = g; color.color.b = b; color.color.a = a;
    msg.object_colors.push_back(color);
    psi_.applyPlanningScene(msg); 
}

void MtcTask::loadConfig()
{
    node_->get_parameter_or("mtc.arm_group_name", config_.arm_group_name, std::string("left_arm"));
    node_->get_parameter_or("mtc.eef_name", config_.eef_name, std::string("left_ee"));
    node_->get_parameter_or("mtc.hand_group_name", config_.hand_group_name, std::string("left_gripper"));
    node_->get_parameter_or("mtc.hand_frame", config_.hand_frame, std::string("openarm_left_hand_tcp"));
    node_->get_parameter_or("mtc.world_frame", config_.world_frame, std::string("world"));
    node_->get_parameter_or("mtc.surface_link", config_.surface_link, std::string("world"));
    node_->get_parameter_or("mtc.hand_open_state", config_.hand_open_state, std::string("open"));
    node_->get_parameter_or("mtc.hand_closed_state", config_.hand_closed_state, std::string("closed"));
    node_->get_parameter_or("mtc.arm_home_state", config_.arm_home_state, std::string("home"));
    node_->get_parameter_or("mtc.arm_ready_state", config_.arm_ready_state, std::string("hands_up"));
    node_->get_parameter_or("mtc.approach_min", config_.approach_min, 0.05);
    node_->get_parameter_or("mtc.approach_max", config_.approach_max, 0.15);
    node_->get_parameter_or("mtc.lift_min", config_.lift_min, 0.05);
    node_->get_parameter_or("mtc.lift_max", config_.lift_max, 0.15);
    node_->get_parameter_or("mtc.retreat_min", config_.retreat_min, 0.05);
    node_->get_parameter_or("mtc.retreat_max", config_.retreat_max, 0.15);
    node_->get_parameter_or("mtc.max_solutions", config_.max_solutions, 5);
    node_->get_parameter_or("mtc.grasp_angle_delta", config_.grasp_angle_delta, 0.262);
    
    double grasp_offset = 0.0;
    node_->get_parameter_or("mtc.grasp_offset", grasp_offset, 0.0);
    std::vector<double> grasp_rpy;
    node_->get_parameter_or("mtc.grasp_frame_rpy", grasp_rpy, std::vector<double>{0.0, -M_PI / 2, M_PI});
    
    config_.grasp_frame_transform = Eigen::Isometry3d::Identity();
    config_.grasp_frame_transform.rotate(
        Eigen::AngleAxisd(grasp_rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(grasp_rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(grasp_rpy[0], Eigen::Vector3d::UnitX()));
    config_.grasp_frame_transform.translate(Eigen::Vector3d(grasp_offset, 0.0, 0.0));
}

void MtcTask::setupSolvers()
{
    pipeline_planner_ = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_, "ompl");
    pipeline_planner_->setMaxVelocityScalingFactor(0.5);
    pipeline_planner_->setMaxAccelerationScalingFactor(0.5);

    cartesian_planner_ = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
    cartesian_planner_->setMaxVelocityScalingFactor(0.5);
    cartesian_planner_->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner_->setStepSize(0.005);

    joint_planner_ = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
}

void MtcTask::addCollisionObject(const std::string& object_id, const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Vector3& size)
{
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = config_.world_frame;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = { size.x, size.y, size.z };
    object.pose = pose;
    psi_.applyCollisionObject(object);
    object_poses_[object_id] = pose;
}

void MtcTask::removeCollisionObject(const std::string& object_id)
{
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = config_.world_frame;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi_.applyCollisionObject(object);
    object_poses_.erase(object_id);
}

void MtcTask::detachAndRemoveObject(const std::string& object_id)
{
    moveit_msgs::msg::AttachedCollisionObject detach;
    detach.link_name = config_.hand_frame;
    detach.object.id = object_id;
    detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi_.applyAttachedCollisionObject(detach);
    removeCollisionObject(object_id);
}

bool MtcTask::plan()
{
    try {
        task_.init();
        return (task_.plan(config_.max_solutions) == moveit::core::MoveItErrorCode::SUCCESS);
    } catch (...) {
        return false;
    }
}

bool MtcTask::execute()
{
    if (task_.solutions().empty()) return false;
    moveit_task_constructor_msgs::msg::Solution solution_msg;
    task_.solutions().front()->toMsg(solution_msg);
    return executeSolution(solution_msg);
}

bool MtcTask::executeSolution(const moveit_task_constructor_msgs::msg::Solution& solution_msg)
{
    moveit::planning_interface::MoveGroupInterface arm_group(node_, config_.arm_group_name);
    moveit::planning_interface::MoveGroupInterface gripper_group(node_, config_.hand_group_name);
    
    const auto arm_joints = arm_group.getJointNames();
    const auto gripper_joints = gripper_group.getJointNames();

    moveit_msgs::msg::RobotTrajectory merged_traj;
    std::string current_group = "";

    auto execute_merged = [&]() -> bool {
        if (merged_traj.joint_trajectory.points.empty()) return true;

        auto& active_group = (current_group == "arm") ? arm_group : gripper_group;
        auto& active_joints = (current_group == "arm") ? arm_joints : gripper_joints;
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = merged_traj;

        auto current_state = active_group.getCurrentJointValues();
        if (!current_state.empty()) {
            auto& pt0 = plan.trajectory_.joint_trajectory.points[0];
            for (size_t j = 0; j < plan.trajectory_.joint_trajectory.joint_names.size(); ++j) {
                auto it = std::find(active_joints.begin(), active_joints.end(), plan.trajectory_.joint_trajectory.joint_names[j]);
                if (it != active_joints.end()) {
                    pt0.positions[j] = current_state[std::distance(active_joints.begin(), it)];
                }
            }
            pt0.time_from_start.sec = 0;
            pt0.time_from_start.nanosec = 0;
            pt0.velocities.assign(pt0.positions.size(), 0.0);
            pt0.accelerations.assign(pt0.positions.size(), 0.0);
        }

        active_group.setStartStateToCurrentState();
        bool ok = (active_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        merged_traj = moveit_msgs::msg::RobotTrajectory(); 
        return ok;
    };

    for (const auto& sub : solution_msg.sub_trajectory) {
        bool has_scene_diff = !sub.scene_diff.robot_state.attached_collision_objects.empty() || 
                              !sub.scene_diff.world.collision_objects.empty() || 
                              !sub.scene_diff.allowed_collision_matrix.entry_names.empty();

        if (has_scene_diff) {
            if (!execute_merged()) return false;
            psi_.applyPlanningScene(sub.scene_diff);
        }

        if (!sub.trajectory.joint_trajectory.points.empty()) {
            bool is_arm = false;
            for (const auto& j : sub.trajectory.joint_trajectory.joint_names) {
                if (std::find(arm_joints.begin(), arm_joints.end(), j) != arm_joints.end()) {
                    is_arm = true; break;
                }
            }
            
            std::string target_group = is_arm ? "arm" : "gripper";
            if (!current_group.empty() && current_group != target_group) {
                if (!execute_merged()) return false;
            }
            current_group = target_group;

            if (merged_traj.joint_trajectory.points.empty()) {
                merged_traj = sub.trajectory;
            } else {
                double offset = rclcpp::Duration(merged_traj.joint_trajectory.points.back().time_from_start).seconds();
                for (size_t p = 1; p < sub.trajectory.joint_trajectory.points.size(); ++p) { 
                    auto pt = sub.trajectory.joint_trajectory.points[p];
                    pt.time_from_start = rclcpp::Duration::from_seconds(offset + rclcpp::Duration(pt.time_from_start).seconds());
                    merged_traj.joint_trajectory.points.push_back(pt);
                }
            }
        }
    }
    return execute_merged();
}

} // namespace fbot_manipulator