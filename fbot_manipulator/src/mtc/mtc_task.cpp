#include "fbot_manipulator/mtc/mtc_task.hpp"
#include <chrono>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/object_color.hpp>

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

    RCLCPP_INFO(logger(), "[MtcTask:%s] Reconfigured for arm: %s (eef: %s, hand_frame: %s)",
                task_name_.c_str(), arm_name.c_str(), config_.eef_name.c_str(), config_.hand_frame.c_str());
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
    moveit_msgs::msg::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;

    moveit_msgs::msg::ObjectColor obj_color;
    obj_color.id = object_id;
    obj_color.color.r = r;
    obj_color.color.g = g;
    obj_color.color.b = b;
    obj_color.color.a = a;

    planning_scene_msg.object_colors.push_back(obj_color);
    psi_.applyPlanningScene(planning_scene_msg); 
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
    node_->get_parameter_or("mtc.arm_ready_state", config_.arm_ready_state, std::string("holdup"));
    node_->get_parameter_or("mtc.approach_min", config_.approach_min, 0.05);
    node_->get_parameter_or("mtc.approach_max", config_.approach_max, 0.15);
    node_->get_parameter_or("mtc.lift_min", config_.lift_min, 0.05);
    node_->get_parameter_or("mtc.lift_max", config_.lift_max, 0.15);
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

    RCLCPP_INFO(logger(), "[MtcTask:%s] Config loaded: arm='%s', hand='%s', frame='%s'",
                task_name_.c_str(), config_.arm_group_name.c_str(), config_.hand_group_name.c_str(), config_.hand_frame.c_str());
}

void MtcTask::setupSolvers()
{
    pipeline_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    pipeline_planner_->setMaxVelocityScalingFactor(0.5);
    pipeline_planner_->setMaxAccelerationScalingFactor(0.5);

    cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner_->setMaxVelocityScalingFactor(0.5);
    cartesian_planner_->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner_->setStepSize(0.005);

    joint_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
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
    } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(logger(), "[MtcTask:" << task_name_ << "] Init failed:\n" << e);
        return false;
    }

    if (!task_.plan(config_.max_solutions)) {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] Planning failed", task_name_.c_str());
        task_.printState();
        return false;
    }

    RCLCPP_INFO(logger(), "[MtcTask:%s] Planning succeeded with %zu solutions", task_name_.c_str(), task_.solutions().size());
    return true;
}

bool MtcTask::execute()
{
    if (task_.solutions().empty()) {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] No solutions to execute", task_name_.c_str());
        return false;
    }

    const auto& solution = task_.solutions().front();

    task_.introspection().publishSolution(*solution);
    
    RCLCPP_INFO(logger(), "[MtcTask:%s] Solution published. Execute via RViz 'Motion Planning Tasks' panel.", 
                task_name_.c_str());
    
    return true;
}
} // namespace fbot_manipulator