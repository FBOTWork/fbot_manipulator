#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <string>
#include <map>

namespace fbot_manipulator
{

struct MtcConfig {
    std::string arm_group_name;
    std::string hand_group_name;
    std::string eef_name;
    std::string hand_frame;
    std::string world_frame;
    std::string surface_link;
    std::string hand_open_state;
    std::string hand_closed_state;
    std::string arm_home_state;
    std::string arm_ready_state;
    
    double approach_min;
    double approach_max;
    double lift_min;
    double lift_max;
    double retreat_min;
    double retreat_max;
    int max_solutions;
    double grasp_angle_delta;
    Eigen::Isometry3d grasp_frame_transform;
};

class MtcTask
{
public:
    using Ptr = std::shared_ptr<MtcTask>;

    MtcTask(const std::string& task_name, rclcpp::Node::SharedPtr node);
    virtual ~MtcTask() = default;

    virtual bool buildTask() = 0;
    bool plan();
    bool execute();
    bool executeSolution(const moveit_task_constructor_msgs::msg::Solution& solution_msg);

    void loadConfigForArm(const std::string& arm_name);
    void addCollisionObject(const std::string& object_id, const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Vector3& size);
    void removeCollisionObject(const std::string& object_id);
    void detachAndRemoveObject(const std::string& object_id);
    void setCollisionObjectColor(const std::string& object_id, float r, float g, float b, float a);

protected:
    void initTask();
    void loadConfig();
    void setupSolvers();
    
    // Função devolvida para que as classes filhas (Pick, Place) consigam fazer o log
    rclcpp::Logger logger() const { return node_->get_logger(); }

    std::string task_name_;
    rclcpp::Node::SharedPtr node_;
    moveit::task_constructor::Task task_;
    moveit::planning_interface::PlanningSceneInterface psi_;
    MtcConfig config_;
    std::map<std::string, geometry_msgs::msg::Pose> object_poses_;

    std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> pipeline_planner_;
    std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesian_planner_;
    std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> joint_planner_;
};

} // namespace fbot_manipulator