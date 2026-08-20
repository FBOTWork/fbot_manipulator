#ifndef FBOT_MANIPULATOR_MTC_TASK_HPP_
#define FBOT_MANIPULATOR_MTC_TASK_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <Eigen/Geometry>

namespace mtc = moveit::task_constructor;

namespace fbot_manipulator
{

struct MtcConfig
{
    std::string arm_group_name{"left_arm"};
    std::string hand_group_name{"left_gripper"};
    std::string eef_name{"left_ee"};
    std::string hand_frame{"openarm_left_link7"};
    std::string world_frame{"world"};
    std::string surface_link{"world"};

    std::string hand_open_state{"open"};
    std::string hand_closed_state{"closed"};
    std::string arm_home_state{"home"};
    std::string arm_ready_state{"ready"};

    double approach_min{0.01};
    double approach_max{0.10};
    double lift_min{0.01};
    double lift_max{0.10};
    double place_lower_min{0.01};
    double place_lower_max{0.10};
    double retreat_min{0.01};
    double retreat_max{0.10};

    int max_solutions{5};

    double grasp_angle_delta{M_PI / 12.0};
    double pour_angle_delta{M_PI / 6.0};
    double pour_wait_time{2.0};
    double pour_side_offset{0.12};
    double pour_above_offset{0.08};

    std::vector<double> grasp_frame_rpy{0.0, -M_PI / 2, M_PI};
    Eigen::Isometry3d grasp_frame_transform{Eigen::Isometry3d::Identity()};
};

class MtcTask
{
public:
    using Ptr = std::shared_ptr<MtcTask>;
    using ConstPtr = std::shared_ptr<const MtcTask>;

    MtcTask(const std::string& task_name, rclcpp::Node::SharedPtr node);
    virtual ~MtcTask() = default;

    void setCollisionObjectColor(const std::string& object_id, float r, float g, float b, float a = 1.0);
    void addCollisionObject(const std::string& object_id,
                            const geometry_msgs::msg::Pose& pose,
                            const geometry_msgs::msg::Vector3& size);
    void removeCollisionObject(const std::string& object_id);
    void detachAndRemoveObject(const std::string& object_id);

    void initTask();
    virtual bool buildTask() = 0; 

    bool plan();
    bool execute();

    void loadConfigForArm(const std::string& arm_name);

    rclcpp::Logger logger() const { return node_->get_logger(); }

protected:
    void loadConfig();
    void setupSolvers();

    std::string task_name_;
    rclcpp::Node::SharedPtr node_;
    MtcConfig config_;

    mtc::Task task_;
    moveit::planning_interface::PlanningSceneInterface psi_;

    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner_;
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner_;

    std::map<std::string, geometry_msgs::msg::Pose> object_poses_;
    std::map<std::string, MtcConfig> arm_configs_;
};

} // namespace fbot_manipulator

#endif // FBOT_MANIPULATOR_MTC_TASK_HPP_