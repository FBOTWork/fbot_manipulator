#ifndef FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP
#define FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// Includes of  MoveIt Task Constructor
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/solvers.h>


namespace fbot_manipulator
{

namespace mtc = moveit::task_constructor;
struct MtcConfig;
class MtcTask; 

class MtcSharedLogic
{
public:

    static void setupWorkspace(MtcTask* task_instance);
     
    static mtc::Stage* addPickStages(
        mtc::Task& task,
        const std::string& object_id,
        const geometry_msgs::msg::Pose& object_pose,
        mtc::Stage* current_state,
        const MtcConfig& config,
        std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
        std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
        std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
        rclcpp::Logger logger);

    static void addPlaceStages(
        mtc::Task& task,
        const std::string& object_id,
        const geometry_msgs::msg::Pose& place_pose,
        mtc::Stage* attach_stage, 
        const MtcConfig& config,
        std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
        std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
        std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
        rclcpp::Logger logger);
};

} // namespace fbot_manipulator

#endif // FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP