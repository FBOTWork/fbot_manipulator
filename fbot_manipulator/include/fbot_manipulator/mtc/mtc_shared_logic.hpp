#ifndef FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP
#define FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP

#include <string>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// MoveIt Task Constructor includes
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/solvers.h>

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

namespace mtc = moveit::task_constructor;

struct PlaceTargets
{
    std::vector<geometry_msgs::msg::Pose> poses;       // cartesian mode
    std::vector<std::vector<double>> joint_targets;    // named joint mode

    bool named() const { return !joint_targets.empty(); }
    std::size_t count() const { return named() ? joint_targets.size() : poses.size(); }
};

class MtcSharedLogic
{
public:
    static void setupWorkspace(MtcTask* task_instance,
                               const std::vector<ObjectDetection>& objects_scene,
                               geometry_msgs::msg::Vector3& pick_offset,
                               const std::string& target_id);

    static void setupWorkspace(MtcTask* task_instance,
                               const std::vector<ObjectDetection>& objects_scene,
                               geometry_msgs::msg::Vector3& pick_offset,
                               const std::vector<std::string>& target_ids);

    /**
     * @brief Resolves the place destinations. Exactly one mode: names (the `poses` section from YAML),
     * cartesian poses, or explicit joint targets, never combined.
     * A list with 1 element means only that place target.
     */
    static bool resolvePlaceTargets(
        mtc::Task& task,
        const MtcConfig& config,
        const std::vector<std::string>& names,
        const std::vector<geometry_msgs::msg::Pose>& poses,
        const std::vector<std::vector<double>>& joint_targets,
        const geometry_msgs::msg::Pose& legacy_single_pose,
        rclcpp::Logger logger,
        PlaceTargets& out);

    /**
     * @brief Builds all Pick stages and injects them into the task.
     * @return Pointer to the 'attach_object' stage (required for subsequent Place stages).
     */
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

    /**
     * @brief Builds all Place stages and injects them into the task.
     * If place_joint_target is provided, the destination is a joint-space configuration.
     */
    static mtc::Stage* addPlaceStages(
        mtc::Task& task,
        const std::string& object_id,
        const geometry_msgs::msg::Pose& place_pose,
        mtc::Stage* attach_stage,
        const MtcConfig& config,
        std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
        std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
        std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
        rclcpp::Logger logger,
        const std::vector<double>* place_joint_target = nullptr);
};

} // namespace fbot_manipulator

#endif // FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP