#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

namespace mtc = moveit::task_constructor;

// Shared between the "*_SHAKE" task variants only (MtcUnloadCargoShakeTask,
// MtcPickAndPlaceShakeTask) -- kept out of MtcSharedLogic on purpose, so the shake behavior never
// touches the addPlaceStages every other place/pick_and_place/unload_cargo task goes through.
class MtcShakeSharedLogic
{
public:
    /**
     * @brief Same as MtcSharedLogic::addPlaceStages (cartesian place only -- no place_joint_target),
     * but instead of a plain release, runs a seat-the-object sequence BEFORE release/detach/retreat:
     * the hand opens only part way (config.shake_intermediate_open_ratio between hand_open_state
     * and hand_closed_state, per joint), shakes sideways
     * along the hand frame's Z, lifts off by config.shake_turn_lift_distance, turns the wrist 90
     * degrees (config.shake_turn_angle) in place, lowers back down by the same distance, and shakes
     * again -- now along what the turn made the other horizontal axis. Only then does the gripper
     * fully open at the exact place pose. Every shake/lift/lower move uses exact distances
     * (min==max, or an equal lift/lower pair) so they cancel out and the arm ends each leg at the
     * same pose it started -- no tolerance on purpose, so a surroundings mismatch fails planning
     * loudly instead of releasing off the intended place point.
     * @return Pointer to the 'place pose IK' stage (required for chaining further targets).
     */
    static mtc::Stage* addPlaceStagesWithPreReleaseShake(
        mtc::Task& task,
        const std::string& target_id,
        const geometry_msgs::msg::Pose& place_pose,
        mtc::Stage* attach_stage,
        const MtcConfig& config,
        std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
        std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
        std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
        rclcpp::Logger logger,
        bool approach_from_front = false,
        GraspYaw grasp_yaw = GraspYaw::FREE);
};

} // namespace fbot_manipulator
