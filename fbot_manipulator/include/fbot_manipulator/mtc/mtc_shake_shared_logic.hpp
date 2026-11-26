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
     * but the gripper shake happens BEFORE release/detach/retreat instead of after: the object is
     * still attached and the gripper still closed while it shakes left, then right (crossing past
     * center), then back to center; the gripper only opens once the arm is back at the exact place
     * pose. The three shake moves use exact distances (min==max): left and the return-to-center move
     * exactly shake_distance each, the middle move exactly twice that, so they cancel out.
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
