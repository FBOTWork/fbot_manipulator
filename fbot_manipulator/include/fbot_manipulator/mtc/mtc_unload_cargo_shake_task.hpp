#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

// Same as MtcUnloadCargoTask (cargo -> table place), but after each place+retreat finishes, shakes
// the open gripper sideways (hand frame Y, one way then the other) before moving to the next target
// or returning home. A separate task instead of a flag on MtcUnloadCargoTask/addPlaceStages: this
// shake is not wanted on every unload, only when this task type is explicitly requested.
class MtcUnloadCargoShakeTask : public MtcTask
{
public:
    /**
     * @param approach_from_front true: places on the table from the front; the cargo pick stays top-down.
     */
    MtcUnloadCargoShakeTask(
        rclcpp::Node::SharedPtr node,
        const ManipulationGoal& goal,
        bool approach_from_front = false);

    bool buildTask() override;

    std::string firstFailedTargetId() const;

private:
    ManipulationGoal goal_;
    bool approach_from_front_;
};

} // namespace fbot_manipulator
