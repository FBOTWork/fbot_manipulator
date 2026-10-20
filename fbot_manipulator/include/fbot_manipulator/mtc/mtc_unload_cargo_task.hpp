#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcUnloadCargoTask : public MtcTask
{
public:
    /**
     * @param approach_from_front true: places on the table from the front; the cargo pick stays top-down.
     */
    MtcUnloadCargoTask(
        rclcpp::Node::SharedPtr node,
        const ManipulationGoal& goal,
        bool approach_from_front = false);

    bool buildTask() override;

    static geometry_msgs::msg::Pose poseForCargoIndex(int cargo_id);

    std::string firstFailedTargetId() const;

private:
    ManipulationGoal goal_;
    bool approach_from_front_;
};

} // namespace fbot_manipulator