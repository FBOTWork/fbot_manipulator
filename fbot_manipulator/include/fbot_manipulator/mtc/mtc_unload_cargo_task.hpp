#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcUnloadCargoTask : public MtcTask
{
public:
    MtcUnloadCargoTask(
        rclcpp::Node::SharedPtr node,
        const ManipulationGoal& goal);

    bool buildTask() override;
    
    static geometry_msgs::msg::Pose poseForCargoIndex(int cargo_id);

private:
    ManipulationGoal goal_;
};

} // namespace fbot_manipulator