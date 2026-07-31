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
        const std::string& object_id,
        uint8_t cargo_index,
        const geometry_msgs::msg::Pose& place_pose);

    bool buildTask() override;
    
    static geometry_msgs::msg::Pose poseForCargoIndex(uint8_t cargo_index);

private:
    std::string object_id_;
    uint8_t cargo_index_;
    geometry_msgs::msg::Pose place_pose_;
};

} // namespace fbot_manipulator