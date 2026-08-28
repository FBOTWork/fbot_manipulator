#pragma once

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcPlaceTask : public MtcTask
{
public:
    MtcPlaceTask(rclcpp::Node::SharedPtr node,
                 const std::string& object_id,
                 const geometry_msgs::msg::Pose& place_pose,
                 bool approach_from_front = false);

    MtcPlaceTask(rclcpp::Node::SharedPtr node,
                 const std::string& object_id,
                 const std::string& place_pose_name,
                 bool approach_from_front = false);

    bool buildTask() override;

private:
    std::string object_id_;
    geometry_msgs::msg::Pose place_pose_;
    std::string place_pose_name_;
    bool approach_from_front_;
};

} // namespace fbot_manipulator
