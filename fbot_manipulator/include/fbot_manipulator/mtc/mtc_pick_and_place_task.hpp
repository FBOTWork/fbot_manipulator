#pragma once

#include "fbot_manipulator/mtc/mtc_task.hpp"
#include <geometry_msgs/msg/pose.hpp>

namespace fbot_manipulator
{

class MtcPickAndPlaceTask : public MtcTask
{
public:
    using Ptr = std::shared_ptr<MtcPickAndPlaceTask>;

    // Construtor com pose geométrica
    MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                       const std::string& object_id,
                       const geometry_msgs::msg::Pose& place_pose);

    // Construtor com pose nomeada (SRDF)
    MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                       const std::string& object_id,
                       const std::string& place_pose_name);

    bool buildTask() override;

private:
    std::string object_id_;
    geometry_msgs::msg::Pose place_pose_;
    std::string place_pose_name_;
    bool use_named_pose_;
};

} // namespace fbot_manipulator