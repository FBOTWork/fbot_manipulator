#pragma once

#include <cstdint>
#include <array>
#include <stdexcept>  // para std::out_of_range
#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcLoadCargoTask : public MtcTask
{
public:
    // Destination pose already resolved ("canonical" usage: the caller already knows the pose).
    MtcLoadCargoTask(rclcpp::Node::SharedPtr node,
                      const std::string& object_id,
                         uint8_t cargo_index,
                         const geometry_msgs::msg::Pose& place_pose);

    // Convenience: pose automatically derived from the slot index (0 to 4).
    MtcLoadCargoTask(rclcpp::Node::SharedPtr node,
                         const std::string& object_id,
                         uint8_t cargo_index);

    bool buildTask() override;

private:
    // Resolve a destination pose for an inventory slot. Throws std::out_of_range
    // if cargo_index is out of the valid range.
    static geometry_msgs::msg::Pose poseForCargoIndex(uint8_t cargo_index);

    std::string object_id_;
    uint8_t cargo_index_;
    geometry_msgs::msg::Pose place_pose_;
};

} // namespace fbot_manipulator