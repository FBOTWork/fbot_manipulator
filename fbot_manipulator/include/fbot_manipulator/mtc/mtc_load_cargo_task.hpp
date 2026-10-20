#pragma once

#include <cstdint>
#include <array>
#include <stdexcept>  
#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcLoadCargoTask : public MtcTask
{
public:
    /**
     * @param approach_from_front true: picks from the table from the front; the cargo place stays top-down.
     */
    MtcLoadCargoTask(rclcpp::Node::SharedPtr node,
                     const ManipulationGoal& goal,
                     bool approach_from_front = false);
    std::string firstFailedTargetId() const;
    bool buildTask() override;

private:

    static geometry_msgs::msg::Pose poseForCargoIndex(int cargo_id);

    ManipulationGoal goal_;
    bool approach_from_front_;
};

} // namespace fbot_manipulator