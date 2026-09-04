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
    MtcLoadCargoTask(rclcpp::Node::SharedPtr node,
                     const ManipulationGoal& goal); 

    bool buildTask() override;

private:
    
    static geometry_msgs::msg::Pose poseForCargoIndex(int cargo_id); 

    ManipulationGoal goal_; 
};

} // namespace fbot_manipulator