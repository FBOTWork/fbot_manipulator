#pragma once

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcPickAndPlaceTask : public MtcTask
{
public:
    MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                        const ManipulationGoal& goal);

    bool buildTask() override;

private:
    ManipulationGoal goal_;
};

} // namespace fbot_manipulator