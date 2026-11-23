#pragma once

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

// Same as MtcPickAndPlaceTask, but shakes the gripper left/right/back-to-center before releasing
// the object (see MtcShakeSharedLogic::addPlaceStagesWithPreReleaseShake) instead of a plain place.
// A separate task, not a flag on MtcPickAndPlaceTask: PICK_AND_PLACE keeps its current behavior.
class MtcPickAndPlaceShakeTask : public MtcTask
{
public:
    MtcPickAndPlaceShakeTask(rclcpp::Node::SharedPtr node,
                             const ManipulationGoal& goal,
                             bool approach_from_front = false);

    bool buildTask() override;

private:
    ManipulationGoal goal_;
    bool approach_from_front_;
};

} // namespace fbot_manipulator
