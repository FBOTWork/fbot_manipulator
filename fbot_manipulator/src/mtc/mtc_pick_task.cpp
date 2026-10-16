#include "fbot_manipulator/mtc/mtc_pick_task.hpp"
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp" // Importing our logic

namespace fbot_manipulator
{

MtcPickTask::MtcPickTask(rclcpp::Node::SharedPtr node,
                         const ManipulationGoal& goal)
    : MtcTask("pick", node),
      goal_(goal)
{
}

bool MtcPickTask::buildTask()
{
    task_.stages()->setName("pick_" + goal_.target_id);
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene, goal_.pick_offset, goal_.target_id);

    // 1. Current State
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    if (object_poses_.find(goal_.target_id) == object_poses_.end()) {
        RCLCPP_ERROR(logger(), "ERROR: target_id '%s' was not found!", goal_.target_id.c_str());
        return false;
    }

    // pick_offset is already applied by MtcSharedLogic::setupWorkspace: the target's collision
    // object, and so object_poses_, is at detection + pick_offset (arm frame)
    geometry_msgs::msg::Pose object_pose = object_poses_[goal_.target_id];

    // 2. Call the shared pick logic
    MtcSharedLogic::addPickStages(
        task_, goal_.target_id, object_pose, current_state, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger()
    );

    // 3. Return Home
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_ready_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator