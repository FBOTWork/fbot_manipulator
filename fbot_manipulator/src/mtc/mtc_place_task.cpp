#include "fbot_manipulator/mtc/mtc_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"

namespace fbot_manipulator
{

MtcPlaceTask::MtcPlaceTask(rclcpp::Node::SharedPtr node,
                           const ManipulationGoal& goal)
    : MtcTask("place", node),
      goal_(goal)
{
}

bool MtcPlaceTask::buildTask()
{
    std::string target_id = goal_.target_id;
    if (target_id.empty() && !goal_.target_ids.empty()) {
        target_id = goal_.target_ids.front();
    }
    if (target_id.empty()) {
        RCLCPP_ERROR(logger(), "FAIL: no target_id provided for place");
        return false;
    }

    task_.stages()->setName("place_" + target_id);
    task_.loadRobotModel(node_);

    // Names OR poses OR explicit joints, never combined. A single-element list means this place target.
    PlaceTargets targets;
    if (!MtcSharedLogic::resolvePlaceTargets(
            task_, config_, goal_.place_pose_names, goal_.place_poses,
            goal_.place_joint_targets, goal_.place_pose, logger(), targets)) {
        return false;
    }
    if (targets.count() != 1) {
        RCLCPP_ERROR(logger(), "FAIL: place accepts exactly 1 destination, received %zu", targets.count());
        return false;
    }

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene, goal_.pick_offset, target_id);

    // 1. Current state (object assumed already attached)
    mtc::Stage* attach_object_stage = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        attach_object_stage = stage.get();
        task_.add(std::move(stage));
    }

    // 2. Shared place
    geometry_msgs::msg::Pose place_pose;
    place_pose.orientation.w = 1.0;
    const std::vector<double>* place_joint_target = nullptr;
    if (targets.named()) {
        place_joint_target = &targets.joint_targets.front();
    } else {
        place_pose = targets.poses.front();
    }

    mtc::Stage* place_ik_stage = MtcSharedLogic::addPlaceStages(
        task_, target_id, place_pose, attach_object_stage,
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger(),
        place_joint_target);
    if (!place_ik_stage) {
        RCLCPP_ERROR(logger(), "FAIL: addPlaceStages failed for '%s'", target_id.c_str());
        return false;
    }

    // 3. Return home
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_home_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator