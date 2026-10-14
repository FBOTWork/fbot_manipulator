#include "fbot_manipulator/mtc/mtc_load_cargo_task.hpp"
#include <array>
#include <stdexcept>
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"

namespace fbot_manipulator
{

geometry_msgs::msg::Pose MtcLoadCargoTask::poseForCargoIndex(int cargo_id)
{
    static const std::array<geometry_msgs::msg::Pose, 4> kCargoSlotPoses = [] {
        std::array<geometry_msgs::msg::Pose, 4> poses{};

        poses[0].position.x = -0.08; poses[0].position.y = 0.1; poses[0].position.z = 0.04;
        poses[0].orientation.w = 1.0;

        poses[1].position.x = -0.08; poses[1].position.y = -0.11; poses[1].position.z = 0.04;
        poses[1].orientation.w = 1.0;

        poses[2].position.x = -0.13; poses[2].position.y = 0.1; poses[2].position.z = 0.04;
        poses[2].orientation.w = 1.0;

        poses[3].position.x = -0.13; poses[3].position.y = -0.115; poses[3].position.z = 0.04;
        poses[3].orientation.w = 1.0;

        return poses;
    }();

    if (cargo_id < 0 || cargo_id >= static_cast<int>(kCargoSlotPoses.size()))
    {
        throw std::out_of_range(
            "MtcLoadCargoTask: cargo_id " + std::to_string(cargo_id) +
            " out of range [0, " + std::to_string(kCargoSlotPoses.size() - 1) + "]");
    }
    return kCargoSlotPoses[cargo_id];
}

MtcLoadCargoTask::MtcLoadCargoTask(rclcpp::Node::SharedPtr node,
                                   const ManipulationGoal& goal)
    : MtcTask("load_cargo", node),
      goal_(goal)
{
}

bool MtcLoadCargoTask::buildTask()
{
    std::vector<std::string> target_ids = goal_.target_ids;
    std::vector<int> cargo_indices = goal_.cargo_indices;

    if (target_ids.empty() && !goal_.target_id.empty()) {
        target_ids.push_back(goal_.target_id);
    }
    if (cargo_indices.empty() && goal_.cargo_id >= 0) {
        cargo_indices.push_back(goal_.cargo_id);
    }

    if (target_ids.empty() || cargo_indices.empty()) {
        RCLCPP_ERROR(logger(), "FAIL: no target_ids or cargo_indices provided for load_cargo");
        return false;
    }

    if (target_ids.size() != cargo_indices.size()) {
        RCLCPP_ERROR(logger(), "FAIL: target_ids and cargo_indices size mismatch (%zu != %zu)",
                     target_ids.size(), cargo_indices.size());
        return false;
    }

    stage_checkpoints_.clear();

    task_.stages()->setName("load_cargo_" + std::to_string(target_ids.size()));
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene, goal_.pick_offset, target_ids);

    // 1. Current State
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    for (size_t i = 0; i < target_ids.size(); ++i) {
        const std::string& target_id = target_ids[i];
        const int cargo_id = cargo_indices[i];

        if (object_poses_.find(target_id) == object_poses_.end()) {
            RCLCPP_ERROR(logger(), "FAIL: target_id '%s' not found!", target_id.c_str());
            return false;
        }

        // pick_offset is already applied by MtcSharedLogic::setupWorkspace: the target's collision
        // object, and so object_poses_, is at detection + pick_offset (arm frame)
        geometry_msgs::msg::Pose object_pose = object_poses_[target_id];

        // 2. Call the pick logic
        mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
            task_, target_id, object_pose, current_state,
            config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger()
        );

        stage_checkpoints_.emplace_back(target_id, attach_stage);

        // 3. Get the destination pose based on the cargo_id
        geometry_msgs::msg::Pose place_pose = poseForCargoIndex(cargo_id);

        // 4. Call the place logic
        mtc::Stage* place_ik_stage = MtcSharedLogic::addPlaceStages(
            task_, target_id, place_pose, attach_stage,
            config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger()
        );
        // Place checkpoint: if place_ik_stage has no solution after plan(),
        // this target_id is the one that got stuck in the place phase (destination-slot IK).
        stage_checkpoints_.emplace_back(target_id, place_ik_stage);

        current_state = attach_stage;
    }

    // 5. Return Home Final
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_ready_state);
        task_.add(std::move(stage));
    }

    return true;
}

std::string MtcLoadCargoTask::firstFailedTargetId() const
{
    for (const auto& checkpoint : stage_checkpoints_) {
        const std::string& target_id = checkpoint.first;
        const mtc::Stage* stage = checkpoint.second;

        if (stage == nullptr || stage->solutions().empty()) {
            return target_id;
        }
    }
    return {};
}

} // namespace fbot_manipulator