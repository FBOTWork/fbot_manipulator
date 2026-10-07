#include "fbot_manipulator/mtc/mtc_unload_cargo_task.hpp"
#include <array>
#include <stdexcept>
#include <string>
#include <vector>
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"
#include "fbot_manipulator/mtc/mtc_task.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace fbot_manipulator
{

geometry_msgs::msg::Pose MtcUnloadCargoTask::poseForCargoIndex(int cargo_id)
{
    static const std::array<geometry_msgs::msg::Pose, 4> kCargoSlotPoses = [] {
        std::array<geometry_msgs::msg::Pose, 4> poses{};

        poses[0].position.x = -0.08; poses[0].position.y = 0.1; poses[0].position.z = 0.02;
        poses[0].orientation.w = 1.0;

        poses[1].position.x = -0.08; poses[1].position.y = -0.11; poses[1].position.z = 0.02;
        poses[1].orientation.w = 1.0;

        poses[2].position.x = -0.13; poses[2].position.y = 0.1; poses[2].position.z = 0.02;
        poses[2].orientation.w = 1.0;

        poses[3].position.x = -0.13; poses[3].position.y = -0.115; poses[3].position.z = 0.02;
        poses[3].orientation.w = 1.0;

        return poses;
    }();

    if (cargo_id < 0 || cargo_id >= static_cast<int>(kCargoSlotPoses.size()))
    {
        throw std::out_of_range(
            "MtcUnloadCargoTask: cargo_id " + std::to_string(cargo_id) +
            " out of range [0, " + std::to_string(kCargoSlotPoses.size() - 1) + "]");
    }
    return kCargoSlotPoses[cargo_id];
}

MtcUnloadCargoTask::MtcUnloadCargoTask(
    rclcpp::Node::SharedPtr node,
    const ManipulationGoal& goal)
    : MtcTask("unload_cargo", node),
      goal_(goal)
{
}

bool MtcUnloadCargoTask::buildTask()
{
    std::vector<std::string> target_ids = goal_.target_ids;
    std::vector<int> cargo_indices = goal_.cargo_indices;

    if (target_ids.empty() && !goal_.target_id.empty()) {
        target_ids.push_back(goal_.target_id);
    }
    if (cargo_indices.empty() && goal_.cargo_id >= 0) {
        cargo_indices.push_back(goal_.cargo_id);
    }

    stage_checkpoints_.clear();

    task_.stages()->setName("unload_cargo_" + std::to_string(target_ids.size()));
    task_.loadRobotModel(node_);

    // Names OR poses OR explicit joints, never combined. A single-element list means this place target.
    PlaceTargets targets;
    if (!MtcSharedLogic::resolvePlaceTargets(
            task_, config_, goal_.place_pose_names, goal_.place_poses,
            goal_.place_joint_targets, goal_.place_pose, logger(), targets)) {
        return false;
    }

    if (target_ids.empty() || cargo_indices.empty() || targets.count() == 0) {
        RCLCPP_ERROR(logger(), "FAIL: no target_ids, cargo_indices, or place targets provided for unload_cargo");
        return false;
    }
    if (target_ids.size() != cargo_indices.size() || target_ids.size() != targets.count()) {
        RCLCPP_ERROR(logger(), "FAIL: vectors size mismatch (targets: %zu, cargos: %zu, place targets: %zu)",
                     target_ids.size(), cargo_indices.size(), targets.count());
        return false;
    }

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene, goal_.pick_offset, target_ids);

    ::geometry_msgs::msg::Vector3 tag_size;
    tag_size.x = 0.04;
    tag_size.y = 0.04;
    tag_size.z = 0.04;

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

        // Named/joint mode: cartesian orientation does not apply.
        // Cartesian mode: the goal pose (position + orientation) is used as the target.
        geometry_msgs::msg::Pose place_pose;
        place_pose.orientation.w = 1.0;
        const std::vector<double>* place_joint_target = nullptr;
        if (targets.named()) {
            place_joint_target = &targets.joint_targets[i];
        } else {
            place_pose = targets.poses[i];
        }

        geometry_msgs::msg::Pose pick_pose;
        try {
            pick_pose = poseForCargoIndex(cargo_id);
        } catch (const std::out_of_range& e) {
            RCLCPP_ERROR(logger(), "FAIL: %s", e.what());
            return false;
        }

        MtcTask::addCollisionObject(target_id, pick_pose, tag_size);

        // 2. PICK
        mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
            task_, target_id, pick_pose, current_state,
            config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger());
        if (!attach_stage) {
            RCLCPP_ERROR(logger(), "FAIL: addPickStages failed for '%s'", target_id.c_str());
            return false;
        }
        stage_checkpoints_.emplace_back(target_id, attach_stage);

        // 3. PLACE
        mtc::Stage* place_ik_stage = MtcSharedLogic::addPlaceStages(
            task_, target_id, place_pose, attach_stage,
            config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger(),
            place_joint_target);
        if (!place_ik_stage) {
            RCLCPP_ERROR(logger(), "FAIL: addPlaceStages failed for '%s'", target_id.c_str());
            return false;
        }
        stage_checkpoints_.emplace_back(target_id, place_ik_stage);

        current_state = place_ik_stage;
    }

    // Return Home
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_ready_state);
        task_.add(std::move(stage));
    }

    return true;
}

std::string MtcUnloadCargoTask::firstFailedTargetId() const
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