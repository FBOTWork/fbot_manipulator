#include "fbot_manipulator/mtc/mtc_unload_cargo_task.hpp"
#include <array>
#include <stdexcept>
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
            " fora do intervalo válido [0, " + std::to_string(kCargoSlotPoses.size() - 1) + "]");
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
    std::vector<geometry_msgs::msg::Pose> place_poses = goal_.place_poses;

    if (target_ids.empty() && !goal_.target_id.empty()) {
        target_ids.push_back(goal_.target_id);
    }
    if (cargo_indices.empty() && goal_.cargo_id >= 0) {
        cargo_indices.push_back(goal_.cargo_id);
    }
    if (place_poses.empty()) {
        place_poses.push_back(goal_.place_pose);
    }

    if (target_ids.empty() || cargo_indices.empty() || place_poses.empty()) {
        RCLCPP_ERROR(logger(), "FAIL: no target_ids, cargo_indices, or place_poses provided for unload_cargo");
        return false;
    }

    if (target_ids.size() != cargo_indices.size() || target_ids.size() != place_poses.size()) {
        RCLCPP_ERROR(logger(), "FAIL: vectors size mismatch (targets: %zu, cargos: %zu, poses: %zu)",
                     target_ids.size(), cargo_indices.size(), place_poses.size());
        return false;
    }

    stage_checkpoints_.clear();

    task_.stages()->setName("unload_cargo_" + std::to_string(target_ids.size()));
    task_.loadRobotModel(node_);

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
        const geometry_msgs::msg::Pose& place_pose = place_poses[i];

        geometry_msgs::msg::Pose pick_pose = poseForCargoIndex(cargo_id);

        MtcTask::addCollisionObject(target_id, pick_pose, tag_size);

        // 2. CHAMA O PICK
        mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
            task_,
            target_id,
            pick_pose,
            current_state,
            config_,
            pipeline_planner_,
            cartesian_planner_,
            joint_planner_,
            logger()
        );
        stage_checkpoints_.emplace_back(target_id, attach_stage);

        // 3. CHAMA O PLACE
        mtc::Stage* place_ik_stage = MtcSharedLogic::addPlaceStages(
            task_,
            target_id,
            place_pose,
            attach_stage,
            config_,
            pipeline_planner_,
            cartesian_planner_,
            joint_planner_,
            logger()
        );
        stage_checkpoints_.emplace_back(target_id, place_ik_stage);

        // Atualiza a referência de estado inicial para a próxima iteração do loop
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