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

        poses[0].position.x = -0.07; poses[0].position.y = 0.09; poses[0].position.z = 0.02;
        poses[0].orientation.w = 1.0;

        poses[1].position.x = -0.07; poses[1].position.y = -0.09; poses[1].position.z = 0.02;
        poses[1].orientation.w = 1.0;

        poses[2].position.x = -0.15; poses[2].position.y = 0.09; poses[2].position.z = 0.02;
        poses[2].orientation.w = 1.0;

        poses[3].position.x = -0.15; poses[3].position.y = -0.09; poses[3].position.z = 0.02;
        poses[3].orientation.w = 1.0;

        return poses;
    }();

    if (cargo_id < 0 || cargo_id >= static_cast<int>(kCargoSlotPoses.size()))
    {
        throw std::out_of_range(
            "MtcLoadCargoTask: cargo_id " + std::to_string(cargo_id) +
            " fora do intervalo válido [0, " + std::to_string(kCargoSlotPoses.size() - 1) + "]");
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
    task_.stages()->setName("load_cargo_" + std::to_string(goal_.cargo_id));
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene);

    // 1. Current State
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    if (object_poses_.find(goal_.target_id) == object_poses_.end()) {
        RCLCPP_ERROR(logger(), "FALHA: O target_id '%s' nao foi encontrado!", goal_.target_id.c_str());
        return false;
    }

    // Aplica o pick_offset
    geometry_msgs::msg::Pose object_pose = object_poses_[goal_.target_id];
    object_pose.position.x += goal_.pick_offset.x;
    object_pose.position.y += goal_.pick_offset.y;
    object_pose.position.z += goal_.pick_offset.z;

    // 2. CHAMA O PICK 
    mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
        task_, goal_.target_id, object_pose, current_state, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger()
    );

    // 3. Obtém a pose de destino baseada no cargo_id
    geometry_msgs::msg::Pose place_pose = poseForCargoIndex(goal_.cargo_id);

    // 4. CHAMA O PLACE
    MtcSharedLogic::addPlaceStages(
        task_, goal_.target_id, place_pose, attach_stage, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger()
    );

    // 5. Return Home Final
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_ready_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator