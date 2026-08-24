#include "fbot_manipulator/mtc/mtc_unload_cargo_task.hpp"
#include <array>
#include <stdexcept>
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"

namespace fbot_manipulator
{

geometry_msgs::msg::Pose MtcUnloadCargoTask::poseForCargoIndex(int cargo_id)
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
    task_.stages()->setName("unload_cargo_" + std::to_string(goal_.cargo_id));
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene);

    // 1. Obtém a pose REAL do Slot de acordo com o índice e aplica o pick_offset
    geometry_msgs::msg::Pose pick_pose = poseForCargoIndex(goal_.cargo_id);
    pick_pose.position.x += goal_.pick_offset.x;
    pick_pose.position.y += goal_.pick_offset.y;
    pick_pose.position.z += goal_.pick_offset.z;

    // Current State
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    // PICK 
    mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
        task_,
        goal_.target_id,
        pick_pose,
        current_state,
        config_,
        pipeline_planner_,
        cartesian_planner_,
        joint_planner_,
        logger()
    );

    // PLACE 
    MtcSharedLogic::addPlaceStages(
        task_,
        goal_.target_id,
        goal_.place_pose,
        attach_stage,
        config_,
        pipeline_planner_,
        cartesian_planner_,
        joint_planner_,
        logger()
    );

    // Return Home
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_ready_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator