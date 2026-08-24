#include "fbot_manipulator/mtc/mtc_pick_task.hpp"
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp" // Importando nossa lógica!

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

    // Pega a pose do objeto do mapa
    geometry_msgs::msg::Pose object_pose = object_poses_[goal_.target_id];
    object_pose.position.x += goal_.pick_offset.x;
    object_pose.position.y += goal_.pick_offset.y;
    object_pose.position.z += goal_.pick_offset.z;

    // 2. CHAMA A LÓGICA COMPARTILHADA DE PICK
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