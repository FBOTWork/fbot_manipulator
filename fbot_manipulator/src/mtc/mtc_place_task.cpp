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
    task_.stages()->setName("place_" + goal_.target_id);
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene);

    // 1. Current State (object assumed already attached)
    mtc::Stage* attach_object_stage = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        attach_object_stage = stage.get();
        task_.add(std::move(stage));
    }

    // 2. CHAMA A LÓGICA COMPARTILHADA DE PLACE
    MtcSharedLogic::addPlaceStages(
        task_, goal_.target_id, goal_.place_pose, attach_object_stage, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger()
    );
    
    // 3. Return Home
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_home_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator