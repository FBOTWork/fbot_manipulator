#include "fbot_manipulator/mtc/mtc_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"
#include <moveit/task_constructor/stages.h> 

namespace mtc = ::moveit::task_constructor;

namespace fbot_manipulator
{

MtcPlaceTask::MtcPlaceTask(rclcpp::Node::SharedPtr node,
                           const std::string& object_id,
                           const geometry_msgs::msg::Pose& place_pose)
    : MtcTask("place", node),
      object_id_(object_id),
      place_pose_(place_pose)
{
}

MtcPlaceTask::MtcPlaceTask(rclcpp::Node::SharedPtr node,
                           const std::string& object_id,
                           const std::string& place_pose_name)
    : MtcTask("place", node),
      object_id_(object_id),
      place_pose_name_(place_pose_name)
{
}

bool MtcPlaceTask::buildTask()
{
    task_.stages()->setName("place_" + object_id_);
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.eef_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this);

    // 1. Current State (object assumed already attached)
    mtc::Stage* attach_object_stage = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        attach_object_stage = stage.get();
        task_.add(std::move(stage));
    }

    // 2. Place stages via shared logic (pose geométrica ou nomeada)
    if (place_pose_name_.empty())
    {
        MtcSharedLogic::addPlaceStages(
            task_, object_id_, place_pose_, attach_object_stage,
            config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger()
        );
    }
    else
    {
        MtcSharedLogic::addPlaceStagesNamed(
            task_, object_id_, place_pose_name_, attach_object_stage,
            config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger()
        );
    }

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