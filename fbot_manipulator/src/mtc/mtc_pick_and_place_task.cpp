#include "fbot_manipulator/mtc/mtc_pick_and_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"
#include <moveit/task_constructor/stages.h>

namespace mtc = ::moveit::task_constructor;

namespace fbot_manipulator
{

MtcPickAndPlaceTask::MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                                         const std::string& object_id,
                                         const geometry_msgs::msg::Pose& place_pose)
    : MtcTask("pick_and_place", node),
      object_id_(object_id),
      place_pose_(place_pose)
{
}

MtcPickAndPlaceTask::MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                                         const std::string& object_id,
                                         const std::string& place_pose_name)
    : MtcTask("pick_and_place", node),
      object_id_(object_id),
      place_pose_name_(place_pose_name)
{
}

bool MtcPickAndPlaceTask::buildTask()
{
    task_.stages()->setName("pick_and_place_" + object_id_);
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.eef_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this);

    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    geometry_msgs::msg::Pose object_pose = object_poses_[object_id_];

    mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
        task_, object_id_, object_pose, current_state, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger()
    );

    if (!attach_stage) {
        RCLCPP_ERROR(logger(), "[MtcTask:pick_and_place] addPickStages returned nullptr! Cannot proceed to place.");
        return false;
    }

    if (place_pose_name_.empty()) {
        MtcSharedLogic::addPlaceStages(
            task_, object_id_, place_pose_, attach_stage, 
            config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger()
        );
    } else {
        RCLCPP_INFO(logger(), "[MtcTask:pick_and_place] Using named place pose: '%s'", place_pose_name_.c_str());
        
        auto move_stage = std::make_unique<mtc::stages::MoveTo>("move to place pose", pipeline_planner_);
        move_stage->setGroup(config_.arm_group_name);
        move_stage->setGoal(place_pose_name_);
        task_.add(std::move(move_stage));

        auto release_stage = std::make_unique<mtc::stages::MoveTo>("release object", joint_planner_);
        release_stage->setGroup(config_.hand_group_name);
        release_stage->setGoal(config_.hand_open_state);
        task_.add(std::move(release_stage));

        auto detach_stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        detach_stage->detachObject(object_id_, config_.hand_frame);
        task_.add(std::move(detach_stage));

        auto remove_stage = std::make_unique<mtc::stages::ModifyPlanningScene>("remove object");
        remove_stage->removeObject(object_id_);
        task_.add(std::move(remove_stage));
    }

    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("ensure object removed");
        stage->removeObject(object_id_);
        task_.add(std::move(stage));
    }
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        
        std::string home_pose = config_.arm_home_state.empty() ? config_.arm_ready_state : config_.arm_home_state;
        stage->setGoal(home_pose);

        stage->setTimeout(5.0);
        
        task_.add(std::move(stage));
    }

    RCLCPP_INFO(logger(), "[MtcTask:pick_and_place] Task built successfully");
    return true;
}

} // namespace fbot_manipulator