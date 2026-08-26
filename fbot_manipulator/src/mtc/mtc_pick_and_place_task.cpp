#include "fbot_manipulator/mtc/mtc_pick_and_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"
#include <moveit/task_constructor/stages.h>

namespace mtc = ::moveit::task_constructor;

namespace fbot_manipulator
{

MtcPickAndPlaceTask::MtcPickAndPlaceTask(
    rclcpp::Node::SharedPtr node,
    const std::string& object_id,
    const geometry_msgs::msg::Pose& place_pose)
    : MtcTask("pick_and_place", node),
      object_id_(object_id),
      place_pose_(place_pose),
      use_named_pose_(false)
{
}

MtcPickAndPlaceTask::MtcPickAndPlaceTask(
    rclcpp::Node::SharedPtr node,
    const std::string& object_id,
    const std::string& place_pose_name)
    : MtcTask("pick_and_place", node),
      object_id_(object_id),
      place_pose_name_(place_pose_name),
      use_named_pose_(true)
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

    // 1. Current State
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    // 2. ADD PICK STAGES (sem close gripper)
    if (!object_poses_.count(object_id_)) {
        RCLCPP_ERROR(logger(), "[MtcPickAndPlaceTask] Objeto '%s' não encontrado!", 
                     object_id_.c_str());
        return false;
    }

    geometry_msgs::msg::Pose object_pose = object_poses_[object_id_];

    mtc::Stage* attach_object_stage = MtcSharedLogic::addPickStages(
        task_,
        object_id_,
        object_pose,
        current_state,
        config_,
        pipeline_planner_,
        cartesian_planner_,
        joint_planner_,
        logger()
    );

    // 3. ADD PLACE STAGES (sem open gripper)
    if (use_named_pose_) {
        // ---- Move to named SRDF place pose ----
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("move to place pose", pipeline_planner_);
            stage->setGroup(config_.arm_group_name);
            stage->setGoal(place_pose_name_);
            task_.add(std::move(stage));
        }

        // Detach object
        {
            std::string prefix = "openarm_left";
            if (config_.hand_group_name.find("right") != std::string::npos) {
                prefix = "openarm_right";
            }
            
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject(object_id_, prefix + "_hand");
            task_.add(std::move(stage));
        }

        // Remove collision object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("remove object");
            stage->removeObject(object_id_);
            task_.add(std::move(stage));
        }
    } else {
        // Usar shared logic para place com pose geométrica
        MtcSharedLogic::addPlaceStages(
            task_,
            object_id_,
            place_pose_,
            attach_object_stage,
            config_,
            pipeline_planner_,
            cartesian_planner_,
            joint_planner_,
            logger()
        );
    }

    // 4. Return Home
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_home_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator