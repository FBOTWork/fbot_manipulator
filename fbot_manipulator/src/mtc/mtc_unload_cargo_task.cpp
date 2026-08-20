#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "fbot_manipulator/mtc/mtc_task.hpp"
#include "fbot_manipulator/mtc/mtc_unload_cargo_task.hpp"
#include <array>
#include <stdexcept>  // for std::out_of_range
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"
#include <moveit/task_constructor/stages.h> 

namespace mtc = ::moveit::task_constructor;

namespace fbot_manipulator
{


// Table of fixed poses per inventory slot (0 to 3).
// Adjust the real values according to your shelf/rack geometry.
geometry_msgs::msg::Pose MtcUnloadCargoTask::poseForCargoIndex(uint8_t cargo_index)
{
    static const std::array<geometry_msgs::msg::Pose, 4> kCargoSlotPoses = [] {
        std::array<geometry_msgs::msg::Pose, 4> poses;

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

    if (cargo_index >= kCargoSlotPoses.size())
    {
        throw std::out_of_range(
            "MtcUnloadCargoTask: cargo_index " + std::to_string(cargo_index) +
            " fora do intervalo válido [0, " + std::to_string(kCargoSlotPoses.size() - 1) + "]");
    }
    return kCargoSlotPoses[cargo_index];
}

// Constructor
MtcUnloadCargoTask::MtcUnloadCargoTask(
    rclcpp::Node::SharedPtr node,
    const std::string& object_id,
    uint8_t cargo_index,
    const geometry_msgs::msg::Pose& place_pose)
    : MtcTask("unload_cargo", node),
      object_id_(object_id),
      cargo_index_(cargo_index),
      place_pose_(place_pose)
{
}


bool MtcUnloadCargoTask::buildTask()
{
    task_.stages()->setName("unload_cargo_" + std::to_string(cargo_index_));
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this);

    // 1. Get the actual pose of the slot according to the index
    geometry_msgs::msg::Pose pick_pose = poseForCargoIndex(cargo_index_);

    // // 2. Ensure MoveIt knows that the object is EXACTLY in the correct slot!
    // // (Adjust the size according to your box/object)
    // geometry_msgs::msg::Vector3 object_size;
    // object_size.x = 0.03; object_size.y = 0.03; object_size.z = 0.03; 

    // addCollisionObject(object_id_, pick_pose, object_size);

    // Current State
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    // PICK (Pick from the slot)
    mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
        task_,
        object_id_,
        pick_pose,
        current_state,
        config_,
        pipeline_planner_,
        cartesian_planner_,
        joint_planner_,
        logger()
    );

    // PLACE (Take to the final location specified in the Action)
    MtcSharedLogic::addPlaceStages(
        task_,
        object_id_,
        place_pose_,
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