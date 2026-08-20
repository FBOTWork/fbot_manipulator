#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "fbot_manipulator/mtc/mtc_task.hpp"
#include "fbot_manipulator/mtc/mtc_load_cargo_task.hpp"
#include <moveit/task_constructor/stages.h>
#include <array>
#include <stdexcept>  // for std::out_of_range
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp" 

namespace mtc = ::moveit::task_constructor;

namespace fbot_manipulator
{


// Table of fixed poses per inventory slot (0 to 3).
// Adjust the real values according to your shelf/rack geometry.
geometry_msgs::msg::Pose MtcLoadCargoTask::poseForCargoIndex(uint8_t cargo_index)
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
            "MtcLoadCargoTask: cargo_index " + std::to_string(cargo_index) +
            " fora do intervalo válido [0, " + std::to_string(kCargoSlotPoses.size() - 1) + "]");
    }
    return kCargoSlotPoses[cargo_index];
}

// Canonical constructor: pose is provided externally
MtcLoadCargoTask::MtcLoadCargoTask(rclcpp::Node::SharedPtr node,
                                          const std::string& object_id,
                                          uint8_t cargo_index,
                                          const geometry_msgs::msg::Pose& place_pose)
    : MtcTask("load_cargo", node),
        object_id_(object_id),
        cargo_index_(cargo_index),
        place_pose_(place_pose)
{
}

// Convenience constructor: pose derived from the index
MtcLoadCargoTask::MtcLoadCargoTask(rclcpp::Node::SharedPtr node,
                                          const std::string& object_id,
                                          uint8_t cargo_index)
    : MtcLoadCargoTask(node, object_id, cargo_index, poseForCargoIndex(cargo_index))
{
}


bool MtcLoadCargoTask::buildTask()
{
    task_.stages()->setName("load_cargo_" + std::to_string(cargo_index_));
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this);

    // 1. Current State
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    // Get the object's pose from the map
    geometry_msgs::msg::Pose object_pose = object_poses_[object_id_];

    // 2. CALL THE PICK AND STORE THE RESULT
    mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
        task_, object_id_, object_pose, current_state, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger()
    );

    // 3. Move Home (Optional between Pick and Place)
    // {
    //     auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
    //     stage->setGroup(config_.arm_group_name);
    //     stage->setGoal(config_.arm_ready_state);
    //     task_.add(std::move(stage));
    // }

    // 4. CALL THE PLACE
    MtcSharedLogic::addPlaceStages(
        task_, object_id_, place_pose_, attach_stage, 
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