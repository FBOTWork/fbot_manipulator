#include "fbot_manipulator/mtc/mtc_load_cargo_task.hpp"
#include <array>
#include <stdexcept> 
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace fbot_manipulator
{

geometry_msgs::msg::Pose MtcLoadCargoTask::poseForCargoIndex(int cargo_id)
{
    static const std::array<geometry_msgs::msg::Pose, 4> kCargoSlotPoses = [] {
        std::array<geometry_msgs::msg::Pose, 4> poses{};

        poses[0].position.x = -0.08; poses[0].position.y = 0.1; poses[0].position.z = 0.04;
        poses[0].orientation.w = 1.0;

        poses[1].position.x = -0.08; poses[1].position.y = -0.11; poses[1].position.z = 0.04;
        poses[1].orientation.w = 1.0;

        poses[2].position.x = -0.13; poses[2].position.y = 0.1; poses[2].position.z = 0.04;
        poses[2].orientation.w = 1.0;

        poses[3].position.x = -0.13; poses[3].position.y = -0.115; poses[3].position.z = 0.04;
        poses[3].orientation.w = 1.0;

        return poses;
    }();

    if (cargo_id < 0 || cargo_id >= static_cast<int>(kCargoSlotPoses.size()))
    {
        throw std::out_of_range(
            "MtcLoadCargoTask: cargo_id " + std::to_string(cargo_id) +
            " out of range [0, " + std::to_string(kCargoSlotPoses.size() - 1) + "]");
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

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene, goal_.pick_offset, goal_.target_id);

    // 1. Current State
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    if (object_poses_.find(goal_.target_id) == object_poses_.end()) {
        RCLCPP_ERROR(logger(), "FAIL: target_id '%s' not founded!", goal_.target_id.c_str());
        return false;
    }

    // O offset entra em relação ao frame do braço/base, mas a pose efetiva usada pelo MTC
    // precisa ser aplicada no frame da câmera. Portanto convertemos o offset para o frame da
    // câmera antes de somá-lo à pose do objeto.
    geometry_msgs::msg::Pose object_pose = object_poses_[goal_.target_id];

    geometry_msgs::msg::Vector3 translated_offset = goal_.pick_offset;
    const std::vector<std::string> camera_frames = {
        "camera_link",
        "camera_color_optical_frame",
        "camera_optical_frame",
        "realsense_link"
    };

    tf2_ros::Buffer tf_buffer(node_->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer, node_, false);

    for (const auto& camera_frame : camera_frames) {
        if (!tf_buffer.canTransform(camera_frame, config_.world_frame, tf2::TimePointZero, tf2::durationFromSec(0.5))) {
            continue;
        }

        geometry_msgs::msg::Vector3Stamped offset_in_arm;
        offset_in_arm.header.frame_id = config_.world_frame;
        offset_in_arm.vector = goal_.pick_offset;

        geometry_msgs::msg::Vector3Stamped offset_in_camera;
        const auto transform = tf_buffer.lookupTransform(camera_frame, config_.world_frame, tf2::TimePointZero);
        tf2::doTransform(offset_in_arm, offset_in_camera, transform);

        translated_offset = offset_in_camera.vector;
        break;
    }

    object_pose.position.x += translated_offset.x;
    object_pose.position.y += translated_offset.y;
    object_pose.position.z += translated_offset.z;

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
