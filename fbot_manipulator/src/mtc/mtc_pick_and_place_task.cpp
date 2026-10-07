#include "fbot_manipulator/mtc/mtc_pick_and_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace fbot_manipulator
{

MtcPickAndPlaceTask::MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                                         const ManipulationGoal& goal)
    : MtcTask("pick_and_place", node),
      goal_(goal)
{
}

bool MtcPickAndPlaceTask::buildTask()
{
    std::string target_id = goal_.target_id;
    if (target_id.empty() && !goal_.target_ids.empty()) {
        target_id = goal_.target_ids.front();
    }
    if (target_id.empty()) {
        RCLCPP_ERROR(logger(), "FAIL: no target_id provided for pick_and_place");
        return false;
    }

    task_.stages()->setName("pick_and_place_" + target_id);
    task_.loadRobotModel(node_);

    // Names OR poses OR explicit joints, never combined. A single-element list means this place target.
    PlaceTargets targets;
    if (!MtcSharedLogic::resolvePlaceTargets(
            task_, config_, goal_.place_pose_names, goal_.place_poses,
            goal_.place_joint_targets, goal_.place_pose, logger(), targets)) {
        return false;
    }
    if (targets.count() != 1) {
        RCLCPP_ERROR(logger(), "FAIL: pick_and_place accepts exactly 1 destination, received %zu", targets.count());
        return false;
    }

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene, goal_.pick_offset, target_id);

    // 1. Current state
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    if (object_poses_.find(target_id) == object_poses_.end()) {
        RCLCPP_ERROR(logger(), "ERROR: target_id '%s' was not found!", target_id.c_str());
        return false;
    }

    // The offset is defined relative to the arm/base frame, but the effective pose used by MTC
    // must be adjusted in the camera frame. Therefore, we convert the offset into the camera frame
    // before applying it to the detected object pose.
    geometry_msgs::msg::Pose object_pose = object_poses_[target_id];

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

    // 2. PICK
    mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
        task_, target_id, object_pose, current_state,
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger());
    if (!attach_stage) {
        RCLCPP_ERROR(logger(), "FAIL: addPickStages failed for '%s'", target_id.c_str());
        return false;
    }

    // 3. PLACE
    geometry_msgs::msg::Pose place_pose;
    place_pose.orientation.w = 1.0;
    const std::vector<double>* place_joint_target = nullptr;
    if (targets.named()) {
        place_joint_target = &targets.joint_targets.front();
    } else {
        place_pose = targets.poses.front();
    }

    mtc::Stage* place_ik_stage = MtcSharedLogic::addPlaceStages(
        task_, target_id, place_pose, attach_stage,
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger(),
        place_joint_target);
    if (!place_ik_stage) {
        RCLCPP_ERROR(logger(), "FAIL: addPlaceStages failed for '%s'", target_id.c_str());
        return false;
    }

    // 4. Return home final
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_ready_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator