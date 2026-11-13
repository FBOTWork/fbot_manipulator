#include "fbot_manipulator/mtc/mtc_unload_cargo_shake_task.hpp"
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"
#include "fbot_manipulator/mtc/mtc_task.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace fbot_manipulator
{

namespace
{

// Same as MtcSharedLogic::addPlaceStages, but the gripper shake happens BEFORE release/detach/retreat
// instead of after: the object is still attached and the gripper still closed while it shakes side to
// side, and only opens once the arm is back at the exact place pose (center). Inserting that mid-
// sequence isn't possible through addPlaceStages' public interface (it returns after building its own
// self-contained container), so this duplicates its cartesian-place body with the shake spliced in --
// kept local to this task on purpose, instead of changing the shared function every other place/pick_
// and_place/unload_cargo task goes through.
mtc::Stage* addPlaceStagesWithPreReleaseShake(
    mtc::Task& task,
    const std::string& target_id,
    const geometry_msgs::msg::Pose& place_pose,
    mtc::Stage* attach_stage,
    const MtcConfig& config,
    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
    rclcpp::Logger logger,
    bool approach_from_front)
{
    const bool from_front = approach_from_front;

    // Cartesian place: connect to the pose generator inside the container.
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to place [" + target_id + "]",
            mtc::stages::Connect::GroupPlannerVector{ { config.arm_group_name, pipeline_planner } });
        stage->setTimeout(1.5);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }

    mtc::Stage* place_ik_stage = nullptr;

    auto container = std::make_unique<mtc::SerialContainer>("place object [" + target_id + "]");
    task.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
    container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Lower
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "lower object [" + target_id + "]", cartesian_planner);
        stage->properties().set("marker_ns", "lower");
        stage->properties().set("link", config.hand_frame);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(config.place_lower_min, config.place_lower_max);

        geometry_msgs::msg::Vector3Stamped vec;
        if (from_front) {
            vec.header.frame_id = config.hand_frame;
            vec.vector.x = 1.0;
        } else {
            vec.header.frame_id = config.world_frame;
            vec.vector.z = -1.0;
        }
        stage->setDirection(vec);
        container->insert(std::move(stage));
    }

    // Allow object-table collisions
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "allow object-table collisions [" + target_id + "]");
        stage->allowCollisions(target_id, "workspace_table", true);
        if (!config.surface_link.empty() && config.surface_link != "workspace_table") {
            stage->allowCollisions(target_id, config.surface_link, true);
        }
        container->insert(std::move(stage));
    }

    // Generate Place Pose + IK
    {
        const std::size_t arm_dof =
            task.getRobotModel()->getJointModelGroup(config.arm_group_name)->getActiveJointModels().size();
        const bool waist_aligned = arm_dof < 6;

        std::unique_ptr<mtc::Stage> generator;
        if (waist_aligned) {
            tf2::Quaternion q_place;
            if (from_front) {
                const double yaw_to_place = std::atan2(place_pose.position.y, place_pose.position.x);
                q_place.setRPY(M_PI, 0.0, yaw_to_place);
            } else {
                tf2::Quaternion q_in(
                    place_pose.orientation.x, place_pose.orientation.y,
                    place_pose.orientation.z, place_pose.orientation.w);
                if (q_in.length2() < 1e-12) {
                    q_in = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
                } else {
                    q_in.normalize();
                }

                double roll, pitch, yaw;
                tf2::Matrix3x3(q_in).getRPY(roll, pitch, yaw);

                if (std::abs(roll) > 0.05 || std::abs(pitch) > 0.05) {
                    RCLCPP_WARN(logger,
                        "place [%s]: roll/pitch (%.2f, %.2f) ignorados, place top-down em 5DOF",
                        target_id.c_str(), roll, pitch);
                }

                double place_yaw = std::fmod(yaw, M_PI_2);
                if (place_yaw < 0.0) place_yaw += M_PI_2;

                q_place.setRPY(0.0, M_PI_2, place_yaw);
            }
            q_place.normalize();

            geometry_msgs::msg::PoseStamped target;
            target.header.frame_id = config.world_frame;
            target.pose.position = place_pose.position;
            target.pose.orientation.x = q_place.x();
            target.pose.orientation.y = q_place.y();
            target.pose.orientation.z = q_place.z();
            target.pose.orientation.w = q_place.w();

            auto stage = std::make_unique<mtc::stages::GeneratePose>(
                "generate place pose [" + target_id + "]");
            stage->properties().set("marker_ns", "place_pose");
            stage->setPose(target);
            stage->setMonitoredStage(attach_stage);
            generator = std::move(stage);
        } else {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>(
                "generate place pose [" + target_id + "]");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject(target_id);

            geometry_msgs::msg::PoseStamped target;
            target.header.frame_id = config.world_frame;
            target.pose = place_pose;
            stage->setPose(target);
            stage->setMonitoredStage(attach_stage);
            generator = std::move(stage);
        }

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
            "place pose IK [" + target_id + "]", std::move(generator));
        wrapper->setMaxIKSolutions(waist_aligned ? 8 : 4);
        wrapper->setMinSolutionDistance(0.1);
        wrapper->setIKFrame(config.grasp_frame_transform, config.hand_frame);
        wrapper->setTimeout(1.5);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

        place_ik_stage = wrapper.get();
        container->insert(std::move(wrapper));
    }

    // SHAKE: object still attached, gripper still closed. Left, then right (crossing past center),
    // then back to center -- exact distances (min==max) so the three moves cancel out and the arm is
    // guaranteed back at the place pose before release opens the gripper. No tolerance here on purpose:
    // releasing off-center would miss the intended place point, which matters more than the shake
    // succeeding -- if the real surroundings don't leave room for 2*shake_distance sideways, this
    // should fail loudly rather than release somewhere else.
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "shake gripper left [" + target_id + "]", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(config.shake_distance, config.shake_distance);
        stage->setIKFrame(config.grasp_frame_transform, config.hand_frame);
        stage->properties().set("marker_ns", "shake");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = config.hand_frame;
        vec.vector.y = 1.0;
        stage->setDirection(vec);
        container->insert(std::move(stage));
    }
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "shake gripper right [" + target_id + "]", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(2.0 * config.shake_distance, 2.0 * config.shake_distance);
        stage->setIKFrame(config.grasp_frame_transform, config.hand_frame);
        stage->properties().set("marker_ns", "shake");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = config.hand_frame;
        vec.vector.y = -1.0;
        stage->setDirection(vec);
        container->insert(std::move(stage));
    }
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "shake gripper back to center [" + target_id + "]", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(config.shake_distance, config.shake_distance);
        stage->setIKFrame(config.grasp_frame_transform, config.hand_frame);
        stage->properties().set("marker_ns", "shake");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = config.hand_frame;
        vec.vector.y = 1.0;
        stage->setDirection(vec);
        container->insert(std::move(stage));
    }

    // Release
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>(
            "release object [" + target_id + "]", joint_planner);
        stage->setGroup(config.hand_group_name);
        stage->setGoal(config.hand_open_state);
        container->insert(std::move(stage));
    }

    // Detach
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "detach object [" + target_id + "]");
        stage->detachObject(target_id, config.hand_frame);
        container->insert(std::move(stage));
    }

    // Retreat
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "retreat [" + target_id + "]", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(config.retreat_min, config.retreat_max);
        stage->setIKFrame(config.grasp_frame_transform, config.hand_frame);
        stage->properties().set("marker_ns", "retreat");

        geometry_msgs::msg::Vector3Stamped vec;
        if (from_front) {
            vec.header.frame_id = config.hand_frame;
            vec.vector.x = -1.0;
        } else {
            vec.header.frame_id = config.world_frame;
            vec.vector.z = 1.0;
        }
        stage->setDirection(vec);
        container->insert(std::move(stage));
    }

    task.add(std::move(container));
    return place_ik_stage;
}

} // namespace

MtcUnloadCargoShakeTask::MtcUnloadCargoShakeTask(
    rclcpp::Node::SharedPtr node,
    const ManipulationGoal& goal,
    bool approach_from_front)
    : MtcTask("unload_cargo_shake", node),
      goal_(goal),
      approach_from_front_(approach_from_front)
{
}

bool MtcUnloadCargoShakeTask::buildTask()
{
    std::vector<std::string> target_ids = goal_.target_ids;
    std::vector<int> cargo_indices = goal_.cargo_indices;

    if (target_ids.empty() && !goal_.target_id.empty()) {
        target_ids.push_back(goal_.target_id);
    }
    if (cargo_indices.empty() && goal_.cargo_id >= 0) {
        cargo_indices.push_back(goal_.cargo_id);
    }

    stage_checkpoints_.clear();

    task_.stages()->setName("unload_cargo_shake_" + std::to_string(target_ids.size()));
    task_.loadRobotModel(node_);

    // Names OR poses OR explicit joints, never combined. A single-element list means this place target.
    // Named/joint-space places aren't supported here (the shake assumes a cartesian place pose to
    // return to): only poses are used below.
    PlaceTargets targets;
    if (!MtcSharedLogic::resolvePlaceTargets(
            task_, config_, goal_.place_pose_names, goal_.place_poses,
            goal_.place_joint_targets, goal_.place_pose, logger(), targets)) {
        return false;
    }

    if (target_ids.empty() || cargo_indices.empty() || targets.count() == 0) {
        RCLCPP_ERROR(logger(), "FAIL: no target_ids, cargo_indices, or place targets provided for unload_cargo_shake");
        return false;
    }
    if (targets.named()) {
        RCLCPP_ERROR(logger(), "FAIL: unload_cargo_shake only supports cartesian place_poses, not place_pose_names");
        return false;
    }
    if (target_ids.size() != cargo_indices.size() || target_ids.size() != targets.count()) {
        RCLCPP_ERROR(logger(), "FAIL: vectors size mismatch (targets: %zu, cargos: %zu, place targets: %zu)",
                     target_ids.size(), cargo_indices.size(), targets.count());
        return false;
    }

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene, goal_.pick_offset, target_ids);

    ::geometry_msgs::msg::Vector3 tag_size;
    tag_size.x = 0.04;
    tag_size.y = 0.04;
    tag_size.z = 0.04;

    // 1. Current State
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    for (size_t i = 0; i < target_ids.size(); ++i) {
        const std::string& target_id = target_ids[i];
        const int cargo_id = cargo_indices[i];
        const geometry_msgs::msg::Pose& place_pose = targets.poses[i];

        geometry_msgs::msg::Pose pick_pose;
        try {
            pick_pose = cargoSlotPose(cargo_id, config_.cargo_unload_z);
        } catch (const std::out_of_range& e) {
            RCLCPP_ERROR(logger(), "FAIL: %s", e.what());
            build_failed_target_id_ = target_id;
            return false;
        }

        MtcTask::addCollisionObject(target_id, pick_pose, tag_size);

        // 2. PICK (from the cargo slot: always top-down)
        mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
            task_, target_id, pick_pose, current_state,
            config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger());
        if (!attach_stage) {
            RCLCPP_ERROR(logger(), "FAIL: addPickStages failed for '%s'", target_id.c_str());
            build_failed_target_id_ = target_id;
            return false;
        }
        stage_checkpoints_.emplace_back(target_id, attach_stage);

        // 3. PLACE + SHAKE (shake happens before release, while still holding the object; see
        // addPlaceStagesWithPreReleaseShake above)
        mtc::Stage* place_ik_stage = addPlaceStagesWithPreReleaseShake(
            task_, target_id, place_pose, attach_stage,
            config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger(),
            approach_from_front_);
        if (!place_ik_stage) {
            RCLCPP_ERROR(logger(), "FAIL: addPlaceStagesWithPreReleaseShake failed for '%s'", target_id.c_str());
            build_failed_target_id_ = target_id;
            return false;
        }
        stage_checkpoints_.emplace_back(target_id, place_ik_stage);

        current_state = place_ik_stage;
    }

    // Return Home
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_ready_state);
        task_.add(std::move(stage));
    }

    return true;
}

std::string MtcUnloadCargoShakeTask::firstFailedTargetId() const
{
    for (const auto& checkpoint : stage_checkpoints_) {
        const std::string& target_id = checkpoint.first;
        const mtc::Stage* stage = checkpoint.second;

        if (stage == nullptr || stage->solutions().empty()) {
            return target_id;
        }
    }
    return {};
}

} // namespace fbot_manipulator
