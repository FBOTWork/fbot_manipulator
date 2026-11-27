#include "fbot_manipulator/mtc/mtc_shake_shared_logic.hpp"
#include <cmath>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace fbot_manipulator
{

mtc::Stage* MtcShakeSharedLogic::addPlaceStagesWithPreReleaseShake(
    mtc::Task& task,
    const std::string& target_id,
    const geometry_msgs::msg::Pose& place_pose,
    mtc::Stage* attach_stage,
    const MtcConfig& config,
    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
    rclcpp::Logger logger,
    bool approach_from_front,
    GraspYaw grasp_yaw_mode)
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

    // Gripper yaw at the place (top-down 5-DOF only; 0 otherwise). The post-turn XY offset below
    // is rotated by it, so it follows the gripper like it would for a ROTATED grasp yaw.
    double gripper_yaw = 0.0;

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

                // Same band selection as the pick's grasp_yaw (mtc_shared_logic.cpp): FREE keeps
                // whichever of [0,90) the place target's own yaw gives; STRAIGHT/ROTATED force it
                // into [-45,45) / [45,135) respectively.
                if (grasp_yaw_mode == GraspYaw::STRAIGHT && place_yaw >= M_PI_4) {
                    place_yaw -= M_PI_2;
                } else if (grasp_yaw_mode == GraspYaw::ROTATED && place_yaw < M_PI_4) {
                    place_yaw += M_PI_2;
                }

                gripper_yaw = place_yaw;
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

    // SEAT: object still (loosely) held, helping it settle fully into a tight-fitting slot before
    // the real release. Opens to an intermediate position first (not config.hand_open_state) so the
    // object can still slide under the shakes instead of being clamped rigid; shakes along the hand
    // frame's Y, lifts off, turns the wrist 90 degrees in place, lowers back down, then shakes again
    // along the hand frame's Z -- the wrist turn plus the different local axis means this second
    // shake sweeps a different world-space direction than the first. The lift/lower pair uses exact
    // (min==max) equal distances so they cancel out and
    // the arm is back at the pre-turn height before the second shake, same rationale as the shake
    // moves below: this is about seating the object, not about where the gripper ends up, so any
    // drift here should fail planning rather than quietly land off the intended place pose.
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>(
            "open gripper to intermediate [" + target_id + "]", joint_planner);
        stage->setGroup(config.hand_group_name);
        stage->properties().set("marker_ns", "shake");

        // Interpolate per joint between the two named SRDF states instead of a hardcoded absolute
        // value: hand groups differ per robot (xarm6's single drive_joint goes 0 (open) -> 0.85
        // (closed), while the Interbotix gripper's mirrored left_finger/right_finger pair goes
        // +0.037/-0.037 (open) -> +0.015/-0.015 (closed) -- a single scalar applied to every joint
        // would be out of range and wrong-signed on the latter.
        const auto* hand_jmg = task.getRobotModel()->getJointModelGroup(config.hand_group_name);
        moveit::core::RobotState ref_state(task.getRobotModel());
        std::vector<double> open_values;
        std::vector<double> closed_values;
        if (!ref_state.setToDefaultValues(hand_jmg, config.hand_open_state)) {
            RCLCPP_ERROR(logger, "FAIL: unknown hand_open_state '%s' for group '%s'",
                         config.hand_open_state.c_str(), config.hand_group_name.c_str());
            return nullptr;
        }
        ref_state.copyJointGroupPositions(hand_jmg, open_values);
        if (!ref_state.setToDefaultValues(hand_jmg, config.hand_closed_state)) {
            RCLCPP_ERROR(logger, "FAIL: unknown hand_closed_state '%s' for group '%s'",
                         config.hand_closed_state.c_str(), config.hand_group_name.c_str());
            return nullptr;
        }
        ref_state.copyJointGroupPositions(hand_jmg, closed_values);

        const auto& hand_joint_names = hand_jmg->getActiveJointModelNames();
        std::map<std::string, double> hand_intermediate_goal;
        for (std::size_t i = 0; i < hand_joint_names.size(); ++i) {
            const double t = config.shake_intermediate_open_ratio;
            hand_intermediate_goal[hand_joint_names[i]] =
                open_values[i] + t * (closed_values[i] - open_values[i]);
        }
        stage->setGoal(hand_intermediate_goal);
        container->insert(std::move(stage));
    }

    // The lateral shake moves get their own solver so they can run at a different speed than the
    // rest of the task (config.shake_velocity_scaling, mtc.shake_velocity_scaling in the yaml).
    auto shake_planner = std::make_shared<mtc::solvers::CartesianPath>();
    shake_planner->setMaxVelocityScalingFactor(config.shake_velocity_scaling);
    shake_planner->setMaxAccelerationScalingFactor(config.shake_acceleration_scaling);
    shake_planner->setStepSize(0.002);

    // Shake #1: left, then right (crossing past center), then back to center -- exact distances
    // (min==max) so the three moves cancel out and the arm is at the same pose before the turn.
    // Hand frame Y here, Z in shake #2 below -- two different local axes (on top of the physical
    // wrist turn between them) so the two shakes don't retrace the same line in world space.
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "shake gripper left [" + target_id + "]", shake_planner);
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
            "shake gripper right [" + target_id + "]", shake_planner);
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
            "shake gripper back to center [" + target_id + "]", shake_planner);
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

    // Up, out of the slot, before rotating the wrist in place.
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "shake sequence lift [" + target_id + "]", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(config.shake_turn_lift_distance, config.shake_turn_lift_distance);
        stage->setIKFrame(config.grasp_frame_transform, config.hand_frame);
        stage->properties().set("marker_ns", "shake");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = config.world_frame;
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        container->insert(std::move(stage));
    }

    // Turn 90 degrees: a relative move on the arm's last (wrist) joint only, so the object rotates
    // about the approach axis roughly in place instead of tracing a cartesian arc.
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "shake sequence turn 90 [" + target_id + "]", joint_planner);
        stage->setGroup(config.arm_group_name);
        stage->properties().set("marker_ns", "shake");

        const auto& arm_joint_names =
            task.getRobotModel()->getJointModelGroup(config.arm_group_name)->getActiveJointModelNames();
        std::map<std::string, double> wrist_turn_delta;
        wrist_turn_delta[arm_joint_names.back()] = config.shake_turn_angle;
        stage->setDirection(wrist_turn_delta);
        container->insert(std::move(stage));
    }

    // Horizontal offset in the world XY plane while still lifted, after the turn
    // (config.shake_turn_offset_x / _y, both 0 = skip).
    const double offset_xy = std::hypot(config.shake_turn_offset_x, config.shake_turn_offset_y);
    if (offset_xy > 1e-6) {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "shake sequence offset XY [" + target_id + "]", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(offset_xy, offset_xy);
        stage->setIKFrame(config.grasp_frame_transform, config.hand_frame);
        stage->properties().set("marker_ns", "shake");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = config.world_frame;
        const double c = std::cos(gripper_yaw), s = std::sin(gripper_yaw);
        vec.vector.x = (c * config.shake_turn_offset_x - s * config.shake_turn_offset_y) / offset_xy;
        vec.vector.y = (s * config.shake_turn_offset_x + c * config.shake_turn_offset_y) / offset_xy;
        stage->setDirection(vec);
        container->insert(std::move(stage));
    }

    // Back down into the slot, now rotated 90 degrees.
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "shake sequence lower [" + target_id + "]", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        // Lowers by the lift distance plus config.shake_turn_offset_z (negative = deeper than the pre-turn height).
        const double lower_distance = config.shake_turn_lift_distance - config.shake_turn_offset_z;
        stage->setMinMaxDistance(lower_distance, lower_distance);
        stage->setIKFrame(config.grasp_frame_transform, config.hand_frame);
        stage->properties().set("marker_ns", "shake");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = config.world_frame;
        vec.vector.z = -1.0;
        stage->setDirection(vec);
        container->insert(std::move(stage));
    }

    // Shake #2: hand frame Z (shake #1 used Y) -- on top of the 90-degree wrist turn already
    // applied, this covers a third distinct world-space direction. Same left/right/back-to-center
    // pattern as shake #1.
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "shake gripper left [" + target_id + "]", shake_planner);
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
            "shake gripper right [" + target_id + "]", shake_planner);
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
            "shake gripper back to center [" + target_id + "]", shake_planner);
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

        // Always lift straight up in the world frame after a shake, regardless of from_front --
        // neither the gripper's own approach axis nor world Y reliably clear the target; world
        // +Z always does.
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = config.world_frame;
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        container->insert(std::move(stage));
    }

    // Remove
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "remove object [" + target_id + "]");
        stage->removeObject(target_id);
        container->insert(std::move(stage));
    }

    task.add(std::move(container));
    return place_ik_stage;
}

} // namespace fbot_manipulator
