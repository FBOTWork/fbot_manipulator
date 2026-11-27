#include "fbot_manipulator/mtc/mtc_shake_shared_logic.hpp"
#include <cmath>
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
        vec.vector.z = 1.0;
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
        vec.vector.z = -1.0;
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
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        container->insert(std::move(stage));
    }
    
    

    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "shake gripper up [" + target_id + "]", cartesian_planner);
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
            "shake gripper down [" + target_id + "]", cartesian_planner);
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



    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "shake gripper left [" + target_id + "]", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(config.shake_distance, config.shake_distance);
        stage->setIKFrame(config.grasp_frame_transform, config.hand_frame);
        stage->properties().set("marker_ns", "shake");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = config.hand_frame;
        vec.vector.z = 1.0;
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
        vec.vector.z = -1.0;
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
        vec.vector.z = 1.0;
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
