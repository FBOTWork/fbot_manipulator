#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"
#include "fbot_manipulator/mtc/mtc_task.hpp"
#include "fbot_manipulator/utils.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <yaml-cpp/yaml.h>

namespace fbot_manipulator
{

void MtcSharedLogic::setupWorkspace(MtcTask* task_instance,
                                    const std::vector<ObjectDetection>& objects_scene,
                                    geometry_msgs::msg::Vector3& pick_offset,
                                    const std::string& target_id)
{
    setupWorkspace(task_instance, objects_scene, pick_offset, std::vector<std::string>{target_id});
}

void MtcSharedLogic::setupWorkspace(MtcTask* task_instance,
                                    const std::vector<ObjectDetection>& objects_scene,
                                    geometry_msgs::msg::Vector3& pick_offset,
                                    const std::vector<std::string>& target_ids)
{
    geometry_msgs::msg::Vector3 workspace_size;
    workspace_size.x = 0.30;
    workspace_size.y = 0.30;
    workspace_size.z = 0.05;

    geometry_msgs::msg::Pose workspace_pose;
    workspace_pose.orientation.w = 1.0;
    workspace_pose.position.x = -0.1;
    workspace_pose.position.y = 0.0;
    workspace_pose.position.z = -0.026;

    task_instance->addCollisionObject("workspace_table", workspace_pose, workspace_size);
    task_instance->setCollisionObjectColor("workspace_table", 0.5, 0.5, 0.5, 1.0);

    geometry_msgs::msg::Vector3 dorso_size;
    dorso_size.x = 0.05;
    dorso_size.y = 0.30;
    dorso_size.z = 0.40;

    geometry_msgs::msg::Pose dorso_pose;
    dorso_pose.orientation.w = 1.0;
    dorso_pose.position.x = -0.275;
    dorso_pose.position.y = 0.0;
    dorso_pose.position.z = 0.20;

    task_instance->addCollisionObject("robot_spine", dorso_pose, dorso_size);
    task_instance->setCollisionObjectColor("robot_spine", 0.35, 0.35, 0.35, 1.0);

    for (const auto& obj : objects_scene) {

        geometry_msgs::msg::Pose collisor_pose;
        const bool is_target = std::find(target_ids.begin(), target_ids.end(), obj.id) != target_ids.end();

        if (is_target){
            collisor_pose.position.x = obj.pose.position.x + pick_offset.x;
            collisor_pose.position.y = obj.pose.position.y + pick_offset.y;
            collisor_pose.position.z = obj.pose.position.z + pick_offset.z;
        } else{
            collisor_pose.position.x = obj.pose.position.x;
            collisor_pose.position.y = obj.pose.position.y;
            collisor_pose.position.z = obj.pose.position.z;
        }
        collisor_pose.orientation.x = obj.pose.orientation.x;
        collisor_pose.orientation.y = obj.pose.orientation.y;
        collisor_pose.orientation.z = obj.pose.orientation.z;
        collisor_pose.orientation.w = obj.pose.orientation.w;

        task_instance->addCollisionObject(obj.id, collisor_pose, obj.size);
        task_instance->setCollisionObjectColor(obj.id, 0.0, 1.0, 0.0, 1.0);
    }
}

bool MtcSharedLogic::resolvePlaceTargets(
    mtc::Task& task,
    const MtcConfig& config,
    const std::vector<std::string>& names,
    const std::vector<geometry_msgs::msg::Pose>& poses,
    const std::vector<std::vector<double>>& joint_targets,
    const geometry_msgs::msg::Pose& legacy_single_pose,
    rclcpp::Logger logger,
    PlaceTargets& out)
{
    out = PlaceTargets{};

    const int modes = (!names.empty() ? 1 : 0) + (!poses.empty() ? 1 : 0) + (!joint_targets.empty() ? 1 : 0);
    if (modes > 1) {
        RCLCPP_ERROR(logger,
            "FAIL: provide only place_pose_names OR place_poses OR place_joint_targets, never combined");
        return false;
    }

    if (names.empty()) {
        out.joint_targets = joint_targets;
        out.poses = poses;
        if (modes == 0) {
            out.poses.push_back(legacy_single_pose);  // legacy single-place compatibility
        }
        return true;
    }

    RCLCPP_INFO(logger, "Received %zu place pose names", names.size());

    const auto* group = task.getRobotModel()->getJointModelGroup(config.arm_group_name);
    if (!group) {
        RCLCPP_ERROR(logger, "FAIL: group '%s' does not exist in the model", config.arm_group_name.c_str());
        return false;
    }
    const std::size_t expected = group->getActiveJointModels().size();

    try {
        const std::string path = ament_index_cpp::get_package_share_directory("fbot_manipulator") +
                                 "/config/" + config.arm_group_name + "/manipulator_config.yaml";
        const YAML::Node root = Utils::readYaml(path);
        const YAML::Node table = root["poses"];
        if (!table || !table.IsMap()) {
            RCLCPP_ERROR(logger, "Poses map missing in %s", path.c_str());
            return false;
        }

        std::string available;
        for (const auto& kv : table) {
            available += (available.empty() ? "" : ", ") + kv.first.as<std::string>();
        }

        for (const auto& name : names) {
            const YAML::Node node = table[name];
            if (!node.IsDefined() || !node.IsSequence()) {
                RCLCPP_ERROR(logger, "Pose '%s' does not exist in 'poses' (available: %s)",
                             name.c_str(), available.c_str());
                return false;
            }
            std::vector<double> joints;
            joints.reserve(node.size());
            for (const auto& v : node) joints.push_back(v.as<double>());
            if (joints.size() != expected) {
                RCLCPP_ERROR(logger, "Pose '%s' has %zu joints, group '%s' expects %zu",
                             name.c_str(), joints.size(), config.arm_group_name.c_str(), expected);
                return false;
            }
            out.joint_targets.push_back(std::move(joints));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Failed to read 'poses' from YAML: %s", e.what());
        return false;
    }
    return true;
}

mtc::Stage* MtcSharedLogic::addPickStages(
    mtc::Task& task,
    const std::string& target_id,
    const geometry_msgs::msg::Pose& object_pose,
    mtc::Stage* current_state,
    const MtcConfig& config,
    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
    rclcpp::Logger /*logger*/)
{
    // ---- Open gripper ----
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper [" + target_id + "]", joint_planner);
        stage->setGroup(config.hand_group_name);
        stage->setGoal(config.hand_open_state);
        task.add(std::move(stage));
    }

    const std::size_t arm_dof =
        task.getRobotModel()->getJointModelGroup(config.arm_group_name)->getActiveJointModels().size();
    const bool waist_aligned = arm_dof < 6;

    mtc::Stage* grasp_monitor = current_state;
    if (waist_aligned) {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "allow object-robot collisions [" + target_id + "]");
        stage->allowCollisions(target_id, task.getRobotModel()->getLinkModelNames(), true);
        grasp_monitor = stage.get();
        task.add(std::move(stage));
    }

    // ---- Move to pick (connect) ----
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to pick [" + target_id + "]",
            mtc::stages::Connect::GroupPlannerVector{
                { config.arm_group_name, pipeline_planner }
            });
        stage->setTimeout(1.5);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }

    mtc::Stage* attach_object_stage = nullptr;

    // ---- Pick Object Container ----
    {
        auto container = std::make_unique<mtc::SerialContainer>("pick object [" + target_id + "]");
        task.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
        container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // Approach
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>(
                "approach object [" + target_id + "]", cartesian_planner);
            stage->properties().set("marker_ns", "approach");
            stage->properties().set("link", config.hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(config.approach_min, config.approach_max);

            geometry_msgs::msg::Vector3Stamped vec;
            if (waist_aligned) {
                vec.header.frame_id = config.world_frame;
                vec.vector.x = 0.0;
                vec.vector.y = 0.0;
                vec.vector.z = -1.0; // Moves downward vertically (top-down)
            }
            stage->setDirection(vec);
            container->insert(std::move(stage));
        }

        // Generate grasp pose + IK
        {
            std::unique_ptr<mtc::Stage> generator;
            if (waist_aligned) {
                geometry_msgs::msg::PoseStamped target;
                target.header.frame_id = config.world_frame;
                target.pose.position = object_pose.position;

                tf2::Quaternion q_obj(
                    object_pose.orientation.x,
                    object_pose.orientation.y,
                    object_pose.orientation.z,
                    object_pose.orientation.w
                );

                double obj_roll, obj_pitch, obj_yaw;
                tf2::Matrix3x3(q_obj).getRPY(obj_roll, obj_pitch, obj_yaw);

                while (obj_yaw > M_PI) obj_yaw -= 2.0 * M_PI;
                while (obj_yaw <= -M_PI) obj_yaw += 2.0 * M_PI;

                // The cube has four-sided symmetry around Z, so we reduce the yaw to the
                // equivalent within [0, 90°] while preserving the useful orientation for grasping.
                double grasp_yaw = std::fmod(obj_yaw, M_PI_2);
                if (grasp_yaw < 0.0) grasp_yaw += M_PI_2;

                tf2::Quaternion q_grasp;
                q_grasp.setRPY(0.0, M_PI_2, grasp_yaw);

                target.pose.orientation.x = q_grasp.x();
                target.pose.orientation.y = q_grasp.y();
                target.pose.orientation.z = q_grasp.z();
                target.pose.orientation.w = q_grasp.w();

                auto stage = std::make_unique<mtc::stages::GeneratePose>(
                    "generate grasp pose [" + target_id + "]");
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPose(target);
                stage->setMonitoredStage(grasp_monitor);
                generator = std::move(stage);
            } else {
                auto stage = std::make_unique<mtc::stages::GenerateGraspPose>(
                    "generate grasp pose [" + target_id + "]");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose(config.hand_open_state);
                stage->setObject(target_id);
                stage->setAngleDelta(config.grasp_angle_delta);
                stage->setMonitoredStage(grasp_monitor);
                generator = std::move(stage);
            }

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
                "grasp pose IK [" + target_id + "]", std::move(generator));
            wrapper->setMaxIKSolutions(waist_aligned ? 8 : 4);
            wrapper->setMinSolutionDistance(0.1);
            wrapper->setIKFrame(config.grasp_frame_transform, config.hand_frame);
            wrapper->setTimeout(1.5);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            container->insert(std::move(wrapper));
        }

        // Allow collision (hand, object)
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                "allow collision (hand,object) [" + target_id + "]");
            stage->allowCollisions(config.hand_frame, target_id, true);
            container->insert(std::move(stage));
        }

        // Close gripper
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>(
                "close gripper [" + target_id + "]", joint_planner);
            stage->setGroup(config.hand_group_name);
            stage->setGoal(config.hand_closed_state);
            container->insert(std::move(stage));
        }

        // Attach object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                "attach object [" + target_id + "]");
            stage->attachObject(target_id, config.hand_frame);
            attach_object_stage = stage.get();
            container->insert(std::move(stage));
        }

        // Allow object-surface collision
        if (!config.surface_link.empty()) {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                "allow collision (object,surface) [" + target_id + "]");
            stage->allowCollisions(target_id, config.surface_link, true);
            container->insert(std::move(stage));
        }

        // Lift
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>(
                "lift object [" + target_id + "]", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(config.lift_min, config.lift_max);
            stage->setIKFrame(config.grasp_frame_transform, config.hand_frame);
            stage->properties().set("marker_ns", "lift");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = config.world_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            container->insert(std::move(stage));
        }

        // Forbid object-surface collision
        if (!config.surface_link.empty()) {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                "forbid collision (object,surface) [" + target_id + "]");
            stage->allowCollisions(target_id, config.surface_link, false);
            container->insert(std::move(stage));
        }

        task.add(std::move(container));
    }

    return attach_object_stage;
}

mtc::Stage* MtcSharedLogic::addPlaceStages(
    mtc::Task& task,
    const std::string& target_id,
    const geometry_msgs::msg::Pose& place_pose,
    mtc::Stage* attach_stage,
    const MtcConfig& config,
    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
    rclcpp::Logger logger,
    const std::vector<double>* place_joint_target)
{
    const bool use_joint_target = place_joint_target && !place_joint_target->empty();
    std::map<std::string, double> joint_goal;

    if (use_joint_target) {
        const auto* joint_group = task.getRobotModel()->getJointModelGroup(config.arm_group_name);
        if (!joint_group) {
            RCLCPP_ERROR(logger, "Group '%s' does not exist", config.arm_group_name.c_str());
            return nullptr;
        }
        const auto& joint_names = joint_group->getActiveJointModelNames();
        if (place_joint_target->size() != joint_names.size()) {
            RCLCPP_ERROR(logger, "place joint target has %zu values, group has %zu joints",
                         place_joint_target->size(), joint_names.size());
            return nullptr;
        }
        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            joint_goal[joint_names[i]] = (*place_joint_target)[i];
        }
    } else {
        // Cartesian place: connect to the pose generator inside the container.
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

    auto make_allow_table = [&]() {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "allow object-table collisions [" + target_id + "]");
        stage->allowCollisions(target_id, "workspace_table", true);
        if (!config.surface_link.empty() && config.surface_link != "workspace_table") {
            stage->allowCollisions(target_id, config.surface_link, true);
        }
        return stage;
    };

    if (use_joint_target) {
        container->insert(make_allow_table());

        auto stage = std::make_unique<mtc::stages::MoveTo>(
            "move to place joint target [" + target_id + "]", pipeline_planner);
        stage->setGroup(config.arm_group_name);
        stage->setGoal(joint_goal);
        place_ik_stage = stage.get();
        container->insert(std::move(stage));
    } else {
        // Lower
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>(
                "lower object [" + target_id + "]", cartesian_planner);
            stage->properties().set("marker_ns", "lower");
            stage->properties().set("link", config.hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(config.place_lower_min, config.place_lower_max);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = config.world_frame;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            container->insert(std::move(stage));
        }

        container->insert(make_allow_table());

        // Generate Place Pose + IK
        {
            const std::size_t arm_dof =
                task.getRobotModel()->getJointModelGroup(config.arm_group_name)->getActiveJointModels().size();
            const bool waist_aligned = arm_dof < 6;

            std::unique_ptr<mtc::Stage> generator;
            if (waist_aligned) {
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

                // Cubo: simetria de 4 lados em Z -> yaw equivalente módulo 90°, em [0, π/2).
                double place_yaw = std::fmod(yaw, M_PI_2);
                if (place_yaw < 0.0) place_yaw += M_PI_2;

                // Mesma convenção do pick: RPY(0, π/2, yaw).
                tf2::Quaternion q_place;
                q_place.setRPY(0.0, M_PI_2, place_yaw);
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