#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"
#include "fbot_manipulator/mtc/mtc_task.hpp" 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace fbot_manipulator
{

void MtcSharedLogic::setupWorkspace(MtcTask* task_instance)
{
    geometry_msgs::msg::Vector3 workspace_size;
    workspace_size.x = 0.30; // 35 cm de comprimento
    workspace_size.y = 0.30; // 35 cm de largura
    workspace_size.z = 0.05; // 5 cm de espessura

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
    
    dorso_pose.position.y = 0.0; // Centralizado junto com a mesa
    
    dorso_pose.position.z = 0.20; 

    task_instance->addCollisionObject("robot_spine", dorso_pose, dorso_size);
    task_instance->setCollisionObjectColor("robot_spine", 0.35, 0.35, 0.35, 1.0);
}

mtc::Stage* MtcSharedLogic::addPickStages(
    mtc::Task& task,
    const std::string& object_id,
    const geometry_msgs::msg::Pose& object_pose,
    mtc::Stage* current_state,
    const MtcConfig& config,
    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
    rclcpp::Logger logger)
{
    // ---- Open Gripper ----
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_planner);
        stage->setGroup(config.hand_group_name);
        stage->setGoal(config.hand_open_state);
        task.add(std::move(stage));
    }

    const std::size_t arm_dof =
        task.getRobotModel()->getJointModelGroup(config.arm_group_name)->getActiveJointModels().size();
    const bool waist_aligned = arm_dof < 6;

    mtc::Stage* grasp_monitor = current_state;
    if (waist_aligned) {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow object-robot collisions");
        stage->allowCollisions(object_id, task.getRobotModel()->getLinkModelNames(), true);
        grasp_monitor = stage.get();
        task.add(std::move(stage));
    }

    // ---- Move to Pick (Connect) ----
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to pick",
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
        auto container = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
        container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // Approach
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach");
            stage->properties().set("link", config.hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(config.approach_min, config.approach_max);

            geometry_msgs::msg::Vector3Stamped vec;
            if (waist_aligned) {
                vec.header.frame_id = config.world_frame;
                vec.vector.x = 0.0;
                vec.vector.y = 0.0;
                vec.vector.z = -1.0; // Desce verticalmente (top-down)
            }
            stage->setDirection(vec);
            container->insert(std::move(stage));
        }

        // Generate Grasp Pose + IK
        {
            std::unique_ptr<mtc::Stage> generator;
            if (waist_aligned) {
                geometry_msgs::msg::PoseStamped target;
                target.header.frame_id = config.world_frame;
                target.pose.position = object_pose.position;

                float quat_w = object_pose.orientation.w;

                if (quat_w > 4.7124) {
                quat_w -= 4.7124;}
                else if (quat_w > 3.1416) {
                quat_w -= 3.1416;} 
                else if (quat_w > 1.5708) {
                quat_w -= 1.5708;}

                // Extrai a rotação real da peça na mesa
                tf2::Quaternion q_obj(
                    object_pose.orientation.x,
                    object_pose.orientation.y,
                    object_pose.orientation.z,
                    quat_w
                );
                
                double obj_roll, obj_pitch, obj_yaw;
                tf2::Matrix3x3(q_obj).getRPY(obj_roll, obj_pitch, obj_yaw);

                tf2::Quaternion q_grasp;
                // Alinha o Yaw da garra com o Yaw da peça e vira a garra para baixo
                q_grasp.setRPY(0.0, M_PI_2, obj_yaw);

                target.pose.orientation.x = q_grasp.x();
                target.pose.orientation.y = q_grasp.y();
                target.pose.orientation.z = q_grasp.z();
                target.pose.orientation.w = q_grasp.w();

                auto stage = std::make_unique<mtc::stages::GeneratePose>("generate grasp pose");
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPose(target);
                stage->setMonitoredStage(grasp_monitor);
                generator = std::move(stage);
            } else {
                auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose(config.hand_open_state);
                stage->setObject(object_id);
                stage->setAngleDelta(config.grasp_angle_delta);
                stage->setMonitoredStage(grasp_monitor);
                generator = std::move(stage);
            }

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(generator));
            wrapper->setMaxIKSolutions(waist_aligned ? 8 : 4);
            wrapper->setMinSolutionDistance(0.2);
            wrapper->setIKFrame(config.grasp_frame_transform, config.hand_frame);
            wrapper->setTimeout(5.0);
            wrapper->setIgnoreCollisions(!waist_aligned);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            container->insert(std::move(wrapper));
        }

        // Allow hand-object collision
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions(object_id,
                                   task.getRobotModel()
                                       ->getJointModelGroup(config.hand_group_name)
                                       ->getLinkModelNamesWithCollisionGeometry(),
                                   true);
            container->insert(std::move(stage));
        }

        // Close gripper
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_planner);
            stage->setGroup(config.hand_group_name);
            stage->setGoal(config.hand_closed_state);
            container->insert(std::move(stage));
        }

        // Attach object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject(object_id, config.hand_frame);
            attach_object_stage = stage.get(); // Salva o ponteiro para retornar
            container->insert(std::move(stage));
        }

        // Allow object-surface collision
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,surface)");
            stage->allowCollisions(object_id, config.surface_link, true);
            container->insert(std::move(stage));
        }

        // Lift
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
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
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
            stage->allowCollisions(object_id, config.surface_link, false);
            container->insert(std::move(stage));
        }

        task.add(std::move(container));
    }

    return attach_object_stage;
}

void MtcSharedLogic::addPlaceStages(
    mtc::Task& task,
    const std::string& object_id,
    const geometry_msgs::msg::Pose& place_pose,
    mtc::Stage* attach_stage,
    const MtcConfig& config,
    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
    rclcpp::Logger logger)
{
    // ---- Move to Place (Connect) ----
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{
                { config.arm_group_name, pipeline_planner }
            });
        stage->setTimeout(1.5);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }

    // ---- Place Object Container ----
    {
        auto container = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
        container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // Lower (Aproximação)
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
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

        // Generate Place Pose + IK
        {
            const std::size_t arm_dof =
                task.getRobotModel()->getJointModelGroup(config.arm_group_name)->getActiveJointModels().size();
            const bool waist_aligned = arm_dof < 6;

            std::unique_ptr<mtc::Stage> generator;
            if (waist_aligned) {
                // const double place_theta = std::atan2(place_pose.position.y, place_pose.position.x);

                // extrai o yaw da pose de destino
                tf2::Quaternion q_target(
                    place_pose.orientation.x,
                    place_pose.orientation.y,
                    place_pose.orientation.z,
                    place_pose.orientation.w);

                double roll, pitch, yaw;
                tf2::Matrix3x3(q_target).getRPY(roll, pitch, yaw);

                tf2::Quaternion q_place;
                q_place.setRPY(0.0, M_PI_2, yaw);

                geometry_msgs::msg::PoseStamped target;
                target.header.frame_id = config.world_frame;
                target.pose.position = place_pose.position;

                // tf2::Quaternion q_place;
                // q_place.setRPY(0.0, M_PI_2, place_theta); // Pitch 90 graus (para baixo)

                target.pose.orientation.x = q_place.x();
                target.pose.orientation.y = q_place.y();
                target.pose.orientation.z = q_place.z();
                target.pose.orientation.w = q_place.w();

                auto stage = std::make_unique<mtc::stages::GeneratePose>("generate place pose");
                stage->properties().set("marker_ns", "place_pose");
                stage->setPose(target);
                stage->setMonitoredStage(attach_stage); // Conectado com o Pick
                generator = std::move(stage);
            } else {
                auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "place_pose");
                stage->setObject(object_id);

                geometry_msgs::msg::PoseStamped target;
                target.header.frame_id = config.world_frame;
                target.pose = place_pose;
                stage->setPose(target);
                stage->setMonitoredStage(attach_stage);
                generator = std::move(stage);
            }

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(generator));
            wrapper->setMaxIKSolutions(waist_aligned ? 8 : 4);
            wrapper->setMinSolutionDistance(0.1);
            wrapper->setIKFrame(config.grasp_frame_transform, config.hand_frame);
            wrapper->setTimeout(1.5);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            container->insert(std::move(wrapper));
        }

        // Release object
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("release object", joint_planner);
            stage->setGroup(config.hand_group_name);
            stage->setGoal(config.hand_open_state);
            container->insert(std::move(stage));
        }

        // Detach object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject(object_id, config.hand_frame);
            container->insert(std::move(stage));
        }

        // Retreat
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
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

        // Remove collision object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("remove object");
            stage->removeObject(object_id);
            container->insert(std::move(stage));
        }

        task.add(std::move(container));
    }
}

} // namespace fbot_manipulator