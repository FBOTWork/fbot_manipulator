#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"
#include "fbot_manipulator/mtc/mtc_task.hpp" 
#include <moveit/task_constructor/stages.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/current_state.h>

namespace mtc = ::moveit::task_constructor;

namespace fbot_manipulator
{

void MtcSharedLogic::setupWorkspace(MtcTask* task_instance)
{
}


mtc::Stage* MtcSharedLogic::addPickStages(
    mtc::Task& task,
    const std::string& object_id,
    const geometry_msgs::msg::Pose& object_pose,
    mtc::Stage* current_state, // Já fornecido pelo buildTask!
    const MtcConfig& config,
    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
    rclcpp::Logger logger)
{
    auto robot_model = task.getRobotModel();
    if (!robot_model) {
        RCLCPP_ERROR(logger, "[addPickStages] Robot model not loaded!");
        return nullptr;
    }

    // 1. Open Gripper
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_planner);
        stage->setGroup(config.hand_group_name);
        stage->setGoal(config.hand_open_state);
        task.add(std::move(stage));
    }

    // 2. Permitir colisão (approach)
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (approach)");
        std::string prefix = (config.hand_group_name.find("right") != std::string::npos) ? "openarm_right" : "openarm_left";
        std::vector<std::string> gripper_links = {prefix + "_hand", prefix + "_left_finger", prefix + "_right_finger"};
        std::vector<std::string> valid_links;
        for (const auto& link : gripper_links) {
            if (robot_model->hasLinkModel(link)) valid_links.push_back(link);
        }
        stage->allowCollisions(object_id, valid_links, true);
        task.add(std::move(stage));
    }

    // 3. Move to Pick (Connect) - APENAS O BRAÇO
    {
        auto stage = std::make_unique<mtc::stages::Connect>("move to pick", 
            mtc::stages::Connect::GroupPlannerVector{
                {config.arm_group_name, pipeline_planner}  // Apenas o braço!
            });
        stage->setTimeout(120.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }
    
    mtc::Stage* attach_object_stage = nullptr;
    
    // 4. Pick Object Container
    {
        auto container = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
        container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // 4.1 Approach (Desce em linha reta)
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach");
            stage->properties().set("link", config.hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.01, 0.05);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = config.world_frame;
            vec.vector.x = 0.0;
            vec.vector.y = 0.0;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            container->insert(std::move(stage));
        }

        // 4.2 Generate Grasp Pose + ComputeIK 
        {
            auto gen_stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            gen_stage->properties().configureInitFrom(mtc::Stage::PARENT);
            gen_stage->setPreGraspPose(config.hand_open_state);
            gen_stage->setObject(object_id);
            gen_stage->setAngleDelta(config.grasp_angle_delta); 
            
            // Usa o current_state passado como referência (já existe no buildTask)
            gen_stage->setMonitoredStage(current_state); 
            
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(gen_stage));
            wrapper->setMaxIKSolutions(8); 
            wrapper->setMinSolutionDistance(0.1);
            wrapper->setIKFrame(config.grasp_frame_transform, config.hand_frame);
            wrapper->setTimeout(5.0);
            wrapper->setIgnoreCollisions(true);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            container->insert(std::move(wrapper)); 
        }

        // 4.3 PERMITIR COLISÃO ENTRE OBJETO E GARRA (dentro do container)
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
            std::string prefix = (config.hand_group_name.find("right") != std::string::npos) ? "openarm_right" : "openarm_left";
            std::vector<std::string> gripper_links = {prefix + "_hand", prefix + "_left_finger", prefix + "_right_finger"};
            std::vector<std::string> valid_links;
            for (const auto& link : gripper_links) {
                if (robot_model->hasLinkModel(link)) valid_links.push_back(link);
            }
            stage->allowCollisions(object_id, valid_links, true);
            container->insert(std::move(stage));
        }

        // 4.4 Close gripper
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_planner);
            stage->setGroup(config.hand_group_name);
            stage->setGoal(config.hand_closed_state);
            container->insert(std::move(stage));
        }

        // 4.5 Attach object
        {
            std::string prefix = (config.hand_group_name.find("right") != std::string::npos) ? "openarm_right" : "openarm_left";
            std::string attach_link = prefix + "_hand";
            
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject(object_id, attach_link);
            attach_object_stage = stage.get();
            container->insert(std::move(stage));
        }

        // 4.6 Lift object
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().set("marker_ns", "lift");
            stage->properties().set("link", config.hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.05, 0.10);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = config.world_frame;
            vec.vector.x = 0.0;
            vec.vector.y = 0.0;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
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
    std::string prefix = "openarm_left";
    if (config.hand_group_name.find("right") != std::string::npos) {
        prefix = "openarm_right";
    }

    // ---- Move to Place (Connect) ----
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{
                { config.arm_group_name, pipeline_planner }
            });
        stage->setTimeout(10.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }

    // ---- Place Object Container ----
    {
        auto container = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
        container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // Lower object
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
            stage->properties().set("marker_ns", "lower");
            stage->properties().set("link", config.hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.01, 0.05);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = config.world_frame;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            container->insert(std::move(stage));
        }

        // Generate Place Pose + IK
        {
            std::unique_ptr<mtc::Stage> generator;
            
            auto stage = std::make_unique<mtc::stages::GeneratePose>("generate place pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "place_pose");
            stage->setMonitoredStage(attach_stage);
            
            geometry_msgs::msg::PoseStamped target;
            target.header.frame_id = config.world_frame;
            target.pose.position = place_pose.position;
            target.pose.orientation = place_pose.orientation;
            stage->setPose(target);
            
            generator = std::move(stage);

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(generator));
            wrapper->setMaxIKSolutions(4);
            wrapper->setMinSolutionDistance(0.05);
            wrapper->setIKFrame(config.grasp_frame_transform, config.hand_frame);
            wrapper->setTimeout(2.0);
            wrapper->setIgnoreCollisions(true);
            
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            container->insert(std::move(wrapper));
        }

        // RELEASE OBJECT - VERSÃO CORRIGIDA
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("release object", joint_planner); // AQUI: joint_planner no lugar de pipeline_planner
            stage->setGroup(config.hand_group_name);
            stage->setGoal(config.hand_open_state);
            // NÃO herdar propriedades
            container->insert(std::move(stage));
        }

        // Detach object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject(object_id, prefix + "_hand");
            container->insert(std::move(stage));
        }

        // Retreat
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.05, 0.15);
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