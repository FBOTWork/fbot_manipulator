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
    mtc::Stage* current_state,
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

    // ---- 1. Open Gripper ----
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_planner);
        stage->setGroup(config.hand_group_name);
        stage->setGoal(config.hand_open_state);
        task.add(std::move(stage));
    }

      // ---- NOVO: Permitir colisão ANTES do move to pick ----
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (approach)");
        
        std::string prefix = (config.hand_group_name.find("right") != std::string::npos) 
            ? "openarm_right" : "openarm_left";
        
        std::vector<std::string> gripper_links = {
            prefix + "_hand",
            prefix + "_left_finger",
            prefix + "_right_finger"
        };
        
        std::vector<std::string> valid_links;
        for (const auto& link : gripper_links) {
            if (task.getRobotModel()->hasLinkModel(link)) {
                valid_links.push_back(link);
            }
        }
        
        RCLCPP_INFO(logger, "Permitindo colisão (approach) entre '%s' e %zu links", 
                   object_id.c_str(), valid_links.size());
        
        stage->allowCollisions(object_id, valid_links, true);
        task.add(std::move(stage));
    }


    // ---- 2. Move to Pick (Connect) ----
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to pick",
            mtc::stages::Connect::GroupPlannerVector{
                { config.arm_group_name, pipeline_planner }
            });
        stage->setTimeout(15.0); 
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }
    
    mtc::Stage* attach_object_stage = nullptr;
    
    // ---- 3. Pick Object Container ----
    {
        auto container = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
        container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // 3.1 Approach (Desce em linha reta)
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

        // 3.2 Generate Grasp Pose + ComputeIK 
        {
            std::unique_ptr<mtc::Stage> generator;
            
            auto gen_stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            gen_stage->properties().configureInitFrom(mtc::Stage::PARENT);
            gen_stage->properties().set("marker_ns", "grasp_pose");
            gen_stage->setPreGraspPose(config.hand_open_state);
            gen_stage->setObject(object_id);
            gen_stage->setAngleDelta(config.grasp_angle_delta); 
            gen_stage->setMonitoredStage(current_state);
            generator = std::move(gen_stage);

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(generator));
            wrapper->setMaxIKSolutions(8); 
            wrapper->setMinSolutionDistance(0.1);
            wrapper->setIKFrame(config.grasp_frame_transform, config.hand_frame);
            wrapper->setTimeout(2.0);
            wrapper->setIgnoreCollisions(true);
            
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            
            container->insert(std::move(wrapper)); 
        }

        // 3.3 PERMITIR COLISÃO ENTRE OBJETO E GARRA (CRUCIAL!)
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
            
            // Lista de links da garra que podem colidir com o objeto
            std::string prefix = (config.hand_group_name.find("right") != std::string::npos) 
                ? "openarm_right" : "openarm_left";
            
            std::vector<std::string> gripper_links = {
                prefix + "_hand",
                prefix + "_left_finger",
                prefix + "_right_finger"
            };
            
            // Filtrar apenas os links que existem
            std::vector<std::string> valid_links;
            for (const auto& link : gripper_links) {
                if (task.getRobotModel()->hasLinkModel(link)) {
                    valid_links.push_back(link);
                }
            }
            
            RCLCPP_INFO(logger, "Permitindo colisão entre '%s' e %zu links da garra", 
                       object_id.c_str(), valid_links.size());
            
            stage->allowCollisions(object_id, valid_links, true);
            container->insert(std::move(stage));
        }

        // 3.4 Close gripper
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_planner);
            stage->setGroup(config.hand_group_name);
            stage->setGoal(config.hand_closed_state);
            container->insert(std::move(stage));
        }

        // 3.5 Attach object
        {
            std::string prefix = "openarm_left";
            if (config.hand_group_name.find("right") != std::string::npos) {
                prefix = "openarm_right";
            }
            std::string attach_link = prefix + "_hand";
            
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject(object_id, attach_link);
            attach_object_stage = stage.get();
            container->insert(std::move(stage));
        }

        // 3.6 Lift object
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
    // Determina o prefixo correto (left ou right)
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
            stage->properties().set("link", config.hand_frame); // OK usar hand_frame aqui, é só para movimento
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

        // // Release object
        // {
        //     auto stage = std::make_unique<mtc::stages::MoveTo>("release object", joint_planner);
        //     stage->setGroup(config.hand_group_name);
        //     stage->setGoal(config.hand_open_state);
        //     container->insert(std::move(stage));
        // }

        // Detach object (CORRIGIDO: Usar o MESMO link físico do attach)
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject(object_id, prefix + "_hand"); // <--- CORREÇÃO CRÍTICA
            container->insert(std::move(stage));
        }

        // Retreat
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(config.retreat_min, config.retreat_max);
            stage->properties().set("marker_ns", "retreat");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = config.world_frame;
            vec.vector.x = 0.0;
            vec.vector.y = 0.0;
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