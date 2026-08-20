#include "fbot_manipulator/mtc/mtc_pick_task.hpp"
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp" 

#include <moveit/task_constructor/stages.h>

namespace mtc = ::moveit::task_constructor;

namespace fbot_manipulator
{

MtcPickTask::MtcPickTask(rclcpp::Node::SharedPtr node,
                         const std::string& object_id)
    : MtcTask("pick", node),
      object_id_(object_id)
{
}

bool MtcPickTask::buildTask()
{
    RCLCPP_INFO(node_->get_logger(), "[DEBUG] === INICIANDO BUILD TASK CORRIGIDO ===");
    
    initTask();
    task_.stages()->setName("pick_" + object_id_);
    task_.loadRobotModel(node_);
    
    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.eef_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    mtc::Stage* current_state_ptr = nullptr;

    // 1. Current State
    RCLCPP_INFO(node_->get_logger(), "[DEBUG] 1. Adicionando CurrentState...");
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state_ptr = stage.get(); 
        task_.add(std::move(stage));
    }

    // 2. Open Gripper
    RCLCPP_INFO(node_->get_logger(), "[DEBUG] 2. Adicionando Open Gripper...");
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_planner_);
        stage->setGroup(config_.hand_group_name);
        stage->setGoal(config_.hand_open_state);
        task_.add(std::move(stage));
    }

    // 3. Connect (Move to Pick)
    RCLCPP_INFO(node_->get_logger(), "[DEBUG] 3. Adicionando Connect...");
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to pick",
            mtc::stages::Connect::GroupPlannerVector{
                { config_.arm_group_name, pipeline_planner_ }
            });
        stage->setTimeout(2.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task_.add(std::move(stage));
    }

    // 4. Pick Container
    RCLCPP_INFO(node_->get_logger(), "[DEBUG] 4. Adicionando Pick Container...");
    {
        auto container = std::make_unique<mtc::SerialContainer>("pick object");
        task_.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
        container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // 4.1 Generate Grasp Pose
        RCLCPP_INFO(node_->get_logger(), "[DEBUG] Configurando GenerateGraspPose + ComputeIK...");
        auto gen_stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        gen_stage->properties().configureInitFrom(mtc::Stage::PARENT);
        gen_stage->properties().set("marker_ns", "grasp_pose");
        gen_stage->setPreGraspPose(config_.hand_open_state);
        gen_stage->setObject(object_id_);
        gen_stage->setAngleDelta(config_.grasp_angle_delta);
        
        gen_stage->setMonitoredStage(current_state_ptr); 

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(gen_stage));
        wrapper->setMaxIKSolutions(8); 
        wrapper->setMinSolutionDistance(0.2);
        
        RCLCPP_INFO(node_->get_logger(), "[DEBUG] grasp_frame_transform translation: [%.3f, %.3f, %.3f]",
                   config_.grasp_frame_transform.translation().x(),
                   config_.grasp_frame_transform.translation().y(),
                   config_.grasp_frame_transform.translation().z());
                   
        wrapper->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
        wrapper->setTimeout(2.0);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        
        container->insert(std::move(wrapper));

        // Allow hand-object collision
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
            
            std::vector<std::string> hand_links = {
                "openarm_left_hand",
                "openarm_left_left_finger",
                "openarm_left_right_finger",
                "openarm_left_link7",
                "openarm_left_link6"
            };
            
            stage->allowCollisions(object_id_, hand_links, true);
            container->insert(std::move(stage));
        }
        // 4.3 Close Gripper
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_planner_);
            stage->setGroup(config_.hand_group_name);
            stage->setGoal(config_.hand_closed_state);
            container->insert(std::move(stage));
        }

        // 4.4 Attach Object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject(object_id_, config_.hand_frame);
            container->insert(std::move(stage));
        }

        // 4.5 Lift Object
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner_);
            stage->properties().set("marker_ns", "lift");
            stage->properties().set("link", config_.hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(config_.lift_min, config_.lift_max);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = config_.world_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            container->insert(std::move(stage));
        }

        task_.add(std::move(container));
    }

    RCLCPP_INFO(node_->get_logger(), "[DEBUG] === BUILD TASK CONCLUÍDO COM SUCESSO ===");
    return true;
}
} // namespace fbot_manipulator