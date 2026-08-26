#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <fbot_manipulator_msgs/action/manipulation_task.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "fbot_manipulator/mtc/mtc_task.hpp"
#include "fbot_manipulator/mtc/mtc_pick_task.hpp"
#include "fbot_manipulator/mtc/mtc_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_pick_and_place_task.hpp"

namespace fbot_manipulator
{

namespace mtc_msgs = moveit_task_constructor_msgs;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class ManipulationTaskServer : public rclcpp::Node
{
public:
    using ManipulationTaskAction = fbot_manipulator_msgs::action::ManipulationTask;
    using ExecuteTaskSolutionAction = mtc_msgs::action::ExecuteTaskSolution;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ManipulationTaskAction>;
    using ExecuteGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTaskSolutionAction>;

    ManipulationTaskServer(const rclcpp::NodeOptions& options)
        : Node("manipulation_task_server", options)
    {
        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<ManipulationTaskAction>(
            this, "fbot_manipulator/manipulation_task",
            std::bind(&ManipulationTaskServer::handleGoal, this, _1, _2),
            std::bind(&ManipulationTaskServer::handleCancel, this, _1),
            std::bind(&ManipulationTaskServer::handleAccepted, this, _1));

        execute_action_server_ = rclcpp_action::create_server<ExecuteTaskSolutionAction>(
            this, "execute_task_solution",
            std::bind(&ManipulationTaskServer::handleExecuteGoal, this, _1, _2),
            std::bind(&ManipulationTaskServer::handleExecuteCancel, this, _1),
            std::bind(&ManipulationTaskServer::handleExecuteAccepted, this, _1));

        RCLCPP_INFO(get_logger(), "ManipulationTaskServer ready");
        RCLCPP_INFO(get_logger(), "  - manipulation_task: %s", action_server_ ? "✓" : "✗");
        RCLCPP_INFO(get_logger(), "  - execute_task_solution: %s", execute_action_server_ ? "✓" : "✗");

        grasp_check_ = makeGraspCheckConfig();
        if (grasp_check_.enabled) {
            joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                grasp_check_.topic, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(joint_mtx_);
                    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
                        joint_positions_[msg->name[i]] = msg->position[i];
                });
        }
    }

private:
    // ================= HANDLERS: ManipulationTask =================
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& /*uuid*/, 
        std::shared_ptr<const ManipulationTaskAction::Goal> /*goal*/)
    {
        if (executing_) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>) { 
        return rclcpp_action::CancelResponse::ACCEPT; 
    }
    
    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread([this, goal_handle]() { executeTask(goal_handle); }).detach();
    }

    // ================= HANDLERS: ExecuteTaskSolution (Apenas para RViz) =================
    rclcpp_action::GoalResponse handleExecuteGoal(
        const rclcpp_action::GoalUUID& /*uuid*/, 
        std::shared_ptr<const ExecuteTaskSolutionAction::Goal> /*goal*/) 
    {
        if (executing_rviz_) { 
            RCLCPP_WARN(get_logger(), "Execução RViz bloqueada: outra tarefa RViz em andamento.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleExecuteCancel(const std::shared_ptr<ExecuteGoalHandle> /*goal_handle*/) { 
        return rclcpp_action::CancelResponse::ACCEPT; 
    }

    void handleExecuteAccepted(const std::shared_ptr<ExecuteGoalHandle> goal_handle) {
        std::thread([this, goal_handle]() { executeRealTrajectoriesFromRviz(goal_handle); }).detach();
    }

    // ================= EXECUÇÃO UNIFICADA (LÓGICA DE ANEXO/DESANEXO) =================
        bool executeMtcSolutionDirectly(const MtcTask::Ptr& mtc_task)
    {
        if (!mtc_task) return false;
        
        moveit::planning_interface::PlanningSceneInterface planning_scene;
        
        // 1. Anexar o objeto ANTES de executar (garante que ele se mova com o braço)
        auto objects = planning_scene.getKnownObjectNames();
        if (!objects.empty()) {
            std::string object_id = objects[0]; // Pega o primeiro objeto na cena
            std::string attached_link = "openarm_right_hand"; // Ajuste para o braço correto se necessário
            
            // Detectar qual braço está sendo usado baseado no nome do objeto ou configuração
            // Por simplicidade, vamos assumir que é o braço direito. Você pode melhorar isso depois.
            
            moveit_msgs::msg::AttachedCollisionObject attached_object;
            attached_object.link_name = attached_link;
            attached_object.object.id = object_id;
            attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
            
            auto scene_objects = planning_scene.getObjects({object_id});
            if (scene_objects.find(object_id) != scene_objects.end()) {
                attached_object.object = scene_objects.at(object_id);
                attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
                
                if (planning_scene.applyAttachedCollisionObject(attached_object)) {
                    RCLCPP_INFO(get_logger(), ">>> OBJETO '%s' ANEXADO AO LINK '%s' ANTES DA EXECUÇÃO <<<", 
                               object_id.c_str(), attached_link.c_str());
                } else {
                    RCLCPP_ERROR(get_logger(), "Falha ao anexar objeto '%s' antes da execução", object_id.c_str());
                }
            }
        }

        // 2. Executar a tarefa usando o método nativo do MTC
        bool exec_success = mtc_task->execute();
        
        if (!exec_success) {
            RCLCPP_ERROR(get_logger(), "Falha na execução da tarefa MTC!");
            return false;
        }

        // 3. Desanexar o objeto APÓS a execução terminar
        if (!objects.empty()) {
            std::string object_id = objects[0];
            std::string attached_link = "openarm_right_hand"; // Ajuste para o braço correto
            
            moveit_msgs::msg::AttachedCollisionObject attached_object;
            attached_object.link_name = attached_link;
            attached_object.object.id = object_id;
            attached_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            
            if (planning_scene.applyAttachedCollisionObject(attached_object)) {
                RCLCPP_INFO(get_logger(), ">>> OBJETO '%s' DESANEXADO DO LINK '%s' APÓS A EXECUÇÃO <<<", 
                           object_id.c_str(), attached_link.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "Falha ao desanexar objeto '%s' após a execução", object_id.c_str());
            }
        }

        RCLCPP_INFO(get_logger(), "Tarefa executada com sucesso!");
        return true;
    }

    // ================= LÓGICA DE EXECUÇÃO (MTC) =================
    void publishFeedback(const std::shared_ptr<GoalHandle>& goal_handle, const std::string& stage, float progress) {
        auto feedback = std::make_shared<ManipulationTaskAction::Feedback>();
        feedback->current_stage = stage; 
        feedback->progress = progress;
        goal_handle->publish_feedback(feedback);
    }

void executeTask(const std::shared_ptr<GoalHandle> goal_handle) {
    executing_ = true;
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ManipulationTaskAction::Result>();
    std::string object_id = goal->object_id;
    std::string arm_name = goal->arm_name.empty() ? "left_arm" : goal->arm_name;
    
    RCLCPP_INFO(get_logger(), ">>> EXECUTANDO APENAS PARA O BRAÇO: %s <<<", arm_name.c_str());
    
    publishFeedback(goal_handle, "Adding collision object", 0.0);

    MtcTask::Ptr mtc_task;
    switch (goal->task_type) {
    case ManipulationTaskAction::Goal::PICK:
        mtc_task = std::make_shared<MtcPickTask>(shared_from_this(), object_id); 
        break;
    case ManipulationTaskAction::Goal::PLACE:
        mtc_task = hasGeometricPose(goal->place_pose) ? 
            std::make_shared<MtcPlaceTask>(shared_from_this(), object_id, goal->place_pose) :
            std::make_shared<MtcPlaceTask>(shared_from_this(), object_id, goal->place_pose_name); 
        break;
    case ManipulationTaskAction::Goal::PICK_AND_PLACE:
        mtc_task = hasGeometricPose(goal->place_pose) ?
            std::make_shared<MtcPickAndPlaceTask>(shared_from_this(), object_id, goal->place_pose) :
            std::make_shared<MtcPickAndPlaceTask>(shared_from_this(), object_id, goal->place_pose_name); 
        break;
    default:
        result->success = false; 
        result->message = "Unsupported task type";
        goal_handle->abort(result); 
        executing_ = false; 
        return;
    }

    mtc_task->loadConfigForArm(arm_name);

    const bool object_already_attached = (goal->task_type == ManipulationTaskAction::Goal::PLACE);
    if (!object_already_attached) {
        mtc_task->addCollisionObject(object_id, goal->object_pose, goal->object_size);
    }

    publishFeedback(goal_handle, "Building task", 0.1);
    if (!mtc_task->buildTask()) {
        result->success = false; 
        result->message = "Failed to build task";
        goal_handle->abort(result); 
        executing_ = false; 
        executing_rviz_ = false;
        return;
    }

    publishFeedback(goal_handle, "Planning", 0.3);
    if (!mtc_task->plan()) {
        result->success = false; 
        result->message = "Planning failed";
        goal_handle->abort(result); 
        executing_ = false; 
        executing_rviz_ = false;
        return;
    }

    if (goal_handle->is_canceling()) {
        if (!object_already_attached) mtc_task->removeCollisionObject(object_id);
        result->success = false;
        result->message = "Cancelled";
        goal_handle->canceled(result);
        executing_ = false;
        executing_rviz_ = false;
        return;
    }

    publishFeedback(goal_handle, "Executing", 0.5);
    
    // EXECUTAR MTC (SEM CLOSE GRIPPER)
    bool exec_success = mtc_task->execute();
    
    if (!exec_success) {
        result->success = false;
        result->message = "MTC execution failed";
        goal_handle->abort(result);
        executing_ = false;
        executing_rviz_ = false;
        return;
    }

    // ✅ FECHAR GARRA DEPOIS DO MTC, SE FOR PICK
    if (goal->task_type == ManipulationTaskAction::Goal::PICK ||
        goal->task_type == ManipulationTaskAction::Goal::PICK_AND_PLACE) {
        publishFeedback(goal_handle, "Closing gripper", 0.7);
        
        if (!mtc_task->closeGripper(arm_name)) {
            RCLCPP_WARN(get_logger(), "Gripper close failed, continuing anyway...");
            // Não abortar, continuar mesmo se fechar falhar
        }
    }

    // ✅ ABRIR GARRA DEPOIS, SE FOR PLACE
    if (goal->task_type == ManipulationTaskAction::Goal::PLACE ||
        goal->task_type == ManipulationTaskAction::Goal::PICK_AND_PLACE) {
        publishFeedback(goal_handle, "Opening gripper", 0.7);
        
        if (!mtc_task->openGripper(arm_name)) {
            RCLCPP_WARN(get_logger(), "Gripper open failed, continuing anyway...");
        }
    }

    publishFeedback(goal_handle, "Done", 1.0);
    result->success = true;
    result->message = "Task completed successfully";
    goal_handle->succeed(result);
    executing_ = false;
    executing_rviz_ = false;
}
    // Fallback para execução manual via RViz (usa a mesma lógica unificada)
    void executeRealTrajectoriesFromRviz(const std::shared_ptr<ExecuteGoalHandle> goal_handle)
    {
        auto result = std::make_shared<ExecuteTaskSolutionAction::Result>();
        if (!last_planned_task_) {
            result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            goal_handle->abort(result);
            return;
        }
        
        executing_rviz_ = true;  // MUDANÇA: usar executing_rviz_
        bool success = executeMtcSolutionDirectly(last_planned_task_);
        executing_rviz_ = false;  // MUDANÇA: usar executing_rviz_

        if (success) {
            result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
            goal_handle->succeed(result);
        } else {
            result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            goal_handle->abort(result);
        }
    }
    static bool hasGeometricPose(const geometry_msgs::msg::Pose& p) {
        return !(p.position.x == 0.0 && p.position.y == 0.0 && p.position.z == 0.0 &&
                 p.orientation.x == 0.0 && p.orientation.y == 0.0 && p.orientation.z == 0.0 &&
                 (p.orientation.w == 0.0 || p.orientation.w == 1.0));
    }

    struct GraspCheckConfig { 
        bool enabled = false; 
        std::string topic; 
        std::string finger_joint; 
        double closed_position = 0.0; 
        double min_gap = 0.0; 
    };

    GraspCheckConfig grasp_check_;
    
    GraspCheckConfig makeGraspCheckConfig() { 
        grasp_check_.enabled = false; 
        return grasp_check_; 
    }

    bool latestFingerPosition(double& position)
    {
        std::lock_guard<std::mutex> lk(joint_mtx_);
        auto it = joint_positions_.find(grasp_check_.finger_joint);
        if (it == joint_positions_.end()) return false;
        position = it->second;
        return true;
    }

    bool verifyGrasp(const MtcTask::Ptr& task, const std::string& object_id, std::string& message)
    {
        if (!grasp_check_.enabled) return true;

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        double position = 0.0;
        bool have_reading = false;
        const auto deadline = now() + rclcpp::Duration::from_seconds(1.0);
        do {
            have_reading = latestFingerPosition(position);
            if (have_reading) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } while (now() < deadline);

        if (!have_reading) {
            RCLCPP_WARN(get_logger(), "Grasp check: no reading available; skipping verification");
            return true;
        }

        const double opening = position - grasp_check_.closed_position;
        RCLCPP_INFO(get_logger(), "Grasp check: opening=%.4f, min=%.4f", opening, grasp_check_.min_gap);

        if (opening <= grasp_check_.min_gap) {
            task->detachAndRemoveObject(object_id);
            message = "grasp verification failed: gripper closed empty";
            return false;
        }
        return true;
    }

    // Variáveis de membro
    rclcpp_action::Server<ManipulationTaskAction>::SharedPtr action_server_;
    rclcpp_action::Server<ExecuteTaskSolutionAction>::SharedPtr execute_action_server_;
    
    bool executing_ = false;
    bool executing_rviz_ = false;  
    MtcTask::Ptr last_planned_task_;
    
    std::map<std::string, rclcpp_action::Client<FollowJointTrajectory>::SharedPtr> fjt_clients_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    std::mutex joint_mtx_;
    std::map<std::string, double> joint_positions_;
};

} // namespace fbot_manipulator

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<fbot_manipulator::ManipulationTaskServer>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}