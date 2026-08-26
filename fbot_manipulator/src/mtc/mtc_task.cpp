#include "fbot_manipulator/mtc/mtc_task.hpp"
#include <chrono>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace fbot_manipulator
{

MtcTask::MtcTask(const std::string& task_name, rclcpp::Node::SharedPtr node)
    : task_name_(task_name), node_(node)
{
    loadConfig();
    setupSolvers();
}

void MtcTask::loadConfigForArm(const std::string& arm_name)
{
    std::string prefix = (arm_name == "right_arm") ? "right" : "left";
    
    config_.arm_group_name = arm_name;
    config_.hand_group_name = prefix + "_gripper";
    config_.eef_name = prefix + "_ee";
    config_.hand_frame = "openarm_" + prefix + "_hand_tcp";

    RCLCPP_INFO(logger(), "[MtcTask:%s] Reconfigured for arm: %s (eef: %s, hand_frame: %s)",
                task_name_.c_str(), arm_name.c_str(), config_.eef_name.c_str(), config_.hand_frame.c_str());
}

void MtcTask::initTask()
{
    task_.reset();
    task_.stages()->setName(task_name_);
    task_.loadRobotModel(node_);
    
    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("hand", config_.hand_group_name);
    task_.setProperty("hand_grasping_frame", config_.hand_frame);
    task_.setProperty("ik_frame", config_.hand_frame);
}

void MtcTask::setCollisionObjectColor(const std::string& object_id, float r, float g, float b, float a)
{
    moveit_msgs::msg::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;

    moveit_msgs::msg::ObjectColor obj_color;
    obj_color.id = object_id;
    obj_color.color.r = r;
    obj_color.color.g = g;
    obj_color.color.b = b;
    obj_color.color.a = a;

    planning_scene_msg.object_colors.push_back(obj_color);
    psi_.applyPlanningScene(planning_scene_msg); 
}

void MtcTask::loadConfig()
{
    node_->get_parameter_or("mtc.arm_group_name", config_.arm_group_name, std::string("left_arm"));
    node_->get_parameter_or("mtc.eef_name", config_.eef_name, std::string("left_ee"));
    node_->get_parameter_or("mtc.hand_group_name", config_.hand_group_name, std::string("left_gripper"));
    node_->get_parameter_or("mtc.hand_frame", config_.hand_frame, std::string("openarm_left_hand_tcp"));
    node_->get_parameter_or("mtc.world_frame", config_.world_frame, std::string("world"));
    node_->get_parameter_or("mtc.surface_link", config_.surface_link, std::string("world"));
    node_->get_parameter_or("mtc.hand_open_state", config_.hand_open_state, std::string("open"));
    node_->get_parameter_or("mtc.hand_closed_state", config_.hand_closed_state, std::string("closed"));
    node_->get_parameter_or("mtc.arm_home_state", config_.arm_home_state, std::string("home"));
    node_->get_parameter_or("mtc.arm_ready_state", config_.arm_ready_state, std::string("hands_up"));
    node_->get_parameter_or("mtc.approach_min", config_.approach_min, 0.05);
    node_->get_parameter_or("mtc.approach_max", config_.approach_max, 0.15);
    node_->get_parameter_or("mtc.lift_min", config_.lift_min, 0.05);
    node_->get_parameter_or("mtc.lift_max", config_.lift_max, 0.15);
    node_->get_parameter_or("mtc.retreat_min", config_.retreat_min, 0.05);
    node_->get_parameter_or("mtc.retreat_max", config_.retreat_max, 0.15);
    node_->get_parameter_or("mtc.max_solutions", config_.max_solutions, 5);
    node_->get_parameter_or("mtc.grasp_angle_delta", config_.grasp_angle_delta, 0.262);
    
    double grasp_offset = 0.0;
    node_->get_parameter_or("mtc.grasp_offset", grasp_offset, 0.0);

    std::vector<double> grasp_rpy;
    node_->get_parameter_or("mtc.grasp_frame_rpy", grasp_rpy, std::vector<double>{0.0, -M_PI / 2, M_PI});
    
    config_.grasp_frame_transform = Eigen::Isometry3d::Identity();
    config_.grasp_frame_transform.rotate(
        Eigen::AngleAxisd(grasp_rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(grasp_rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(grasp_rpy[0], Eigen::Vector3d::UnitX()));
    config_.grasp_frame_transform.translate(Eigen::Vector3d(grasp_offset, 0.0, 0.0));

    RCLCPP_INFO(logger(), "[MtcTask:%s] Config loaded: arm='%s', hand='%s', frame='%s'",
                task_name_.c_str(), config_.arm_group_name.c_str(), config_.hand_group_name.c_str(), config_.hand_frame.c_str());
}

void MtcTask::setupSolvers()
{
    pipeline_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    pipeline_planner_->setMaxVelocityScalingFactor(0.5);
    pipeline_planner_->setMaxAccelerationScalingFactor(0.5);

    cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner_->setMaxVelocityScalingFactor(0.5);
    cartesian_planner_->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner_->setStepSize(0.005);

    joint_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
}

void MtcTask::addCollisionObject(const std::string& object_id, const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Vector3& size)
{
    RCLCPP_INFO(logger(), ">>> ADICIONANDO OBJETO COM ID: '%s' <<<", object_id.c_str());
    
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = config_.world_frame;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = { size.x, size.y, size.z };
    object.pose = pose;

    psi_.applyCollisionObject(object);
    object_poses_[object_id] = pose;
    
    RCLCPP_INFO(logger(), ">>> OBJETO '%s' ADICIONADO COM SUCESSO <<<", object_id.c_str());
}

void MtcTask::removeCollisionObject(const std::string& object_id)
{
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = config_.world_frame;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi_.applyCollisionObject(object);
    object_poses_.erase(object_id);
}

void MtcTask::detachAndRemoveObject(const std::string& object_id)
{
    moveit_msgs::msg::AttachedCollisionObject detach;
    detach.link_name = config_.hand_frame;
    detach.object.id = object_id;
    detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi_.applyAttachedCollisionObject(detach);
    removeCollisionObject(object_id);
}

bool MtcTask::plan()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(logger(), "[MtcTask:%s] ========================================", task_name_.c_str());
    RCLCPP_INFO(logger(), "[MtcTask:%s] INICIANDO PLANEJAMENTO", task_name_.c_str());
    RCLCPP_INFO(logger(), "[MtcTask:%s] ========================================", task_name_.c_str());

    // 1. Inicializar a tarefa
    RCLCPP_INFO(logger(), "[MtcTask:%s] Chamando task_.init()...", task_name_.c_str());
    auto start_init = std::chrono::steady_clock::now();
    
    try {
        task_.init();
        auto end_init = std::chrono::steady_clock::now();
        auto duration_init = std::chrono::duration_cast<std::chrono::milliseconds>(end_init - start_init).count();
        RCLCPP_INFO(logger(), "[MtcTask:%s] Task initialized successfully (%ld ms)", task_name_.c_str(), duration_init);
    } catch (const moveit::task_constructor::InitStageException& e) {
        RCLCPP_ERROR_STREAM(logger(), "[MtcTask:" << task_name_ << "] InitStageException: " << e);
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] Exceção: %s", task_name_.c_str(), e.what());
        return false;
    }

    // 2. Planejar SEM timeout manual (usa o nativo se houver na config)
    RCLCPP_INFO(logger(), "[MtcTask:%s] Chamando task_.plan() com max_solutions=%d...", 
                task_name_.c_str(), config_.max_solutions);
    
    auto start_plan = std::chrono::steady_clock::now();
    
    try {
        moveit::core::MoveItErrorCode plan_result = task_.plan(config_.max_solutions);
        
        auto end_plan = std::chrono::steady_clock::now();
        auto duration_plan = std::chrono::duration_cast<std::chrono::milliseconds>(end_plan - start_plan).count();
        
        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger(), "[MtcTask:%s] Planning failed (%ld ms)", task_name_.c_str(), duration_plan);
            task_.printState();
            return false;
        }
        
        RCLCPP_INFO(logger(), "[MtcTask:%s] Planning succeeded with %zu solutions (%ld ms)", 
                    task_name_.c_str(), task_.solutions().size(), duration_plan);
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] Exceção no plan: %s", task_name_.c_str(), e.what());
        return false;
    }
}

bool MtcTask::execute()
{
    if (task_.solutions().empty()) {
        RCLCPP_ERROR(logger(), "No planning solution available!");
        return false;
    }

    moveit_task_constructor_msgs::msg::Solution solution_msg;
    task_.solutions().front()->toMsg(solution_msg);
    
    return executeSolution(solution_msg);
}

bool MtcTask::executeSolution(const moveit_task_constructor_msgs::msg::Solution& solution_msg)
{
    RCLCPP_INFO(logger(), ">>> Executando solução MTC Otimizada e Fluida <<<");

    moveit::planning_interface::MoveGroupInterface arm_group(node_, config_.arm_group_name);
    moveit::planning_interface::MoveGroupInterface gripper_group(node_, config_.hand_group_name);
    
    const auto& arm_joints = arm_group.getJointNames();
    const auto& gripper_joints = gripper_group.getJointNames();

    moveit_msgs::msg::RobotTrajectory current_merged_traj;
    std::string current_group = "";

    // Função lambda para executar a trajetória acumulada
    auto execute_merged = [&]() -> bool {
        if (current_merged_traj.joint_trajectory.points.empty()) return true;

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = current_merged_traj;

        auto& active_group = (current_group == "arm") ? arm_group : gripper_group;
        auto& active_joints = (current_group == "arm") ? arm_joints : gripper_joints;
        auto current_state = active_group.getCurrentJointValues();
        
        // Correção de Start State Mismatch para a trajetória fundida
        if (!current_state.empty() && !plan.trajectory_.joint_trajectory.points.empty()) {
            auto& first_point = plan.trajectory_.joint_trajectory.points.front();
            for (size_t j = 0; j < plan.trajectory_.joint_trajectory.joint_names.size(); ++j) {
                const std::string& j_name = plan.trajectory_.joint_trajectory.joint_names[j];
                auto it = std::find(active_joints.begin(), active_joints.end(), j_name);
                if (it != active_joints.end()) {
                    size_t idx = std::distance(active_joints.begin(), it);
                    if (idx < current_state.size()) {
                        first_point.positions[j] = current_state[idx];
                    }
                }
            }
            first_point.time_from_start.sec = 0;
            first_point.time_from_start.nanosec = 0;
        }

        active_group.setStartStateToCurrentState();
        RCLCPP_INFO(logger(), "Executando Movimento Contínuo (%zu pontos) no grupo: %s", 
                    current_merged_traj.joint_trajectory.points.size(), current_group.c_str());
        
        auto result = active_group.execute(plan);
        current_merged_traj = moveit_msgs::msg::RobotTrajectory(); // Reseta o acumulador
        current_group = "";

        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger(), "Falha na execução do bloco! Erro: %d", result.val);
            return false;
        }
        return true;
    };

    // Percorre todos os passos (sub_trajectories)
    for (size_t i = 0; i < solution_msg.sub_trajectory.size(); ++i) {
        const auto& sub = solution_msg.sub_trajectory[i];

        bool has_scene_diff = !sub.scene_diff.robot_state.attached_collision_objects.empty() || 
                              !sub.scene_diff.world.collision_objects.empty() || 
                              !sub.scene_diff.allowed_collision_matrix.entry_names.empty();

        // 1. SE HOUVER MUDANÇA DE CENA (Attach/Detach)
        if (has_scene_diff) {
            if (!execute_merged()) return false; // Descarrega a fila de movimentos antes de mudar a física
            RCLCPP_INFO(logger(), "Aplicando mudanças de cena (Attach/Detach/ACMs) do Step %zu", i);
            psi_.applyPlanningScene(sub.scene_diff);
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Espera a física atualizar
        }

        // 2. SE HOUVER MOVIMENTO FÍSICO
        if (!sub.trajectory.joint_trajectory.points.empty()) {
            bool is_arm = false;
            for (const auto& j : sub.trajectory.joint_trajectory.joint_names) {
                if (std::find(arm_joints.begin(), arm_joints.end(), j) != arm_joints.end()) {
                    is_arm = true; break;
                }
            }
            
            std::string target_group = is_arm ? "arm" : "gripper";

            // Se o grupo alvo mudou (ex: braço -> garra), executa os do braço antes de fechar a garra
            if (current_group != "" && current_group != target_group) {
                if (!execute_merged()) return false;
                std::this_thread::sleep_for(std::chrono::milliseconds(250)); // Pausa entre ações da garra/braço
            }

            current_group = target_group;

            // ACUMULA A TRAJETÓRIA
            if (current_merged_traj.joint_trajectory.points.empty()) {
                current_merged_traj = sub.trajectory;
            } else {
                double time_offset = rclcpp::Duration(current_merged_traj.joint_trajectory.points.back().time_from_start).seconds();
                
                // Pula o primeiro ponto (p=1) para não haver timestamps/pontos duplicados no meio da emenda
                for (size_t p = 1; p < sub.trajectory.joint_trajectory.points.size(); ++p) { 
                    auto pt = sub.trajectory.joint_trajectory.points[p];
                    double pt_time = rclcpp::Duration(pt.time_from_start).seconds();
                    pt.time_from_start = rclcpp::Duration::from_seconds(time_offset + pt_time);
                    current_merged_traj.joint_trajectory.points.push_back(pt);
                }
            }
        }
    }
    
    // Executa qualquer bloco de movimento que tenha sobrado no final
    if (!execute_merged()) return false;

    RCLCPP_INFO(logger(), ">>> Execução MTC concluída com sucesso (Fluida)! <<<");
    return true;
}

} // namespace fbot_manipulator