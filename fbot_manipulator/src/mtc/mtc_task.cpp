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

bool MtcTask::closeGripper(const std::string& arm_name)
{
    try {
        std::string gripper_group = (arm_name == "left_arm") ? "left_gripper" : "right_gripper";
        
        moveit::planning_interface::MoveGroupInterface gripper(node_, gripper_group);
        gripper.setMaxVelocityScalingFactor(0.5);
        gripper.setMaxAccelerationScalingFactor(0.5);
        gripper.setNamedTarget(config_.hand_closed_state);
        
        RCLCPP_INFO(logger(), "[MtcTask] Closing gripper '%s'...", gripper_group.c_str());
        
        if (gripper.move() == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger(), "[MtcTask] Gripper closed successfully!");
            return true;
        } else {
            RCLCPP_ERROR(logger(), "[MtcTask] Failed to close gripper");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "[MtcTask] Exception closing gripper: %s", e.what());
        return false;
    }
}

bool MtcTask::openGripper(const std::string& arm_name)
{
    try {
        std::string gripper_group = (arm_name == "left_arm") ? "left_gripper" : "right_gripper";
        
        moveit::planning_interface::MoveGroupInterface gripper(node_, gripper_group);
        gripper.setMaxVelocityScalingFactor(0.5);
        gripper.setMaxAccelerationScalingFactor(0.5);
        gripper.setNamedTarget(config_.hand_open_state);
        
        RCLCPP_INFO(logger(), "[MtcTask] Opening gripper '%s'...", gripper_group.c_str());
        
        if (gripper.move() == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger(), "[MtcTask] Gripper opened successfully!");
            return true;
        } else {
            RCLCPP_ERROR(logger(), "[MtcTask] Failed to open gripper");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "[MtcTask] Exception opening gripper: %s", e.what());
        return false;
    }
}



bool MtcTask::plan()
{
    // Aguarda para garantir que o applyCollisionObject foi processado
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(logger(), "[MtcTask:%s] Iniciando plan()...", task_name_.c_str());

    try {
        // Chama init() explicitamente
        RCLCPP_INFO(logger(), "[MtcTask:%s] Chamando task_.init()...", task_name_.c_str());
        task_.init();
        RCLCPP_INFO(logger(), "[MtcTask:%s] Task initialized successfully", task_name_.c_str());
        
    } catch (const moveit::task_constructor::InitStageException& e) {
        RCLCPP_ERROR_STREAM(logger(), "==================================================");
        RCLCPP_ERROR_STREAM(logger(), "[MtcTask:" << task_name_ << "] InitStageException durante init():");
        RCLCPP_ERROR_STREAM(logger(), e);
        RCLCPP_ERROR_STREAM(logger(), "==================================================");
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] Exceção genérica durante init(): %s", task_name_.c_str(), e.what());
        return false;
    } catch (...) {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] Exceção desconhecida durante init()", task_name_.c_str());
        return false;
    }

    // Agora que o init() passou, podemos planejar com segurança
    RCLCPP_INFO(logger(), "[MtcTask:%s] Chamando task_.plan()...", task_name_.c_str());
    
    try {
        if (!task_.plan(config_.max_solutions)) {
            RCLCPP_ERROR(logger(), "[MtcTask:%s] Planning failed", task_name_.c_str());
            task_.printState();
            return false;
        }
        RCLCPP_INFO(logger(), "[MtcTask:%s] Planning succeeded with %zu solutions", 
                    task_name_.c_str(), task_.solutions().size());
        return true;
        
    } catch (const moveit::task_constructor::InitStageException& e) {
        RCLCPP_ERROR_STREAM(logger(), "==================================================");
        RCLCPP_ERROR_STREAM(logger(), "[MtcTask:" << task_name_ << "] InitStageException durante plan():");
        RCLCPP_ERROR_STREAM(logger(), e);
        RCLCPP_ERROR_STREAM(logger(), "==================================================");
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] Exceção genérica durante plan(): %s", task_name_.c_str(), e.what());
        return false;
    } catch (...) {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] Exceção desconhecida durante plan()", task_name_.c_str());
        return false;
    }
}

bool MtcTask::execute()
{
    if (task_.solutions().empty()) {
        RCLCPP_ERROR(logger(), "No planning solution available!");
        return false;
    }

    RCLCPP_INFO(logger(), ">>> EXECUTANDO TRAJETÓRIA VIA MOVE_GROUP_INTERFACE... <<<");
    
    const auto& solution = task_.solutions().front();
    
    moveit_task_constructor_msgs::msg::Solution solution_msg;
    solution->toMsg(solution_msg);
    
    RCLCPP_INFO(logger(), ">>> Solução convertida: %zu sub-trajectories <<<", 
                solution_msg.sub_trajectory.size());
    
    // Obter os joints do grupo do braço
    moveit::planning_interface::MoveGroupInterface move_group(node_, config_.arm_group_name);
    const auto& arm_joint_names = move_group.getJointNames();
    
    RCLCPP_INFO(logger(), ">>> Joints do braço (%zu): <<<", arm_joint_names.size());
    for (const auto& joint : arm_joint_names) {
        RCLCPP_INFO(logger(), "    - %s", joint.c_str());
    }
    
    // Extrair e combinar APENAS sub-trajectórias do braço
    moveit_msgs::msg::RobotTrajectory arm_trajectory;
    arm_trajectory.joint_trajectory.joint_names = arm_joint_names;
    
    double time_offset = 0.0; // Offset acumulado para garantir monotonicidade
    
    for (const auto& sub_traj : solution_msg.sub_trajectory) {
        const auto& traj = sub_traj.trajectory.joint_trajectory;
        
        if (traj.points.empty()) continue;
        
        // Verificar se esta sub-trajectória contém joints do braço
        bool is_arm_trajectory = false;
        for (const auto& joint : traj.joint_names) {
            if (std::find(arm_joint_names.begin(), arm_joint_names.end(), joint) != arm_joint_names.end()) {
                is_arm_trajectory = true;
                break;
            }
        }
        
        if (!is_arm_trajectory) {
            RCLCPP_INFO(logger(), ">>> Pulando sub-trajectória (não é do braço) <<<");
            continue;
        }
        
        RCLCPP_INFO(logger(), ">>> Processando sub-trajectória do braço com %zu pontos <<<",
                   traj.points.size());
        
        // Mapear índices dos joints do braço nesta sub-trajectória
        std::vector<int> joint_indices;
        for (const auto& arm_joint : arm_joint_names) {
            auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(), arm_joint);
            if (it != traj.joint_names.end()) {
                joint_indices.push_back(std::distance(traj.joint_names.begin(), it));
            } else {
                joint_indices.push_back(-1); // Joint não está nesta sub-trajectória
            }
        }
        
        // Calcular o tempo do último ponto da sub-trajectória anterior
        if (!arm_trajectory.joint_trajectory.points.empty()) {
            time_offset = rclcpp::Duration(
                arm_trajectory.joint_trajectory.points.back().time_from_start
            ).seconds();
            
            // Adicionar uma pequena margem para garantir monotonicidade estrita
            time_offset += 0.001; // 1ms de margem
        }
        
        // Adicionar pontos com timestamps reindexados
        for (size_t i = 0; i < traj.points.size(); ++i) {
            const auto& src_point = traj.points[i];
            trajectory_msgs::msg::JointTrajectoryPoint dst_point;
            
            // Copiar posições apenas para os joints do braço
            dst_point.positions.resize(arm_joint_names.size(), 0.0);
            for (size_t j = 0; j < arm_joint_names.size(); ++j) {
                if (joint_indices[j] >= 0 && joint_indices[j] < (int)src_point.positions.size()) {
                    dst_point.positions[j] = src_point.positions[joint_indices[j]];
                }
            }
            
            // Copiar velocidades se existirem
            if (!src_point.velocities.empty()) {
                dst_point.velocities.resize(arm_joint_names.size(), 0.0);
                for (size_t j = 0; j < arm_joint_names.size(); ++j) {
                    if (joint_indices[j] >= 0 && joint_indices[j] < (int)src_point.velocities.size()) {
                        dst_point.velocities[j] = src_point.velocities[joint_indices[j]];
                    }
                }
            }
            
            // Reindexar timestamp: offset + tempo relativo do ponto
            double relative_time = rclcpp::Duration(src_point.time_from_start).seconds();
            dst_point.time_from_start = rclcpp::Duration::from_seconds(time_offset + relative_time);
            
            arm_trajectory.joint_trajectory.points.push_back(dst_point);
        }
    }
    
    if (arm_trajectory.joint_trajectory.points.empty()) {
        RCLCPP_ERROR(logger(), "Nenhuma sub-trajectória do braço encontrada!");
        return false;
    }
    
    RCLCPP_INFO(logger(), ">>> Trajectória do braço final: %zu pontos, tempo total: %.3f s <<<", 
                arm_trajectory.joint_trajectory.points.size(),
                rclcpp::Duration(arm_trajectory.joint_trajectory.points.back().time_from_start).seconds());
    
    // Verificar monotonicidade dos timestamps
    for (size_t i = 1; i < arm_trajectory.joint_trajectory.points.size(); ++i) {
        double t_prev = rclcpp::Duration(arm_trajectory.joint_trajectory.points[i-1].time_from_start).seconds();
        double t_curr = rclcpp::Duration(arm_trajectory.joint_trajectory.points[i].time_from_start).seconds();
        if (t_curr <= t_prev) {
            RCLCPP_ERROR(logger(), ">>> ERRO: Timestamp não monotônico no ponto %zu (%.6f <= %.6f) <<<",
                        i, t_curr, t_prev);
            return false;
        }
    }
    
    RCLCPP_INFO(logger(), ">>> Todos os timestamps são monotonicamente crescentes ✓ <<<");
    
    move_group.setStartStateToCurrentState();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = arm_trajectory;
    plan.planning_time_ = 0.0;
    
    RCLCPP_INFO(logger(), ">>> Executando trajectória do braço... <<<");
    
    moveit::core::MoveItErrorCode result = move_group.execute(plan);
    
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(logger(), ">>> Execução falhou! Código: %d <<<", result.val);
        return false;
    }

    RCLCPP_INFO(logger(), ">>> Execução concluída com sucesso! <<<");
    return true;
}

} // namespace fbot_manipulator