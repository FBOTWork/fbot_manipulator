#ifndef FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP
#define FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// Includes do MoveIt Task Constructor
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/solvers.h>

// NOTA: Você precisará incluir aqui o arquivo onde a sua struct 'config_' está definida.
// Geralmente ela fica no cabeçalho da tarefa base. Substitua pelo nome correto:
// #include "fbot_manipulator/mtc/mtc_task.hpp" 

namespace fbot_manipulator
{

namespace mtc = moveit::task_constructor;
// O tipo exato da sua configuração (substitua 'MtcConfig' pelo nome real da sua struct, 
// se ela tiver outro nome no seu projeto)
struct MtcConfig;
class MtcTask; 

class MtcSharedLogic
{
public:
    /**
     * @brief Constrói todos os estágios de Pick e injeta na 'task'.
     * @return O ponteiro para o estágio 'attach_object' (necessário para o Place depois).
     */

    static void setupWorkspace(MtcTask* task_instance);
     
    static mtc::Stage* addPickStages(
        mtc::Task& task,
        const std::string& object_id,
        const geometry_msgs::msg::Pose& object_pose,
        mtc::Stage* current_state,
        const MtcConfig& config,
        std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
        std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
        std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
        rclcpp::Logger logger,
        bool approach_from_front = false);

    /**
     * @brief Constrói todos os estágios de Place (Top-Down) e injeta na 'task'.
     */
    static void addPlaceStages(
        mtc::Task& task,
        const std::string& object_id,
        const geometry_msgs::msg::Pose& place_pose,
        mtc::Stage* attach_stage, // O estágio gerado pelo Pick
        const MtcConfig& config,
        std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
        std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
        std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
        rclcpp::Logger logger,
        bool approach_from_front = false);
};

} // namespace fbot_manipulator

#endif // FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP