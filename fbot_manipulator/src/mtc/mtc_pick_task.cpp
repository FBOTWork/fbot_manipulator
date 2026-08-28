#include "fbot_manipulator/mtc/mtc_pick_task.hpp"
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp" // Importando nossa lógica!

namespace fbot_manipulator
{

// [ALTERADO] Adicionado o parâmetro 'bool approach_from_front' no construtor
MtcPickTask::MtcPickTask(rclcpp::Node::SharedPtr node,
                         const std::string& object_id,
                         bool approach_from_front) // <--- NOVO PARÂMETRO
    : MtcTask("pick", node),
      object_id_(object_id),
      approach_from_front_(approach_from_front) // <--- INICIALIZANDO A NOVA VARIÁVEL
{
}

bool MtcPickTask::buildTask()
{
    task_.stages()->setName("pick_" + object_id_);
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this);

    // 1. Current State
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    // Pega a pose do objeto do mapa
    geometry_msgs::msg::Pose object_pose = object_poses_[object_id_];

    // 2. CHAMA A LÓGICA COMPARTILHADA DE PICK
    // [ALTERADO] Passando o parâmetro 'approach_from_front_' no final da chamada
    MtcSharedLogic::addPickStages(
        task_, object_id_, object_pose, current_state, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger(),
        approach_from_front_ // <--- REPASSANDO O PARÂMETRO AQUI
    );

    // 3. Return Home
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_ready_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator