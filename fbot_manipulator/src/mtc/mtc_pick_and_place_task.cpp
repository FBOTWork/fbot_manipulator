#include "fbot_manipulator/mtc/mtc_pick_and_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp" // Importando nossa lógica!

namespace fbot_manipulator
{

// [ALTERADO] Adicionado o parâmetro 'bool approach_from_front' ao construtor com Pose
MtcPickAndPlaceTask::MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                                         const std::string& object_id,
                                         const geometry_msgs::msg::Pose& place_pose,
                                         bool approach_from_front) // <--- NOVO PARÂMETRO
    : MtcTask("pick_and_place", node),
      object_id_(object_id),
      place_pose_(place_pose),
      approach_from_front_(approach_from_front) // <--- INICIALIZANDO A NOVA VARIÁVEL
{
}

// [ALTERADO] Adicionado o parâmetro 'bool approach_from_front' ao construtor com nome da Pose
MtcPickAndPlaceTask::MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                                         const std::string& object_id,
                                         const std::string& place_pose_name,
                                         bool approach_from_front) // <--- NOVO PARÂMETRO
    : MtcTask("pick_and_place", node),
      object_id_(object_id),
      place_pose_name_(place_pose_name),
      approach_from_front_(approach_from_front) // <--- INICIALIZANDO A NOVA VARIÁVEL
{
}

bool MtcPickAndPlaceTask::buildTask()
{
    task_.stages()->setName("pick_and_place_" + object_id_);
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

    // 2. CHAMA O PICK E GUARDA O RESULTADO
    // [ALTERADO] Passando 'approach_from_front_' no final da chamada do Pick
    mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
        task_, object_id_, object_pose, current_state, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger(),
        approach_from_front_ // <--- REPASSANDO O PARÂMETRO PARA O PICK
    );

    // 3. Move Home (Opcional entre o Pick e o Place)
    // {
    //     auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
    //     stage->setGroup(config_.arm_group_name);
    //     stage->setGoal(config_.arm_ready_state);
    //     task_.add(std::move(stage));
    // }

    // 4. CHAMA O PLACE
    // [ALTERADO] Passando 'approach_from_front_' no final da chamada do Place
    MtcSharedLogic::addPlaceStages(
        task_, object_id_, place_pose_, attach_stage, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger(),
        approach_from_front_ // <--- REPASSANDO O PARÂMETRO PARA O PLACE
    );

    // 5. Return Home Final
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_ready_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator