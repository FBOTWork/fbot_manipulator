#include "fbot_manipulator/motion_primitives_openarm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace fbot_manipulator
{

MotionPrimitivesOpenArm::MotionPrimitivesOpenArm(const rclcpp::Node::SharedPtr& node, const std::string& arm_name)
    : MotionPrimitivesBase(node, arm_name)
{
    init();
}

MotionPrimitivesOpenArm::MotionPrimitivesOpenArm(const std::string& arm_name)
    : MotionPrimitivesBase(arm_name)
{
    init();
}

void MotionPrimitivesOpenArm::init()
{
    // Call the base class default initialization (MoveGroupInterface, action clients, etc.)
    MotionPrimitivesBase::init();

    // Try to obtain the package's share directory location
    std::string pkg_share;
    try {
        pkg_share = ament_index_cpp::get_package_share_directory("fbot_manipulator");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[OpenArm %s] Erro ao obter share directory: %s", arm_name_.c_str(), e.what());
        return;
    }

    // 1. Primary path using the group/arm name (e.g., config/left_arm/manipulator_config.yaml)
    std::string config_path = pkg_share + "/config/" + arm_name_ + "/manipulator_config.yaml";

    // 2. If the specific arm directory doesn't exist, use the "openarm" fallback
    if (!std::filesystem::exists(config_path)) {
        std::string fallback_path = pkg_share + "/config/openarm/manipulator_config.yaml";
        RCLCPP_WARN(node_->get_logger(),
                    "[OpenArm %s] Arquivo '%s' nao encontrado. Tentando fallback: '%s'",
                    arm_name_.c_str(), config_path.c_str(), fallback_path.c_str());
        config_path = fallback_path;
    }

    // 3. Check if the final file exists and load it
    if (std::filesystem::exists(config_path)) {
        try {
            manipulator_config_ = YAML::LoadFile(config_path);
            RCLCPP_INFO(node_->get_logger(),
                        "[OpenArm %s] Config YAML carregada com sucesso de: %s",
                        arm_name_.c_str(), config_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(),
                         "[OpenArm %s] Falha ao ler arquivo YAML '%s': %s",
                         arm_name_.c_str(), config_path.c_str(), e.what());
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(),
                     "[OpenArm %s] Arquivo YAML de configuracao nao encontrado em: %s",
                     arm_name_.c_str(), config_path.c_str());
    }
}

bool MotionPrimitivesOpenArm::moveToNamedTarget(const std::string& target_name)
{
    if (!manipulator_config_["poses"]) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[OpenArm %s] Chave 'poses' nao encontrada no arquivo YAML.", arm_name_.c_str());
        return false;
    }

    auto poses = manipulator_config_["poses"];
    if (!poses[target_name]) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[OpenArm %s] Posicao nomeada '%s' nao foi encontrada no YAML.",
                     arm_name_.c_str(), target_name.c_str());
        return false;
    }

    auto target_joint_positions = poses[target_name].as<std::vector<double>>();
    return moveToJointTarget(target_joint_positions);
}

bool MotionPrimitivesOpenArm::moveToJointTarget(const std::vector<double>& joint_positions)
{
    RCLCPP_INFO(node_->get_logger(),
                "[OpenArm %s] Planejando movimento para o alvo de juntas...", arm_name_.c_str());

    bool plan_success = planJointTarget(joint_positions);
    if (!plan_success) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[OpenArm %s] Falha no planejamento do movimento das juntas.", arm_name_.c_str());
        return false;
    }

    bool exec_success = executePath(true);
    if (!exec_success) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[OpenArm %s] Falha na execucao da trajetoria das juntas.", arm_name_.c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[OpenArm %s] Movimento de juntas executado com sucesso!", arm_name_.c_str());
    return true;
}

bool MotionPrimitivesOpenArm::moveToPose(const geometry_msgs::msg::Pose& pose)
{
    RCLCPP_INFO(node_->get_logger(),
                "[OpenArm %s] Planejando movimento Cartesiano (IK)...", arm_name_.c_str());

    bool plan_success = planPoseTarget(pose);
    if (!plan_success) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[OpenArm %s] Falha ao planejar trajetoria cartesiana.", arm_name_.c_str());
        return false;
    }

    bool exec_success = executePath(true);
    if (!exec_success) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[OpenArm %s] Falha ao executar trajetoria cartesiana.", arm_name_.c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[OpenArm %s] Pose atingida com sucesso!", arm_name_.c_str());
    return true;
}

bool MotionPrimitivesOpenArm::controlGripper(double position)
{
    return setGripperPosition(position);
}

} // namespace fbot_manipulator