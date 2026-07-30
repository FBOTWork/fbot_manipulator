#pragma once

#include <cstdint>
#include <array>
#include <stdexcept>  // para std::out_of_range
#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcLoadCargoTask : public MtcTask
{
public:
    // Pose de destino já resolvida (uso "canônico": quem chama já sabe a pose).
    MtcLoadCargoTask(rclcpp::Node::SharedPtr node,
                      const std::string& object_id,
                         uint8_t cargo_index,
                         const geometry_msgs::msg::Pose& place_pose);

    // Conveniência: pose derivada automaticamente do índice do slot (0 a 4).
    MtcLoadCargoTask(rclcpp::Node::SharedPtr node,
                         const std::string& object_id,
                         uint8_t cargo_index);

    bool buildTask() override;

private:
    // Resolve a pose de destino para um slot de inventário. Lança std::out_of_range
    // se cargo_index estiver fora do intervalo válido.
    static geometry_msgs::msg::Pose poseForCargoIndex(uint8_t cargo_index);

    std::string object_id_;
    uint8_t cargo_index_;
    geometry_msgs::msg::Pose place_pose_;
};

} // namespace fbot_manipulator