#pragma once

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcPickTask : public MtcTask
{
public:
    MtcPickTask(rclcpp::Node::SharedPtr node, 
        const std::string& object_id, 
        bool approach_from_front = false);

    bool buildTask() override;

private:
    std::string object_id_;
    bool approach_from_front_; 
};

} // namespace fbot_manipulator
