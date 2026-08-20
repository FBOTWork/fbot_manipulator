#pragma once

#include "fbot_manipulator/motion_primitives_base.hpp"

namespace fbot_manipulator
{

/**
 * @brief Motion primitives implementation for OpenArm (single or bimanual arms).
 */
class MotionPrimitivesOpenArm : public MotionPrimitivesBase
{
public:
    MotionPrimitivesOpenArm(const rclcpp::Node::SharedPtr& node, const std::string& arm_name);
    MotionPrimitivesOpenArm(const std::string& arm_name);

    ~MotionPrimitivesOpenArm() override = default;

    /**
     * @brief Move to a named joint configuration defined in YAML (e.g., "home", "ready")
     */
    bool moveToNamedTarget(const std::string& target_name) override;

    /**
     * @brief Move to explicit joint angles
     */
    bool moveToJointTarget(const std::vector<double>& joint_positions) override;

    /**
     * @brief Move end-effector to a Cartesian Pose
     */
    bool moveToPose(const geometry_msgs::msg::Pose& pose) override;

    /**
     * @brief Auxiliary method for controlling the OpenArm gripper
     */
    bool controlGripper(double position);
private:
    void init();
};

} // namespace fbot_manipulator
