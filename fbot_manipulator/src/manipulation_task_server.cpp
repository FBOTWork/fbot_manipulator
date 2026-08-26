#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <fbot_manipulator_msgs/action/manipulation_task.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "fbot_manipulator/mtc/mtc_task.hpp"
#include "fbot_manipulator/mtc/mtc_pick_task.hpp"
#include "fbot_manipulator/mtc/mtc_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_pick_and_place_task.hpp"

namespace fbot_manipulator
{

using ManipulationTaskAction = fbot_manipulator_msgs::action::ManipulationTask;
using ExecuteTaskSolutionAction = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
using GoalHandle = rclcpp_action::ServerGoalHandle<ManipulationTaskAction>;
using ExecuteGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTaskSolutionAction>;

class MtcRvizTask : public MtcTask {
public:
    MtcRvizTask(rclcpp::Node::SharedPtr node) : MtcTask("rviz_task", node) {}
    bool buildTask() override { return true; } 
};

class ManipulationTaskServer : public rclcpp::Node
{
public:
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

        RCLCPP_INFO(get_logger(), "ManipulationTaskServer running");
    }

private:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ManipulationTaskAction::Goal>) {
        if (executing_) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; }
    void handleAccepted(const std::shared_ptr<GoalHandle> gh) { std::thread([this, gh]() { executeTask(gh); }).detach(); }

    rclcpp_action::GoalResponse handleExecuteGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteTaskSolutionAction::Goal>) {
        if (executing_) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleExecuteCancel(const std::shared_ptr<ExecuteGoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; }
    void handleExecuteAccepted(const std::shared_ptr<ExecuteGoalHandle> gh) { std::thread([this, gh]() { executeRvizTask(gh); }).detach(); }

    void executeTask(const std::shared_ptr<GoalHandle> goal_handle) {
        executing_ = true;
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ManipulationTaskAction::Result>();
        
        std::string arm_name = goal->arm_name.empty() ? "left_arm" : goal->arm_name;
        MtcTask::Ptr mtc_task;

        if (goal->task_type == ManipulationTaskAction::Goal::PICK) {
            mtc_task = std::make_shared<MtcPickTask>(shared_from_this(), goal->object_id); 
        } else if (goal->task_type == ManipulationTaskAction::Goal::PLACE) {
            mtc_task = hasGeometricPose(goal->place_pose) ? 
                std::make_shared<MtcPlaceTask>(shared_from_this(), goal->object_id, goal->place_pose) :
                std::make_shared<MtcPlaceTask>(shared_from_this(), goal->object_id, goal->place_pose_name); 
        } else if (goal->task_type == ManipulationTaskAction::Goal::PICK_AND_PLACE) {
            mtc_task = hasGeometricPose(goal->place_pose) ?
                std::make_shared<MtcPickAndPlaceTask>(shared_from_this(), goal->object_id, goal->place_pose) :
                std::make_shared<MtcPickAndPlaceTask>(shared_from_this(), goal->object_id, goal->place_pose_name); 
        } else {
            result->success = false; goal_handle->abort(result); executing_ = false; return;
        }

        mtc_task->loadConfigForArm(arm_name);

        if (goal->task_type != ManipulationTaskAction::Goal::PLACE) {
            mtc_task->addCollisionObject(goal->object_id, goal->object_pose, goal->object_size);
        }

        if (!mtc_task->buildTask() || !mtc_task->plan()) {
            result->success = false; goal_handle->abort(result); executing_ = false; return;
        }

        if (goal_handle->is_canceling()) {
            if (goal->task_type != ManipulationTaskAction::Goal::PLACE) mtc_task->removeCollisionObject(goal->object_id);
            result->success = false; goal_handle->canceled(result); executing_ = false; return;
        }

        result->success = mtc_task->execute();
        
        if (result->success) goal_handle->succeed(result);
        else goal_handle->abort(result);
        
        executing_ = false;
    }

    void executeRvizTask(const std::shared_ptr<ExecuteGoalHandle> goal_handle) {
        executing_ = true;
        auto result = std::make_shared<ExecuteTaskSolutionAction::Result>();
        
        auto rviz_task = std::make_shared<MtcRvizTask>(shared_from_this());
        
        std::string target_arm = "left_arm";
        for (const auto& sub : goal_handle->get_goal()->solution.sub_trajectory) {
            if (!sub.trajectory.joint_trajectory.joint_names.empty()) {
                if (sub.trajectory.joint_trajectory.joint_names[0].find("right") != std::string::npos) {
                    target_arm = "right_arm"; break;
                }
            }
        }
        
        rviz_task->loadConfigForArm(target_arm);
        bool success = rviz_task->executeSolution(goal_handle->get_goal()->solution);
        
        result->error_code.val = success ? moveit_msgs::msg::MoveItErrorCodes::SUCCESS : moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        
        if (success) goal_handle->succeed(result);
        else goal_handle->abort(result);
        
        executing_ = false;
    }

    static bool hasGeometricPose(const geometry_msgs::msg::Pose& p) {
        return !(p.position.x == 0.0 && p.position.y == 0.0 && p.position.z == 0.0 &&
                 p.orientation.x == 0.0 && p.orientation.y == 0.0 && p.orientation.z == 0.0 &&
                 (p.orientation.w == 0.0 || p.orientation.w == 1.0));
    }

    rclcpp_action::Server<ManipulationTaskAction>::SharedPtr action_server_;
    rclcpp_action::Server<ExecuteTaskSolutionAction>::SharedPtr execute_action_server_;
    bool executing_ = false;
};

} // namespace fbot_manipulator

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<fbot_manipulator::ManipulationTaskServer>(options));
    rclcpp::shutdown();
    return 0;
}