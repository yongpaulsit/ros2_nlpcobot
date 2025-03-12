// Include
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>

// Define Action Message
#include <nlpcobot_interfaces/action/move_robot.hpp>

class CobotCommanderNode : public rclcpp::Node {
public:
    using MoveCommand = nlpcobot_interfaces::action::MoveRobot;
    using GoalHandleMoveCommand = rclcpp_action::ServerGoalHandle<MoveCommand>;

    CobotCommanderNode() : Node("cobot_commander_node") {
        // Initialize action server
        RCLCPP_INFO(this->get_logger(), "Initializing Cobot Commander Node");
        action_server_ = rclcpp_action::create_server<MoveCommand>(
            this,
            "move_robot",
            std::bind(&CobotCommanderNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CobotCommanderNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&CobotCommanderNode::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Cobot Commander Node Initialized");
    }

    void execute_xyz(double x, double y, double z) {
        RCLCPP_INFO(this->get_logger(), "Executing Goal");
        auto result = std::make_shared<MoveCommand::Result>();

        // Initialize MoveGroup Interface
        moveit::planning_interface::MoveGroupInterface move_group_interface(shared_from_this(), "tmr_arm");
        auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools(shared_from_this(), "base_link");
        moveit_visual_tools.deleteAllMarkers();
        moveit_visual_tools.loadRemoteControl();

        // Set target pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        move_group_interface.setPoseTarget(target_pose);

        // Plan to target pose
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, move_group_interface.getRobotModel()->getJointModelGroup("manipulator"));
            moveit_visual_tools.trigger();
            move_group_interface.execute(plan);
            RCLCPP_INFO(this->get_logger(), "Motion executed successfully");
            result->success = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
            result->success = false;
        }
    }

private:
    rclcpp_action::Server<MoveCommand>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &, 
        std::shared_ptr<const MoveCommand::Goal> goal
    ) {
        RCLCPP_INFO(
            this->get_logger(), 
            "Received goal: x=%.2f, y=%.2f, z=%.2f", goal->x, goal->y, goal->z
        );
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveCommand>) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveCommand> goal_handle) {
        execute(goal_handle);
    }

    void execute(const std::shared_ptr<GoalHandleMoveCommand> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing Goal");
        auto result = std::make_shared<MoveCommand::Result>();

        // Initialize MoveGroup Interface
        moveit::planning_interface::MoveGroupInterface move_group_interface(shared_from_this(), "tmr_arm");
        auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools(shared_from_this(), "base_link");
        moveit_visual_tools.deleteAllMarkers();
        moveit_visual_tools.loadRemoteControl();

        // Set target pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = goal_handle->get_goal()->x;
        target_pose.position.y = goal_handle->get_goal()->y;
        target_pose.position.z = goal_handle->get_goal()->z;
        move_group_interface.setPoseTarget(target_pose);

        // Plan to target pose
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, move_group_interface.getRobotModel()->getJointModelGroup("manipulator"));
            moveit_visual_tools.trigger();
            move_group_interface.execute(plan);
            RCLCPP_INFO(this->get_logger(), "Motion executed successfully");
            result->success = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
            result->success = false;
        }
        goal_handle->succeed(result);
    }
};

int main(int argc, char **argv) 
{
    // init ros2
    rclcpp::init(argc, argv);

    // create node object
    auto node = std::make_shared<CobotCommanderNode>();

    // Testing
    node->execute_xyz(1.0, 0.5, 1.5);

    // execute until shutdown
    rclcpp::spin(node);

    // shutdown
    rclcpp::shutdown();

    // exit program
    return 0;
}