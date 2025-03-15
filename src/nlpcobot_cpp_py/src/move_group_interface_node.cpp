#include <memory>
#include <thread>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <nlpcobot_interfaces/action/move_robot.hpp>

class MoveGroupInterfaceNode : public rclcpp::Node
{
public:
  using MoveRobot = nlpcobot_interfaces::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

  explicit MoveGroupInterfaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("move_group_interface_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterfaceNode");
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<MoveRobot>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "move_robot",
        std::bind(&MoveGroupInterfaceNode::handle_goal, this, _1, _2),
        std::bind(&MoveGroupInterfaceNode::handle_cancel, this, _1),
        std::bind(&MoveGroupInterfaceNode::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "MoveGroupInterfaceNode Ready!");
  }

private:
  rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveRobot::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request: [%.2f, %.2f, %.2f]", goal->x, goal->y, goal->z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
  {

    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MoveGroupInterfaceNode::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveRobot::Feedback>();
    auto result = std::make_shared<MoveRobot::Result>();

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(shared_from_this(), "tmr_arm");

    // Set a target Pose
    auto const target_pose = [goal]
    {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;
      msg.position.x = goal->x;
      msg.position.y = goal->y;
      msg.position.z = goal->z;
      return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]
    {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    if (success)
    {
      // Execute the plan
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
      move_group_interface.execute(plan);
      result->success = true;
      goal_handle->succeed(result);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      result->success = false;
      goal_handle->abort(result);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveGroupInterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
