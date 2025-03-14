#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <nlpcobot_interfaces/action/move_robot.hpp>

class MoveGroupInterfaceNode : public rclcpp::Node
{
public:
  MoveGroupInterfaceNode() : Node("move_group_interface_node")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Node");

    using MoveRobot = nlpcobot_interfaces::action::MoveRobot;
    using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

    explicit MoveGroupInterfaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("fibonacci_action_server", options)
    {
      using namespace std::placeholders;

      this->action_server_ = rclcpp_action::create_server<nlpcobot_interfaces::action::MoveRobot>(
          this,
          "move_robot",
          std::bind(&MoveGroupInterfaceNode::handle_goal, this, _1),
          std::bind(&MoveGroupInterfaceNode::handle_cancel, this, _1),
          std::bind(&MoveGroupInterfaceNode::handle_accepted, this, _1));
    }
  }

  void foo()
  {
    move_group_interface->setPoseTarget(generatePoseMessage(0.28, -0.2, 0.5));

    // Create a plan to that target pose
    auto const [success, plan] = planMotion();

    // Execute the plan
    if (success)
    {
      move_group_interface->execute(plan);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
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
    std::thread{std::bind(&MoveGroupInterfaceNode::execute_move, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute_move(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    // Create the MoveIt MoveGroup Interface
    auto move_group_interface = std::shared_ptr<moveit::planning_interface::MoveGroupInterface>(this->get_name(), "tmr_arm");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nlpcobot_interfaces::action::MoveRobot::Result>();
    auto feedback = std::make_shared<nlpcobot_interfaces::action::MoveRobot::Feedback>();

    // Set the target pose from the goal
    move_group_interface->setPoseTarget(generatePoseMessage(goal->x, goal->y, goal->z));

    // Plan the motion
    auto const [success, plan] = planMotion();

    if (success)
    {
      // Execute the plan
      move_group_interface->execute(plan);
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

  geometry_msgs::msg::Pose const generatePoseMessage(double x, double y, double z)
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    return msg;
  }

  std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan> planMotion()
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    bool success = static_cast<bool>(move_group_interface->plan(msg));
    return std::make_pair(success, msg);
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
