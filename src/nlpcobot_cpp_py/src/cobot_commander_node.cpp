#include "rclcpp/rclcpp.hpp"
#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("cobot_commander_node");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> cobot_commander_node =
      rclcpp::Node::make_shared("cobot_commander_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(cobot_commander_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  const std::string PLANNING_GROUP = "tmr_arm";
  robot_model_loader::RobotModelLoader robot_model_loader(cobot_commander_node, "robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::vector<std::string> planner_plugin_names;

  if (!cobot_commander_node->get_parameter("ompl.planning_plugins", planner_plugin_names))
    RCLCPP_FATAL(LOGGER, "Could not find planner plugin names");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
      "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
  }

  if (planner_plugin_names.empty())
  {
    RCLCPP_ERROR(LOGGER,
      "No planner plugins defined. Please make sure that the planning_plugins parameter is not empty.");
    return -1;
  }

  const auto& planner_name = planner_plugin_names.at(0);
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_name));
    if (!planner_instance->initialize(robot_model, cobot_commander_node,
                                      cobot_commander_node->get_namespace()))
      RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
    RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (const auto& cls : classes)
      ss << cls << " ";
    RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_name.c_str(),
                ex.what(), ss.str().c_str());
  }

  moveit::planning_interface::MoveGroupInterface move_group(cobot_commander_node, PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(cobot_commander_node, "panda_link0",
                                                      "move_group_tutorial", move_group.getRobotModel());
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();  // clear all old markers
  visual_tools.trigger();

  /* Remote control is an introspection tool that allows users to step through a high level script
    via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

  /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
  visual_tools.trigger();

  /* We can also use visual_tools to wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the arm of the Panda
  // specifying the desired pose of the end-effector as input.
  visual_tools.trigger();
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function available
  // from the
  // :moveit_codedir:`kinematic_constraints<moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp>`
  // package.
  moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);

  // Define workspace bounds
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
      req.workspace_parameters.min_corner.z = -5.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
      req.workspace_parameters.max_corner.z = 5.0;

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this
  // planning context
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code);

  if (!context)
  {
    RCLCPP_ERROR(LOGGER, "Failed to create planning context");
    return -1;
  }
  context->solve(res);
  if (res.error_code.val != res.error_code.SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
    return -1;
  }

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
      cobot_commander_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
                                                                                              1);
  moveit_msgs::msg::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  moveit_msgs::msg::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher->publish(display_trajectory);

  /* Set the state in the planning scene to the final state of the last plan */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // Display the goal state
  visual_tools.publishAxisLabeled(pose.pose, "goal_1");
  visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* We can also use visual_tools to wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  rclcpp::shutdown();
  return 0;
}
