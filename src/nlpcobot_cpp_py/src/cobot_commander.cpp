// Include
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>

// Define Action Message
#include <nlpcobot_interfaces/action/move_robot.hpp>

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_cpp_node") {
        static const rclcpp::Logger LOGGER = this->get_logger();

        RCLCPP_INFO(LOGGER, "Initializing Node");

        const std::string PLANNING_GROUP = "panda_arm";
        robot_model_loader::RobotModelLoader robot_model_loader(motion_planning_api_tutorial_node, "robot_description");
        const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
        
        /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
        const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

        planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

        planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

        std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
        planning_interface::PlannerManagerPtr planner_instance;
        std::string planner_plugin_name;

        if (!motion_planning_api_tutorial_node->get_parameter("planning_plugin", planner_plugin_name))
            RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
        try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
        }
        catch (pluginlib::PluginlibException& ex) {
        RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
        }
        try {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, motion_planning_api_tutorial_node,
                                            motion_planning_api_tutorial_node->get_namespace()))
            RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
        RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
        }
        catch (pluginlib::PluginlibException& ex) {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes)
            ss << cls << " ";
        RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                    ex.what(), ss.str().c_str());
        }

        moveit::planning_interface::MoveGroupInterface move_group(motion_planning_api_tutorial_node, PLANNING_GROUP);

        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools(motion_planning_api_tutorial_node, "panda_link0",
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

        visual_tools.trigger();
        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "panda_link0";
        pose.pose.position.x = 0.3;
        pose.pose.position.y = 0.4;
        pose.pose.position.z = 0.75;
        pose.pose.orientation.w = 1.0;

        std::vector<double> tolerance_pose(3, 0.01);
        std::vector<double> tolerance_angle(3, 0.01);

        moveit_msgs::msg::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

        req.group_name = PLANNING_GROUP;
        req.goal_constraints.push_back(pose_goal);

        planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
        context->solve(res);
        if (res.error_code_.val != res.error_code_.SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
        return 0;
        }
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

#ifdef __hi__
class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_cpp_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Node");

        static const std::string PLANNING_GROUP = "panda_arm";
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
        moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

        RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
        std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
        std::ostream_iterator<std::string>(std::cout, ", "));

        geometry_msgs::msg::Pose target_pose1;
        target_pose1.orientation.w = 1.0;
        target_pose1.position.x = 0.28;
        target_pose1.position.y = -0.2;
        target_pose1.position.z = 0.5;
        move_group.setPoseTarget(target_pose1);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
        move_group.move();
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

#endif
#ifdef __FORREAL__
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
        // auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools(shared_from_this(), "base_link");
        // moveit_visual_tools.deleteAllMarkers();
        // moveit_visual_tools.loadRemoteControl();

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
            // moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, move_group_interface.getRobotModel()->getJointModelGroup("manipulator"));
            // moveit_visual_tools.trigger();
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

#endif