import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Vector3

class MoveGroupClient(Node):
    def __init__(self, group_name='tmr_arm'):
        super().__init__('moveit_client')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._group_name = group_name
        self._base_link = 'link_1'  # Ensure this matches your TF frames

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('MoveGroup Action Server not available, waiting...')

        self.get_logger().info('MoveGroup Action Client Ready!')

    def send_goal(self, target_pose, end_effector_link='link_6'):
        """ Send a goal to MoveGroup action server. """
        self._action_client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self._group_name
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.goal_constraints.append(self._create_pose_constraint(target_pose, end_effector_link))

        self._action_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)

    def _create_pose_constraint(self, pose, end_effector_link):
        """ Create position and orientation constraints. """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self._base_link
        pose_stamped.pose = pose

        constraints = Constraints()
        constraints.name = "goal_constraint"

        # Position Constraint
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = end_effector_link
        position_constraint.target_point_offset = Vector3(x=0.01, y=0.01, z=0.01)  # 0.01m tolerance

        # Define a bounding volume (sphere) for the position constraint
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]  # 2 cm tolerance
        bounding_volume.primitives.append(sphere)
        bounding_volume.primitive_poses.append(pose_stamped.pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        # Orientation Constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.absolute_x_axis_tolerance = 0.05  # Relaxed constraint
        orientation_constraint.absolute_y_axis_tolerance = 0.05
        orientation_constraint.absolute_z_axis_tolerance = 0.05
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def _goal_response_callback(self, future):
        """ Handle goal response. """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        goal_handle.get_result_async().add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """ Handle goal result. """
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
