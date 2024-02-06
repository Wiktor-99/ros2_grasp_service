from rclpy import init, spin_once, shutdown, spin
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from builtin_interfaces.msg import Duration
from time import sleep


class FollowJointTrajectoryActionClient(Node):
    def __init__(self, joints_controller_name):
        super().__init__("send_trajectory_action")
        self.action_client = ActionClient(
            self, FollowJointTrajectory, f"/{joints_controller_name}/follow_joint_trajectory"
        )
        self.status = GoalStatus.STATUS_EXECUTING

    def send_goal(self, goal_msg):
        self.status = GoalStatus.STATUS_EXECUTING
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.get_logger().info(f"Goal accepted {goal_handle.accepted}")
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Call back with status {result}")
        self.status = GoalStatus.STATUS_SUCCEEDED


class GripperActionClient(Node):
    def __init__(self):
        super().__init__("gripper_action")
        self.action_client = ActionClient(self, GripperCommand, "/gripper_action_controller/gripper_cmd")
        self.status = GoalStatus.STATUS_EXECUTING

    def send_goal(self, goal_msg):
        self.status = GoalStatus.STATUS_EXECUTING
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.get_logger().info(f"Goal accepted {goal_handle.accepted}")
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Call back with status {result}")
        self.status = GoalStatus.STATUS_SUCCEEDED


class GraspService(Node):
    def __init__(self):
        super().__init__("grasp_service")
        self.declare_initial_parameters()
        self.get_initial_parameters()
        self.declare_nested_parameters()
        self.get_nested_parameters()
        self.log_parameters()

        self.service = self.create_service(srv_type=Empty, srv_name="grasp_service", callback=self.grasp)
        self.joint_trajectory_action_client = FollowJointTrajectoryActionClient(self.joints_controller_name)
        self.gripper_action = GripperActionClient()

    def declare_nested_parameters(self):
        self.declar_times_and_positions_parameters_from_list(
            "times_for_positions_to_target", "positions_to_target", self.positions_to_target_list
        )
        self.declar_times_and_positions_parameters_from_list(
            "times_for_positions_to_home", "positions_to_home", self.positions_to_home_list
        )

    def get_nested_parameters(self):
        self.positions_to_target = self.get_nested_double_array_parameters(
            "positions_to_target", self.positions_to_target_list
        )

        self.times_for_positions_to_target = self.get_nested_double_parameters(
            "times_for_positions_to_target", self.positions_to_target_list
        )

        self.positions_to_home = self.get_nested_double_array_parameters(
            "positions_to_home", self.positions_to_home_list
        )

        self.times_for_positions_to_home = self.get_nested_double_parameters(
            "times_for_positions_to_home", self.positions_to_home_list
        )

    def get_nested_double_array_parameters(self, primary_key_name, nested_keys_names_list):
        return [
            self.get_parameter(f"{primary_key_name}.{nested_key_name}").get_parameter_value().double_array_value
            for nested_key_name in nested_keys_names_list
        ]

    def get_nested_double_parameters(self, primary_key_name, nested_keys_names_list):
        return [
            self.get_parameter(f"{primary_key_name}.{nested_key_name}").get_parameter_value().integer_value
            for nested_key_name in nested_keys_names_list
        ]

    def get_initial_parameters(self):
        self.joints_controller_name = self.get_parameter("joints_controller_name").get_parameter_value().string_value
        self.joints_names = self.get_parameter("joints_names").get_parameter_value().string_array_value
        self.time_to_wait_on_target = self.get_parameter("time_to_wait_on_target").get_parameter_value().integer_value
        self.positions_to_target_list = (
            self.get_parameter("positions_to_target_list").get_parameter_value().string_array_value
        )
        self.positions_to_home_list = (
            self.get_parameter("positions_to_home_list").get_parameter_value().string_array_value
        )
        self.gripper_close_position = self.get_parameter("gripper_close").get_parameter_value().double_value
        self.gripper_open_position = self.get_parameter("gripper_open").get_parameter_value().double_value

    def declare_initial_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("joints_controller_name", Parameter.Type.STRING),
                ("joints_names", Parameter.Type.STRING_ARRAY),
                ("positions_to_target_list", Parameter.Type.STRING_ARRAY),
                ("positions_to_target_list", Parameter.Type.STRING_ARRAY),
                ("positions_to_home_list", Parameter.Type.STRING_ARRAY),
                ("time_to_wait_on_target", Parameter.Type.INTEGER),
                ("gripper_close", Parameter.Type.DOUBLE),
                ("gripper_open", Parameter.Type.DOUBLE),
            ],
        )

    def log_parameters(self):
        self.get_logger().info(f"Got joint {self.joints_names}")
        self.get_logger().info(f"Got positions {self.positions_to_target_list}")
        self.get_logger().info(f"Got positions to target {self.positions_to_target}")
        self.get_logger().info(f"Got times for positions to target {self.times_for_positions_to_target}")
        self.get_logger().info(f"Got positions to home {self.positions_to_home}")
        self.get_logger().info(f"Got times for positions to home {self.times_for_positions_to_home}")
        self.get_logger().info(f"Got time to wait on target {self.time_to_wait_on_target}")

    def declar_times_and_positions_parameters_from_list(self, times_prefix, position_prefix, list_of_parameters):
        for position in list_of_parameters:
            self.get_logger().info(f"{position_prefix}.{position}")
            self.declare_parameter(f"{times_prefix}.{position}", Parameter.Type.INTEGER)
            self.declare_parameter(f"{position_prefix}.{position}", Parameter.Type.DOUBLE_ARRAY)

    def create_trajectory_goal(self, array_of_points, times_in_sec):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints_names
        trajectory.points = [
            JointTrajectoryPoint(positions=points, time_from_start=Duration(sec=time_from_sec))
            for points, time_from_sec in zip(array_of_points, times_in_sec)
        ]

        trajectory.points.append(
            JointTrajectoryPoint(
                positions=array_of_points[-1], time_from_start=Duration(sec=times_in_sec[-1], nanosec=1)
            )
        )
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        return goal_msg

    def create_gripper_msg(self, gripper_value):
        msg = GripperCommand.Goal()
        msg.command.position = gripper_value

        return msg

    def grasp(self, request, response):
        self.get_logger().info("Trigger received. Grasp sequence will be started.")

        self.joint_trajectory_action_client.send_goal(
            self.create_trajectory_goal(self.positions_to_target, self.times_for_positions_to_target)
        )
        while self.joint_trajectory_action_client.status != GoalStatus.STATUS_SUCCEEDED:
            spin_once(self.joint_trajectory_action_client)

        self.gripper_action.send_goal(self.create_gripper_msg(self.gripper_close_position))
        while self.gripper_action.status != GoalStatus.STATUS_SUCCEEDED:
            spin_once(self.gripper_action)

        sleep(self.time_to_wait_on_target)

        self.joint_trajectory_action_client.send_goal(
            self.create_trajectory_goal(self.positions_to_home, self.times_for_positions_to_home)
        )
        while self.joint_trajectory_action_client.status != GoalStatus.STATUS_SUCCEEDED:
            spin_once(self.joint_trajectory_action_client)

        self.gripper_action.send_goal(self.create_gripper_msg(self.gripper_open_position))
        while self.gripper_action.status != GoalStatus.STATUS_SUCCEEDED:
            spin_once(self.gripper_action)

        return response


def main(args=None):
    init(args=args)
    spin(GraspService())
    shutdown()


if __name__ == "__main__":
    main()
