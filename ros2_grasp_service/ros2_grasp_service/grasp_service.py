from rclpy import init, spin_once, shutdown, spin
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.node import Node
from std_srvs.srv import Empty
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from builtin_interfaces.msg import Duration
from time import sleep
from ros2_grasp_service.grasp_service_parameters import grasp_service_parameters


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
    def __init__(self, gripper_controller_name):
        super().__init__("gripper_action")
        self.action_client = ActionClient(self, GripperCommand, f"/{gripper_controller_name}/gripper_cmd")
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
        param_listener = grasp_service_parameters.ParamListener(self)
        self.params = param_listener.get_params()
        self.log_parameters()

        self.service = self.create_service(srv_type=Empty, srv_name="grasp_service", callback=self.grasp)
        self.joint_trajectory_action_client = FollowJointTrajectoryActionClient(self.params.joints_controller_name)
        self.gripper_action = GripperActionClient(self.params.gripper_controller_name)

    def log_parameters(self):
        self.get_logger().info(f"Got joints {self.params.joints_names}")
        self.get_logger().info(f"Got positions to target list {self.params.positions_to_target_list}")
        for pos_to_target in self.params.positions_to_target_list:
            self.get_logger().info(
                f"Got {pos_to_target} to target"
                f" {self.params.positions_to_target.get_entry(pos_to_target).positions} with time"
                f" {self.params.positions_to_target.get_entry(pos_to_target).time_to_target}"
            )

        self.get_logger().info(f"Got positions to home list {self.params.positions_to_home_list}")

        for pos_to_home in self.params.positions_to_home_list:
            self.get_logger().info(
                f"Got {pos_to_home} to home {self.params.positions_to_home.get_entry(pos_to_home).positions} with time"
                f" {self.params.positions_to_home.get_entry(pos_to_home).time_to_target}"
            )

        self.get_logger().info(f"Got time to wait on target {self.params.time_to_wait_on_target}")

        self.get_logger().info(f"Got joints_controller_name {self.params.joints_controller_name}")
        self.get_logger().info(f"Got gripper_controller_name {self.params.gripper_controller_name}")

        self.get_logger().info(f"Got gripper_close {self.params.gripper_close}")
        self.get_logger().info(f"Got gripper_open {self.params.gripper_open}")

    def get_time_for_position(self, list_of_position, map_of_position, target_position):
        time_to_target_postion = 0
        for position in list_of_position:
            time_to_target_postion += map_of_position.get_entry(position).time_to_target
            if position == target_position:
                return time_to_target_postion

        return time_to_target_postion

    def create_trajectory_goal(self, list_of_position, map_of_positions):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.params.joints_names
        trajectory.points = [
            JointTrajectoryPoint(
                positions=map_of_positions.get_entry(positions).positions,
                time_from_start=Duration(sec=self.get_time_for_position(list_of_position, map_of_positions, positions)),
            )
            for positions in list_of_position
        ]

        trajectory.points.append(
            JointTrajectoryPoint(
                positions=map_of_positions.get_entry(list_of_position[-1]).positions,
                time_from_start=Duration(
                    sec=self.get_time_for_position(list_of_position, map_of_positions, list_of_position[-1]), nanosec=1
                ),
            )
        )
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        return goal_msg

    def create_gripper_msg(self, gripper_value):
        msg = GripperCommand.Goal()
        msg.command.position = gripper_value

        return msg

    def go_to_pose(self, array_of_points, times_in_sec):
        self.joint_trajectory_action_client.send_goal(self.create_trajectory_goal(array_of_points, times_in_sec))
        while self.joint_trajectory_action_client.status != GoalStatus.STATUS_SUCCEEDED:
            spin_once(self.joint_trajectory_action_client)

    def set_gripper_postion(self, gripper_postion):
        self.gripper_action.send_goal(self.create_gripper_msg(gripper_postion))
        while self.gripper_action.status != GoalStatus.STATUS_SUCCEEDED:
            spin_once(self.gripper_action)

    def grasp(self, request, response):
        self.get_logger().info("Trigger received. Grasp sequence will be started.")

        self.go_to_pose(self.params.positions_to_target_list, self.params.positions_to_target)
        self.set_gripper_postion(self.params.gripper_close)

        sleep(self.params.time_to_wait_on_target)

        self.go_to_pose(self.params.positions_to_home_list, self.params.positions_to_home)
        self.set_gripper_postion(self.params.gripper_open)

        return response


def main(args=None):
    init(args=args)
    spin(GraspService())
    shutdown()


if __name__ == "__main__":
    main()
