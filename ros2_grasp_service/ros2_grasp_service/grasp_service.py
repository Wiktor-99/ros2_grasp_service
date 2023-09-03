from rclpy import init, spin, shutdown, Parameter, spin_once
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from builtin_interfaces.msg import Duration
from time import sleep

class FollowJointTrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('send_trajectory_service')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.status = GoalStatus.STATUS_EXECUTING

    def send_goal(self, goal_msg):
        self.status = GoalStatus.STATUS_EXECUTING
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.status = GoalStatus.STATUS_SUCCEEDED


class GraspService(Node):
    def __init__(self):
        super().__init__('grasp_service')
        self.service = self.create_service(Empty, 'grasp_service', self.grasp)
        self.joint_trajectory_action_client = FollowJointTrajectoryActionClient()
        self.declare_initial_parameters()
        self.get_initial_parameters()
        self.declare_nested_parameters()
        self.get_nested_parameters()
        self.log_parameters()

    def declare_nested_parameters(self):
        self.declar_times_and_postions_parameters_from_list(
            'times_for_postions_to_target',
            'positions_to_target',
            self.positions_to_target_list)
        self.declar_times_and_postions_parameters_from_list(
            'times_for_postions_to_home',
            'positions_to_home',
            self.positions_to_home_list)

    def get_nested_parameters(self):
        self.positions_to_target = self.get_nested_double_array_parameters(
            'positions_to_target',
            self.positions_to_target_list)

        self.times_for_postions_to_target = self.get_nested_double_parameters(
            'times_for_postions_to_target',
            self.positions_to_target_list)

        self.positions_to_home = self.get_nested_double_array_parameters(
            'positions_to_home',
            self.positions_to_home_list)

        self.times_for_postions_to_home = self.get_nested_double_parameters(
            'times_for_postions_to_home',
            self.positions_to_home_list)

    def get_nested_double_array_parameters(self, primary_key_name, nested_keys_names_list):
        return [ self.get_parameter(f'{primary_key_name}.{nested_key_name}').get_parameter_value().double_array_value
                 for nested_key_name
                 in nested_keys_names_list]

    def get_nested_double_parameters(self, primary_key_name, nested_keys_names_list):
        return [ self.get_parameter(f'{primary_key_name}.{nested_key_name}').get_parameter_value().integer_value
                 for nested_key_name
                 in nested_keys_names_list]


    def get_initial_parameters(self):
        self.joints_names = self.get_parameter('joints_names').get_parameter_value().string_array_value
        self.time_to_wait_on_target = self.get_parameter('time_to_wait_on_target').get_parameter_value().integer_value
        self.positions_to_target_list = self.get_parameter('positions_to_target_list').get_parameter_value().string_array_value
        self.positions_to_home_list = self.get_parameter('positions_to_home_list').get_parameter_value().string_array_value

    def declare_initial_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('joints_names', Parameter.Type.STRING_ARRAY),
                ('positions_to_target_list', Parameter.Type.STRING_ARRAY),
                ('positions_to_target_list', Parameter.Type.STRING_ARRAY),
                ('positions_to_home_list', Parameter.Type.STRING_ARRAY),
                ('time_to_wait_on_target', Parameter.Type.INTEGER)
            ])

    def log_parameters(self):
        self.get_logger().info(f'Got joint {self.joints_names}')
        self.get_logger().info(f'Got postions {self.positions_to_target_list}')
        self.get_logger().info(f'Got positions to target {self.positions_to_target}')
        self.get_logger().info(f'Got times for positions to target {self.times_for_postions_to_target}')
        self.get_logger().info(f'Got positions to home {self.positions_to_home}')
        self.get_logger().info(f'Got times for positions to home {self.times_for_postions_to_home}')
        self.get_logger().info(f'Got time to wait on target {self.time_to_wait_on_target}')

    def declar_times_and_postions_parameters_from_list(self, times_prefix, position_prefix, list_of_parameters):
        for position in list_of_parameters:
            self.get_logger().info(f'{position_prefix}.{position}')
            self.declare_parameter(f'{times_prefix}.{position}', Parameter.Type.INTEGER)
            self.declare_parameter(f'{position_prefix}.{position}', Parameter.Type.DOUBLE_ARRAY)

    def create_trajectory_goal(self, points, time_in_sec):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints_names
        result_points = []
        for i in range(len(points)):
            point = JointTrajectoryPoint()
            point.positions = points[i]
            point.time_from_start = Duration(sec=time_in_sec[i])
            result_points += [point]

        trajectory.points = result_points
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        return goal_msg

    def grasp(self, request, response):
        self.get_logger().info('Trigger received. Grasp sequence will be started.')

        self.joint_trajectory_action_client.send_goal(
            self.create_trajectory_goal(self.positions_to_target, self.times_for_postions_to_target))
        while self.joint_trajectory_action_client.status != GoalStatus.STATUS_SUCCEEDED:
            spin_once(self.joint_trajectory_action_client)

        sleep(self.time_to_wait_on_target)

        self.joint_trajectory_action_client.send_goal(
            self.create_trajectory_goal(self.positions_to_home, self.times_for_postions_to_home))
        while self.joint_trajectory_action_client.status != GoalStatus.STATUS_SUCCEEDED:
            spin_once(self.joint_trajectory_action_client)


def main(args=None):
    init(args=args)
    grasp_service = GraspService()
    spin(grasp_service, MultiThreadedExecutor())
    shutdown()


if __name__ == '__main__':
    main()