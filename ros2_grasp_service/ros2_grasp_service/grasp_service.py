from rclpy import init, spin, shutdown, Parameter
from rclpy.node import Node
from std_srvs.srv import Empty
from rclpy.executors import MultiThreadedExecutor


class GraspService(Node):
    def __init__(self):
        super().__init__('grasp_service')
        self.service = self.create_service(Empty, 'grasp_service', self.grasp)
        self.declare_parameter('joints_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('positions_to_target_list', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('positions_to_home_postion_list', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('time_to_wait_on_target', Parameter.Type.INTEGER)

        self.joints_names = self.get_parameter('joints_names').get_parameter_value().string_array_value
        self.time_to_wait_on_target = self.get_parameter('time_to_wait_on_target').get_parameter_value().integer_value
        self.positions_to_target_list = self.get_parameter('positions_to_target_list').get_parameter_value().string_array_value
        self.positions_to_home_postion_list = self.get_parameter('positions_to_home_postion_list').get_parameter_value().string_array_value

        self.declar_times_and_postions_parameters_from_list('times_for_postions_to_target', 'positions_to_target', self.positions_to_target_list)
        self.declar_times_and_postions_parameters_from_list('times_for_postions_to_home_postion', 'positions_to_home_postion', self.positions_to_home_postion_list)

        self.positions_to_target = [ self.get_parameter(f'positions_to_target.{position_to_target}').get_parameter_value().double_array_value for position_to_target in self.positions_to_target_list]
        self.times_for_postions_to_target = [ self.get_parameter(f'times_for_postions_to_target.{position_to_target}').get_parameter_value().double_value for position_to_target in self.positions_to_target_list]

        self.positions_to_home = [ self.get_parameter(f'positions_to_home_postion.{position_to_target}').get_parameter_value().double_array_value for position_to_target in self.positions_to_home_postion_list]
        self.times_for_postions_to_home = [ self.get_parameter(f'times_for_postions_to_home_postion.{position_to_target}').get_parameter_value().double_value for position_to_target in self.positions_to_home_postion_list]

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
            self.declare_parameter(f'{times_prefix}.{position}', Parameter.Type.DOUBLE)
            self.declare_parameter(f'{position_prefix}.{position}', Parameter.Type.DOUBLE_ARRAY)

    def grasp(self, request, response):
        self.get_logger().info('Trigger received. Grasp sequence will be started.')


def main(args=None):
    init(args=args)
    grasp_service = GraspService()
    spin(grasp_service, MultiThreadedExecutor())
    shutdown()


if __name__ == '__main__':
    main()