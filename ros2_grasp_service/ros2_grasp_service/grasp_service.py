from rclpy import init, spin, shutdown
from rclpy.node import Node
from std_srvs.srv import Empty
from rclpy.executors import MultiThreadedExecutor


class GraspService(Node):
    def __init__(self):
        super().__init__('grasp_service')
        self.srv = self.create_service(Empty, 'grasp_service', self.grasp)

    def grasp(self, request, response):
        self.get_logger().info('Trigger received. Grasp sequence will be started.')


def main(args=None):
    init(args=args)
    grasp_service = GraspService()
    spin(grasp_service, MultiThreadedExecutor())
    shutdown()


if __name__ == '__main__':
    main()