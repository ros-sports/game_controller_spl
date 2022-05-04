import rclpy
from rclpy.node import Node


class GCSPL(Node):

    def __init__(self):
        super().__init__('gc_spl')

        self.data_port = self.get_parameter_or(
            'data_port', 3838).get_parameter_value().integer_value
        self.get_logger().info('data_port: "%s"' % self.data_port)

        self.return_port = self.get_parameter_or(
            'return_port', 3939).get_parameter_value().integer_value
        self.get_logger().info('return_port: "%s"' % self.return_port)


def main(args=None):
    rclpy.init(args=args)
    gc_spl = GCSPL()

    rclpy.spin(gc_spl)
    gc_spl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
