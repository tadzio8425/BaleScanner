import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Empty


class TriggerStaticService(Node):

    def __init__(self):
        super().__init__('trigger_static_service')

        self.service = self.create_service(
            Trigger,
            'trigger_static_once',
            self.handle_trigger
        )

        self.publisher = self.create_publisher(Empty, 'trigger_action', 10)

        self.get_logger().info('Trigger service ready.')

    def handle_trigger(self, request, response):
        self.publisher.publish(Empty())
        response.success = True
        response.message = 'Static mapping triggered!'
        self.get_logger().info('Trigger sent.')
        return response


def main():
    rclpy.init()
    node = TriggerStaticService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
