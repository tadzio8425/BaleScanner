#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Empty


class TriggerStaticService(Node):
    """Service to trigger static map learning on demand."""
    
    def __init__(self):
        super().__init__('trigger_static_service')
        
        # Create service
        self.service = self.create_service(
            Trigger,
            'trigger_static_once',
            self.handle_trigger
        )
        
        # Publisher to send trigger signal
        self.publisher = self.create_publisher(Empty, 'trigger_action', 10)
        
        self.get_logger().info('Trigger service ready at /trigger_static_once')
    
    def handle_trigger(self, request, response):
        """Handle incoming trigger request."""
        # Publish empty message to trigger static map learning
        self.publisher.publish(Empty())
        
        response.success = True
        response.message = 'Static mapping triggered!'
        self.get_logger().info('Static mapping trigger sent.')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TriggerStaticService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down trigger service...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()