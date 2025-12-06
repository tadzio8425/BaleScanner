#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty
import numpy as np
from scipy.spatial import cKDTree
from std_msgs.msg import Bool
from typing import Optional


class StaticMapBuilder(Node):
    """Builds a static background map from merged lidar data during initial learning phase."""

    # Configuration constants
    STATIC_LEARNING_TIME = 8.0  # seconds - time for learning the static map

    def __init__(self):
        super().__init__('static_map_builder')

        # Subscriber for merged point cloud
        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar/merged',
            self.cloud_callback,
            10
        )

        # Subscriber for trigger signal
        self.trigger_sub = self.create_subscription(
            Empty,
            'trigger_action',
            self.trigger_callback,
            10
        )

        # Publisher for static map (publishes once when complete)
        self.pub = self.create_publisher(PointCloud2, '/static_map', 10)
        
        # Publisher to signal map is ready
        self.ready_pub = self.create_publisher(Bool, '/static_map/ready', 10)

        # State
        self.start_time: Optional[float] = None
        self.static_points: Optional[np.ndarray] = None
        self.map_published = False
        self.triggered = False
        self.is_learning = False

        self.get_logger().info("Static Map Builder Node Initialized - Waiting for trigger...")

    @staticmethod
    def structured_to_array(points) -> np.ndarray:
        """Convert PointCloud2 read_points generator output to float64 numpy array."""
        return np.array(
            [[p[0], p[1], p[2]] + (list(p[3:]) if len(p) > 3 else []) for p in points],
            dtype=np.float64
        )

    def trigger_callback(self, msg):
        """Start static map learning when triggered."""
        if not self.triggered and not self.map_published:
            self.triggered = True
            self.is_learning = True
            self.start_time = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info("Triggered! Starting static map learning...")

    def cloud_callback(self, cloud_msg: PointCloud2):
        """Accumulate points during learning phase, then publish static map."""
        if self.map_published:
            return  # Already done, ignore further messages

        if not self.triggered:
            return  # Wait for trigger before starting

        if not self.is_learning:
            return  # Not in learning phase

        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds * 1e-9

        current_time = self.get_clock().now().nanoseconds * 1e-9
        elapsed = current_time - self.start_time

        # Read points from message
        points = self.structured_to_array(
            point_cloud2.read_points(cloud_msg, skip_nans=True)
        )

        if points.shape[0] == 0:
            return

        # === Accumulate points during learning phase ===
        if elapsed < self.STATIC_LEARNING_TIME:
            if self.static_points is None:
                self.static_points = points.copy()
            else:
                self.static_points = np.vstack([self.static_points, points])
                # Gentle downsampling to keep memory manageable
                if len(self.static_points) > 1_000_000:
                    self.static_points = self.static_points[::2]

            remaining = max(0, self.STATIC_LEARNING_TIME - elapsed)
            self.get_logger().info(
                f"Learning static map: {len(self.static_points):,} pts | {remaining:.1f}s left"
            )
            return

        # === Learning complete - publish static map once ===
        if self.static_points is not None and not self.map_published:
            self.get_logger().info(
                f"Static learning complete! Publishing map with {len(self.static_points):,} points"
            )
            
            # Publish the static map
            static_cloud = point_cloud2.create_cloud(
                header=cloud_msg.header,
                fields=cloud_msg.fields,
                points=self.static_points.tolist()
            )
            self.pub.publish(static_cloud)
            
            # Signal that map is ready
            ready_msg = Bool()
            ready_msg.data = True
            self.ready_pub.publish(ready_msg)
            
            self.map_published = True
            self.get_logger().info("Static map published successfully")


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down static map builder...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()