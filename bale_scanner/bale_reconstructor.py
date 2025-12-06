#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Bool
import tf2_ros
import tf2_sensor_msgs
import numpy as np
from scipy.spatial import cKDTree
from typing import List, Optional


class BaleReconstructor(Node):
    """Reconstructs a 3D model of a moving bale using two synchronized lidars and conveyor motion compensation."""

    # Configuration constants
    CONVEYOR_VELOCITY = 0.3           # m/s along X-axis
    RIGHT_LIDAR_Y_OFFSET = 1.36128 + 1.36    # meters (lateral offset between lidars)
    SYNC_SLOP = 0.05                  # seconds - max time difference for approximate sync
    QUEUE_SIZE = 10
    SCAN_DURATION = 120
    DISTANCE_THRESHOLD = 0.10         # meters - points closer than this to static map are filtered

    def __init__(self):
        super().__init__('bale_reconstructor')

        # Subscribers (wrapped for message_filters)
        self.left_sub = Subscriber(self, PointCloud2, '/lidar/left')
        self.right_sub = Subscriber(self, PointCloud2, '/lidar/right')

        # Publisher for merged cloud (for static map builder)
        self.merged_pub = self.create_publisher(PointCloud2, '/lidar/merged', 10)

        # Subscriber for static map (received once from static map builder)
        self.static_map_sub = self.create_subscription(
            PointCloud2,
            '/static_map',
            self.static_map_callback,
            10
        )

        # Subscriber for static map ready signal
        self.static_ready_sub = self.create_subscription(
            Bool,
            '/static_map/ready',
            self.static_ready_callback,
            10
        )

        # Publisher to signal when bale is fully scanned
        self.done_pub = self.create_publisher(Bool, '/bale/finished', 10)
        self.bale_finished = False

        # Synchronized callback
        self.ts = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=self.QUEUE_SIZE,
            slop=self.SYNC_SLOP,
        )
        self.ts.registerCallback(self.lidars_callback)

        # Publisher for accumulated 3D bale point cloud
        self.pub = self.create_publisher(PointCloud2, '/bale/reconstruction', 10)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_buffer.clear()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.t0: Optional[float] = None
        self.start_time: Optional[float] = None
        self.accumulated_points: List[List[float]] = []
        
        # Static map for filtering
        self.static_kdtree: Optional[cKDTree] = None
        self.static_map_ready = False

        self.get_logger().info("Bale 3D Reconstructor Node Initialized")

    @staticmethod
    def structured_to_array(points) -> np.ndarray:
        """Convert PointCloud2 read_points generator output to float64 numpy array."""
        return np.array(
            [[p[0], p[1], p[2]] + (list(p[3:]) if len(p) > 3 else []) for p in points],
            dtype=np.float64
        )

    def static_map_callback(self, cloud_msg: PointCloud2):
        """Receive and build KD-tree from static map."""
        if self.static_kdtree is not None:
            return  # Already received

        static_points = self.structured_to_array(
            point_cloud2.read_points(cloud_msg, skip_nans=True)
        )

        if static_points.shape[0] > 0:
            self.static_kdtree = cKDTree(static_points[:, :3])
            self.get_logger().info(
                f"Static map received: {len(static_points):,} points. KD-tree built."
            )

    def static_ready_callback(self, msg: Bool):
        """Receive signal that static map is ready."""
        if msg.data:
            self.static_map_ready = True
            self.get_logger().info("Static map is ready for filtering")

    def filter_static_points(self, points: np.ndarray) -> np.ndarray:
        """Filter out static points using the static map KD-tree."""
        if points.shape[0] == 0:
            return points

        if not self.static_map_ready or self.static_kdtree is None:
            # Static map not ready yet, pass through all points
            return points

        # Query KD-tree to find nearest static point for each input point
        distances, _ = self.static_kdtree.query(points[:, :3], k=1, n_jobs=-1)
        
        # Keep only points far enough from static map (dynamic points)
        dynamic_mask = distances > self.DISTANCE_THRESHOLD
        return points[dynamic_mask]

    def lidars_callback(self, left_cloud: PointCloud2, right_cloud: PointCloud2):
        """Merge, filter, and reconstruct bale from synchronized lidar data."""
        try:
            left_stamp  = left_cloud.header.stamp
            right_stamp = right_cloud.header.stamp

            tf_left  = self.tf_buffer.lookup_transform(
                'map', left_cloud.header.frame_id, left_stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            tf_right = self.tf_buffer.lookup_transform(
                'map', right_cloud.header.frame_id, right_stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            left_map  = tf2_sensor_msgs.do_transform_cloud(left_cloud,  tf_left)
            right_map = tf2_sensor_msgs.do_transform_cloud(right_cloud, tf_right)

        except Exception as e:
            self.get_logger().warn(f"TF failed: {e}")
            return

        pts_left  = self.structured_to_array(point_cloud2.read_points(left_map,  skip_nans=True))
        pts_right = self.structured_to_array(point_cloud2.read_points(right_map, skip_nans=True))

        if pts_left.size == 0 and pts_right.size == 0:
            return

        t_left  = left_stamp.sec  + left_stamp.nanosec  * 1e-9
        t_right = right_stamp.sec + right_stamp.nanosec * 1e-9

        if self.t0 is None:
            self.t0 = t_left
            self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # === 1. Merge clouds in map frame WITHOUT motion compensation ===
        merged = []
        if pts_left.shape[0] > 0:
            merged.append(pts_left)
        if pts_right.shape[0] > 0:
            pts_r = pts_right.copy()
            pts_r[:, 1] += self.RIGHT_LIDAR_Y_OFFSET   # lateral offset
            pts_r[:, 0] -= self.RIGHT_LIDAR_Y_OFFSET
            merged.append(pts_r)

        if not merged:
            return
        
        all_points = np.vstack(merged)

        # Light ground filter
        all_points = all_points[all_points[:, 2] > 0.02]

        # Publish merged cloud (for static map builder during learning phase)
        merged_cloud = point_cloud2.create_cloud(
            header=left_map.header,
            fields=left_map.fields,
            points=all_points.tolist()
        )
        self.merged_pub.publish(merged_cloud)

        # === 2. Filter static points using received static map ===
        dynamic_points = self.filter_static_points(all_points)

        # === 3. Apply conveyor motion compensation to dynamic points ===
        if dynamic_points.shape[0] > 0 and self.static_map_ready:
            t_avg = (t_left + t_right) / 2.0
            dx = -self.CONVEYOR_VELOCITY * (t_avg - self.t0)
            dynamic_points[:, 0] += dx   # shift bale points backward

        # === 4. Accumulate and publish reconstruction ===
        if dynamic_points.shape[0] > 0:
            self.accumulated_points.extend(dynamic_points.tolist())

            out_cloud = point_cloud2.create_cloud(
                header=left_map.header,
                fields=left_map.fields,
                points=self.accumulated_points
            )
            self.pub.publish(out_cloud)

            filtered_count = all_points.shape[0] - dynamic_points.shape[0]
            self.get_logger().info(
                f"Bale cloud: {len(self.accumulated_points):,} pts "
                f"(+{len(dynamic_points)} new, {filtered_count} filtered)"
            )
        
        # === Time-based finished detection ===
        if not self.bale_finished and self.start_time is not None:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            elapsed = current_time - self.start_time
            if elapsed > self.SCAN_DURATION:
                self.bale_finished = True
                done_msg = Bool()
                done_msg.data = True
                self.done_pub.publish(done_msg)
                self.get_logger().info(f"Bale finished scanning (elapsed {elapsed:.1f}s)")


def main(args=None):
    rclpy.init(args=args)
    node = BaleReconstructor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down bale reconstructor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()