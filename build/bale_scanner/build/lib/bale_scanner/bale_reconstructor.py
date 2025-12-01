#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber

import tf2_ros
import tf2_sensor_msgs
import numpy as np
from scipy.spatial import cKDTree
from typing import List, Optional


class BaleReconstructor(Node):
    """Reconstructs a 3D model of a moving bale using two synchronized lidars and conveyor motion compensation."""

    # Configuration constants
    CONVEYOR_VELOCITY = 0.3           # m/s along X-axis
    RIGHT_LIDAR_Y_OFFSET = 1.36128    # meters (lateral offset between lidars)
    SYNC_SLOP = 0.05                  # seconds - max time difference for approximate sync
    QUEUE_SIZE = 10
    STATIC_LEARNING_TIME = 8           # seconds - time for learning the static map

    def __init__(self):
        super().__init__('bale_reconstructor')

        # Subscribers (wrapped for message_filters)
        self.left_sub = Subscriber(self, PointCloud2, '/lidar/left')
        self.right_sub = Subscriber(self, PointCloud2, '/lidar/right')

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
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.t0: Optional[float] = None
        self.accumulated_points: List[List[float]] = []
        self.static_points: Optional[np.ndarray] = None
        self.static_kdtree: Optional[cKDTree] = None

        self.get_logger().info("Bale 3D Reconstructor Node Initialized")

    @staticmethod
    def rotation_matrix(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> np.ndarray:
        """Return rotation matrix from Euler angles (Z-Y-X order)."""
        cos, sin = np.cos, np.sin
        Rx = np.array([[1, 0, 0],
                       [0, cos(roll), -sin(roll)],
                       [0, sin(roll), cos(roll)]])
        Ry = np.array([[cos(pitch), 0, sin(pitch)],
                       [0, 1, 0],
                       [-sin(pitch), 0, cos(pitch)]])
        Rz = np.array([[cos(yaw), -sin(yaw), 0],
                       [sin(yaw), cos(yaw), 0],
                       [0, 0, 1]])
        return Rz @ Ry @ Rx

    @staticmethod
    def structured_to_array(points) -> np.ndarray:
        """Convert PointCloud2 read_points generator output to float64 numpy array."""
        return np.array(
            [[p[0], p[1], p[2]] + (list(p[3:]) if len(p) > 3 else []) for p in points],
            dtype=np.float64
        )


    def filter_static_points(self, new_points: np.ndarray) -> np.ndarray:
        if new_points.shape[0] == 0:
            return new_points

        current_time = self.get_clock().now().nanoseconds * 1e-9
        elapsed = current_time - self.start_time

        # === PHASE 1: Learn static background (once at startup) ===
        if elapsed < self.STATIC_LEARNING_TIME:                                      # ← adjust if needed
            if not hasattr(self, 'static_kdtree') or self.static_kdtree is None:
                self.static_points = new_points.copy()
                self.static_kdtree = cKDTree(self.static_points[:, :3])
                self.get_logger().info("Static background learning started")
            else:
                self.static_points = np.vstack([self.static_points, new_points])
                # Gentle downsampling to keep tree fast
                if len(self.static_points) > 1_000_000:
                    self.static_points = self.static_points[::2]
                self.static_kdtree = cKDTree(self.static_points[:, :3])

            remaining = max(0, self.STATIC_LEARNING_TIME - elapsed)
            self.get_logger().info(f"Learning static map: {len(self.static_points):,} pts | {remaining:.1f}s left")
            return new_points                                           # ← no filtering yet

        # === PHASE 2: Frozen static map – real filtering ===
        if self.static_kdtree is None:
            self.get_logger().warn("No static map learned → passing all points")
            return new_points

        distances, _ = self.static_kdtree.query(new_points[:, :3], k=1, n_jobs=-1)
        dynamic_mask = distances > 0.10                                 # ← 10 cm works perfectly
        return new_points[dynamic_mask]

    @staticmethod
    def displace_and_rotate_points(points: List[list], dx: float, R: np.ndarray) -> List[list]:
        """Apply rotation then translation along X-axis to a list of points."""
        transformed = []
        for p in points:
            xyz = np.array(p[:3])
            xyz = R @ xyz           # Rotate
            xyz[0] += dx            # Translate along conveyor direction
            transformed.append(list(xyz) + p[3:])
        return transformed

    def lidars_callback(self, left_cloud: PointCloud2, right_cloud: PointCloud2):
        try:
            left_stamp  = left_cloud.header.stamp
            right_stamp = right_cloud.header.stamp

            tf_left  = self.tf_buffer.lookup_transform('map', left_cloud.header.frame_id,  left_stamp,
                                                    timeout=rclpy.duration.Duration(seconds=0.1))
            tf_right = self.tf_buffer.lookup_transform('map', right_cloud.header.frame_id, right_stamp,
                                                    timeout=rclpy.duration.Duration(seconds=0.1))

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

        # === 1. Merge clouds in map frame WITHOUT any motion compensation yet ===
        merged = []
        if pts_left.shape[0] > 0:
            merged.append(pts_left)
        if pts_right.shape[0] > 0:
            pts_r = pts_right.copy()
            pts_r[:, 1] += self.RIGHT_LIDAR_Y_OFFSET   # only lateral offset
            pts_r[:, 0] -= self.RIGHT_LIDAR_Y_OFFSET
            merged.append(pts_r)

        if not merged:
            return
        all_points = np.vstack(merged)

        # Light ground filter
        all_points = all_points[all_points[:, 2] > 0.02]

        # === 2. Static / dynamic separation (static map is always at t=0 coordinates) ===
        dynamic_points = self.filter_static_points(all_points)

        # === 3. ONLY NOW apply conveyor motion compensation to the dynamic (bale) points ===
        if dynamic_points.shape[0] > 0:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            elapsed_since_start = current_time - self.start_time

            if elapsed_since_start > self.STATIC_LEARNING_TIME:   # learning phase finished
                # Use average timestamp of the two clouds for compensation
                t_avg = (t_left + t_right) / 2.0
                dx = -self.CONVEYOR_VELOCITY * (t_avg - self.t0)
                dynamic_points[:, 0] += dx   # shift bale points backward

        # === 4. Accumulate & publish ===
        if dynamic_points.shape[0] > 0:
            self.accumulated_points.extend(dynamic_points.tolist())

            out_cloud = point_cloud2.create_cloud(
                header=left_map.header,
                fields=left_map.fields,
                points=self.accumulated_points
            )
            self.pub.publish(out_cloud)

            self.get_logger().info(f"Bale cloud: {len(self.accumulated_points):,} pts "
                                f"(+{len(dynamic_points)} new)")
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