#!/usr/bin/env python3
"""
Modified bale_exporter.py
Exports **two separate bales** when a reconstruction contains two clusters.
- No accumulation between bales.
- Optional outlier removal.
- Optional voxel downsampling.
- Optional Poisson mesh creation.
- If two bales are found, exports **A** and **B** independently.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d
import os
from datetime import datetime
import threading


class BaleExporter(Node):
    def __init__(self):
        super().__init__('bale_exporter')

        # --- Parameters ---
        self.declare_parameter('output_dir', '~/bale_scans')
        self.declare_parameter('voxel_size', 0.005)  # 0 to disable
        self.declare_parameter('remove_outliers', False)
        self.declare_parameter('nb_neighbors', 20)
        self.declare_parameter('std_ratio', 2.0)
        self.declare_parameter('export_ply', True)
        self.declare_parameter('create_poisson_mesh', False)

        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.output_dir = os.path.expanduser(output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.remove_outliers = self.get_parameter('remove_outliers').get_parameter_value().bool_value
        self.nb_neighbors = self.get_parameter('nb_neighbors').get_parameter_value().integer_value
        self.std_ratio = self.get_parameter('std_ratio').get_parameter_value().double_value
        self.export_ply = self.get_parameter('export_ply').get_parameter_value().bool_value
        self.create_poisson_mesh = self.get_parameter('create_poisson_mesh').get_parameter_value().bool_value

        self.current_cloud = None
        self.cloud_ready = False
        self.export_triggered = False
        self.lock = threading.Lock()

        self.create_subscription(PointCloud2, '/bale/reconstruction', self.cloud_callback, 10)
        self.create_subscription(Bool, '/bale/finished', self.finished_callback, 10)

        self.get_logger().info(f"BaleExporter ready. Output: {self.output_dir}")

    def cluster_bales(self, pcd):
        if len(pcd.points) == 0:
            return []

        labels = np.array(pcd.cluster_dbscan(eps=0.07, min_points=5, print_progress=False))

        if labels.max() < 0:
            self.get_logger().warn("No clusters found. Exporting whole cloud as Bale A only.")
            return [pcd]

        valid = labels[labels >= 0]
        counts = np.bincount(valid)

        if len(counts) == 1:
            self.get_logger().warn("Only one bale detected.")
            idx = np.where(labels == 0)[0]
            return [pcd.select_by_index(idx)]

        largest_two = counts.argsort()[-2:][::-1]
        self.get_logger().info(f"Two bales detected: clusters {largest_two}")

        bale_pcds = []
        for label in largest_two:
            idx = np.where(labels == label)[0]
            bale_pcds.append(pcd.select_by_index(idx))
            self.get_logger().info(f"Cluster {label}: {len(idx)} points")

        return bale_pcds

    def cloud_callback(self, msg):
        with self.lock:
            self.current_cloud = msg
            self.cloud_ready = True

    def finished_callback(self, msg):
        if not msg.data:
            return

        with self.lock:
            if self.export_triggered:
                self.get_logger().warn("Duplicate /bale/finished ignored.")
                return
            if not self.cloud_ready:
                self.get_logger().error("/bale/finished received but no cloud yet.")
                return

            cloud = self.current_cloud
            self.export_triggered = True

        threading.Thread(target=self.export_bales, args=(cloud,)).start()

    def export_bales(self, cloud_msg):
        try:
            arr = point_cloud2.read_points_numpy(cloud_msg, field_names=("x","y","z"), skip_nans=True)
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(arr))

            bales = self.cluster_bales(pcd)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            labels = ["A", "B"]

            for i, bale in enumerate(bales):
                tag = labels[i] if i < 2 else f"extra{i}"

                if self.remove_outliers and len(bale.points) > self.nb_neighbors:
                    _, ind = bale.remove_statistical_outlier(self.nb_neighbors, self.std_ratio)
                    bale = bale.select_by_index(ind)

                if self.voxel_size > 0:
                    bale = bale.voxel_down_sample(self.voxel_size)

                if len(bale.points) > 3:
                    bale.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

                ext = bale.get_axis_aligned_bounding_box().get_extent()
                vol = np.prod(ext)
                ctr = bale.get_center()
                self.get_logger().info(
                    f"Bale {tag}: {len(bale.points)} pts, Size=({ext[0]:.3f},{ext[1]:.3f},{ext[2]:.3f}), Vol={vol:.3f}, Ctr={ctr}"
                )

                base = f"bale_{timestamp}_{tag}"
                pcd_path = os.path.join(self.output_dir, f"{base}.pcd")
                ply_path = os.path.join(self.output_dir, f"{base}.ply")

                o3d.io.write_point_cloud(pcd_path, bale)
                self.get_logger().info(f"Saved: {pcd_path}")

                if self.export_ply:
                    o3d.io.write_point_cloud(ply_path, bale)
                    self.get_logger().info(f"Saved: {ply_path}")

        except Exception as e:
            self.get_logger().error(f"Export error: {e}")
        finally:
            with self.lock:
                self.current_cloud = None
                self.cloud_ready = False
                self.export_triggered = False
            self.get_logger().info("Export finished → ready for next bale.")


def main(args=None):
    rclpy.init(args=args)
    node = BaleExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
