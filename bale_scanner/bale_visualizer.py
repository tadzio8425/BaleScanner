#!/usr/bin/env python3
"""
bale_exporter.py
Fixed version: NO accumulation between bales!
Exports ONLY the final point cloud when /bale/finished is received.
Clears state after each bale → clean separation.
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

        # Parameters
        self.declare_parameter('output_dir', '~/bale_scans')
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('remove_outliers', True)
        self.declare_parameter('nb_neighbors', 20)
        self.declare_parameter('std_ratio', 2.0)
        self.declare_parameter('export_ply', True)

        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.output_dir = os.path.expanduser(output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.remove_outliers = self.get_parameter('remove_outliers').get_parameter_value().bool_value
        self.nb_neighbors = self.get_parameter('nb_neighbors').get_parameter_value().integer_value
        self.std_ratio = self.get_parameter('std_ratio').get_parameter_value().double_value
        self.export_ply = self.get_parameter('export_ply').get_parameter_value().bool_value

        # --- Critical fix: use a fresh cloud for each bale ---
        self.current_cloud: PointCloud2 | None = None
        self.cloud_ready = False  # Becomes True when we have at least one cloud
        self.export_triggered = False
        self.lock = threading.Lock()

        # Subscriptions
        self.create_subscription(
            PointCloud2,
            '/bale/reconstruction',
            self.cloud_callback,
            10
        )
        self.create_subscription(
            Bool,
            '/bale/finished',
            self.finished_callback,
            10
        )

        self.get_logger().info(f"BaleExporter ready. Output: {self.output_dir}")

    def cloud_callback(self, msg: PointCloud2):
        with self.lock:
            self.current_cloud = msg
            self.cloud_ready = True

            # Optional throttled logging
            try:
                points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
                self.get_logger().info(
                    f"Updated bale cloud: {len(points):,} points",
                    throttle_duration_sec=3.0
                )
            except:
                pass

    def finished_callback(self, msg: Bool):
        if not msg.data:
            return

        with self.lock:
            if self.export_triggered:
                self.get_logger().warn("Export already in progress or done for this bale. Ignoring duplicate /bale/finished.")
                return
            if not self.cloud_ready or self.current_cloud is None:
                self.get_logger().error("Bale finished but no cloud received yet!")
                return

            # Capture the cloud AT THE MOMENT of finish → safe from future updates
            cloud_to_export = self.current_cloud
            self.export_triggered = True

        self.get_logger().info("Bale finished → exporting in background thread...")
        thread = threading.Thread(target=self.export_bale, args=(cloud_to_export,))
        thread.start()

 #!/usr/bin/env python3
"""
bale_exporter.py
Fixed version: NO accumulation between bales!
Exports ONLY the final point cloud when /bale/finished is received.
Clears state after each bale → clean separation.
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

        # Parameters
        self.declare_parameter('output_dir', '~/bale_scans')
        self.declare_parameter('voxel_size', 0.01)
        self.declare_parameter('remove_outliers', True)
        self.declare_parameter('nb_neighbors', 20)
        self.declare_parameter('std_ratio', 2.0)
        self.declare_parameter('export_ply', True)

        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.output_dir = os.path.expanduser(output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.remove_outliers = self.get_parameter('remove_outliers').get_parameter_value().bool_value
        self.nb_neighbors = self.get_parameter('nb_neighbors').get_parameter_value().integer_value
        self.std_ratio = self.get_parameter('std_ratio').get_parameter_value().double_value
        self.export_ply = self.get_parameter('export_ply').get_parameter_value().bool_value

        # --- Critical fix: use a fresh cloud for each bale ---
        self.current_cloud: PointCloud2 | None = None
        self.cloud_ready = False  # Becomes True when we have at least one cloud
        self.export_triggered = False
        self.lock = threading.Lock()

        # Subscriptions
        self.create_subscription(
            PointCloud2,
            '/bale/reconstruction',
            self.cloud_callback,
            10
        )
        self.create_subscription(
            Bool,
            '/bale/finished',
            self.finished_callback,
            10
        )

        self.get_logger().info(f"BaleExporter ready. Output: {self.output_dir}")

    def cloud_callback(self, msg: PointCloud2):
        with self.lock:
            self.current_cloud = msg
            self.cloud_ready = True

            # Optional throttled logging
            try:
                points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
                self.get_logger().info(
                    f"Updated bale cloud: {len(points):,} points",
                    throttle_duration_sec=3.0
                )
            except:
                pass

    def finished_callback(self, msg: Bool):
        if not msg.data:
            return

        with self.lock:
            if self.export_triggered:
                self.get_logger().warn("Export already in progress or done for this bale. Ignoring duplicate /bale/finished.")
                return
            if not self.cloud_ready or self.current_cloud is None:
                self.get_logger().error("Bale finished but no cloud received yet!")
                return

            # Capture the cloud AT THE MOMENT of finish → safe from future updates
            cloud_to_export = self.current_cloud
            self.export_triggered = True

        self.get_logger().info("Bale finished → exporting in background thread...")
        thread = threading.Thread(target=self.export_bale, args=(cloud_to_export,))
        thread.start()

    def export_bale(self, cloud_msg: PointCloud2):
        try:
            # --- 1. Safely read all points as numpy array ---
            points_np = point_cloud2.read_points_numpy(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            if points_np.size == 0:
                self.get_logger().warn("Received empty cloud. Skipping export.")
                return

            xyz = np.array(points_np, dtype=np.float64)  # ensures Nx3
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)

            self.get_logger().info(f"Raw final bale: {len(xyz):,} points")

            # --- 2. Outlier removal (optional) ---
            if self.remove_outliers and len(xyz) > self.nb_neighbors:
                cl, ind = pcd.remove_statistical_outlier(
                    nb_neighbors=self.nb_neighbors,
                    std_ratio=self.std_ratio
                )
                removed_points = len(pcd.points) - len(ind)
                pcd = pcd.select_by_index(ind)
                self.get_logger().info(f"After outlier removal: {len(pcd.points):,} points "
                                    f"({removed_points:,} points removed)")

            # --- 3. Voxel downsampling (safe small voxel) ---
            if self.voxel_size > 0:
                pcd = pcd.voxel_down_sample(self.voxel_size)
                self.get_logger().info(f"After {self.voxel_size*100:.1f}cm downsampling: {len(pcd.points):,} points")

            # --- 4. Estimate normals for proper PLY export ---
            if len(pcd.points) > 3:
                pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
                pcd.orient_normals_consistent_tangent_plane(30)

            # --- 5. Save PCD and PLY ---
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_name = f"bale_{timestamp}"
            pcd_path = os.path.join(self.output_dir, f"{base_name}.pcd")
            ply_path = os.path.join(self.output_dir, f"{base_name}.ply")

            o3d.io.write_point_cloud(pcd_path, pcd, write_ascii=False)
            self.get_logger().info(f"Saved PCD: {pcd_path}")

            if self.export_ply:
                if not pcd.has_colors():
                    pcd.paint_uniform_color([0.8, 0.7, 0.2])  # uniform bale color
                o3d.io.write_point_cloud(ply_path, pcd)
                self.get_logger().info(f"Saved PLY: {ply_path}")

            # --- 6. Optional: create Poisson mesh for real faces ---
            # Uncomment if you want a mesh with faces
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
            mesh_path = os.path.join(self.output_dir, f"{base_name}_mesh.ply")
            o3d.io.write_triangle_mesh(mesh_path, mesh)
            self.get_logger().info(f"Saved Poisson mesh: {mesh_path}")

        except Exception as e:
            self.get_logger().error(f"Export failed: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # --- 7. Reset state for next bale ---
            with self.lock:
                self.current_cloud = None
                self.cloud_ready = False
                self.export_triggered = False

            self.get_logger().info("Export finished and state reset for next bale.")

def main(args=None):
    rclpy.init(args=args)
    node = BaleExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down bale_exporter...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

def main(args=None):
    rclpy.init(args=args)
    node = BaleExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down bale_exporter...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()