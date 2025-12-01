#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import open3d as o3d
import numpy as np
import threading
import time

class BaleMeshVisualizer(Node):
    def __init__(self):
        super().__init__('bale_visualizer')

        self.sub = self.create_subscription(
            PointCloud2, '/bale/reconstruction', self.callback, 10)

        # Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("Real-time Bale Mesh + Metrology", width=1400, height=900)

        self.pcd = o3d.geometry.PointCloud()
        self.mesh = o3d.geometry.TriangleMesh()
        self.mesh_wire = o3d.geometry.LineSet()
        self.texts = []  # for on-screen labels

        self.first = True
        self.last_reconstruction = 0.0
        self.reconstruction_interval = 1.5  # seconds between mesh updates

        # Visual options
        opt = self.vis.get_render_option()
        opt.background_color = np.array([0.1, 0.1, 0.15])
        opt.point_size = 2.0
        opt.mesh_show_back_face = True
        opt.mesh_show_wireframe = True

        self.get_logger().info("Real-time Bale Mesh Visualizer READY!")

    def add_text(self, text, pos=(20, 20), color=(1, 1, 0)):
        if len(self.texts) > 10:
            self.vis.remove_geometry(self.texts.pop(0), reset_bounding_box=False)
        txt = o3d.geometry.Image(np.zeros((1,1,3)))  # dummy
        label = o3d.visualization.gui.Label(text)
        label.text_color = o3d.visualization.gui.Color(color[0], color[1], color[2])
        label.position = o3d.visualization.gui.Point2i(pos[0], pos[1])
        self.vis.add_geometry(label, reset_bounding_box=False)
        self.texts.append(label)

    def estimate_dimensions(self, points):
        if len(points) < 100:
            return None

        # Axis-Aligned Bounding Box
        aabb = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
            o3d.utility.Vector3dVector(points))
        min_b = aabb.min_bound
        max_b = aabb.max_bound

        L = max_b[0] - min_b[0]  # along conveyor
        W = max_b[1] - min_b[1]  # lateral
        H = max_b[2] - min_b[2]  # height

        volume_m3 = aabb.volume()
        volume_liters = volume_m3 * 1000

        return {
            "length_m": L,
            "width_m": W,
            "height_m": H,
            "volume_m3": volume_m3,
            "volume_liters": volume_liters,
            "points": len(points)
        }

    def reconstruct_mesh(self, points_np):
        # Downsample for speed
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        # Poisson with good defaults for bales
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9, width=0, scale=1.1, linear_fit=False)

        # Remove low-density vertices (noise)
        densities = np.asarray(densities)
        density_threshold = np.percentile(densities, 15)
        vertices_to_remove = densities < density_threshold
        mesh.remove_vertices_by_mask(vertices_to_remove)

        # Smooth a little
        mesh = mesh.filter_smooth_laplacian(number_of_iterations=2, lambda_filter=0.5)

        # Color the mesh (height gradient)
        verts = np.asarray(mesh.vertices)
        z = (verts[:, 2] - verts[:, 2].min()) / (verts[:, 2].ptp() + 1e-6)
        colors = np.zeros((len(verts), 3))
        colors[:, 0] = z * 0.8 + 0.2          # red high
        colors[:, 2] = (1 - z) * 0.8 + 0.2     # blue low
        mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

        # Wireframe
        wire = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
        wire.paint_uniform_color([0.3, 0.3, 0.3])

        return mesh, wire

    def callback(self, msg):
        start = time.time()

        points = point_cloud2.read_points_numpy(msg, field_names=("x","y","z"), skip_nans=True)
        if len(points) < 500:
            return

        self.pcd.points = o3d.utility.Vector3dVector(points)

        # Color points by height
        z = points[:, 2]
        z_norm = (z - z.min()) / (z.ptp() + 1e-6)
        colors = np.stack([z_norm, np.zeros_like(z_norm), 1 - z_norm], axis=1)
        self.pcd.colors = o3d.utility.Vector3dVector(colors)

        # === Real-time mesh reconstruction (only every N seconds) ===
        now = time.time()
        if now - self.last_reconstruction > self.reconstruction_interval:
            self.last_reconstruction = now
            threading.Thread(target=self.update_mesh, args=(points.copy(),), daemon=True).start()

        # Update point cloud
        if self.first:
            self.vis.add_geometry(self.pcd)
            self.vis.add_geometry(self.mesh)
            self.vis.add_geometry(self.mesh_wire)
            self.first = False
        else:
            self.vis.update_geometry(self.pcd)
            self.vis.update_geometry(self.mesh)
            self.vis.update_geometry(self.mesh_wire)

        # === Metrology overlay ===
        dims = self.estimate_dimensions(points)
        if dims:
            lines = [
                f"Points: {dims['points']:,}",
                f"Length : {dims['length_m']:.3f} m",
                f"Width  : {dims['width_m']:.3f} m",
                f"Height : {dims['height_m']:.3f} m",
                f"Volume : {dims['volume_m3']:.3f} m³  ({dims['volume_liters']:.0f} L)",
                f"Mesh update every {self.reconstruction_interval}s",
            ]
            for i, line in enumerate(lines):
                self.add_text(line, pos=(20, 40 + i*35), color=(1, 1, 0) if i < 5 else (0.6, 0.8, 1))

        self.vis.poll_events()
        self.vis.update_renderer()

        elapsed = time.time() - start
        if elapsed > 0.1:
            self.get_logger().warn(f"Slow visualization frame: {elapsed*1000:.1f} ms")

    def update_mesh(self, points):
        try:
            mesh, wire = self.reconstruct_mesh(points)
            self.mesh.vertices = mesh.vertices
            self.mesh.vertex_normals = mesh.vertex_normals
            self.mesh.vertex_colors = mesh.vertex_colors
            self.mesh.triangles = mesh.triangles

            self.mesh_wire.lines = wire.lines
            self.mesh_wire.points = wire.points
            self.mesh_wire.colors = wire.colors
        except Exception as e:
            self.get_logger().error(f"Mesh reconstruction failed: {e}")

def main():
    rclpy.init()
    node = BaleMeshVisualizer()

    def spin():
        rclpy.spin(node)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()

    node.vis.run()  # blocks until window closed

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()