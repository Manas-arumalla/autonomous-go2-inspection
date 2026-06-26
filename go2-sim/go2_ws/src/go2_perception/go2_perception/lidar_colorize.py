#!/usr/bin/env python3
"""lidar_colorize -- fuse the front RGB camera onto the 3D LiDAR cloud (RGB + LiDAR).

Projects every LiDAR point into the camera image (camera_info K + the
utlidar_lidar -> camera_link TF, gz camera_link being x-forward/y-left/z-up) and
assigns it the pixel's RGB. Publishes:
  /points_colored : the LIVE colored scan (only points inside the camera FOV), lidar frame, XYZRGB
  /colored_map    : an ACCUMULATED, voxel-hashed RGB map in the 'map' frame, XYZRGB

Notes:
- The front camera sees an ~80x64 deg cone, so only points in that cone get colour each frame;
  as the robot turns/explores, /colored_map fills in. The lidar-only octomap still shows full geometry.
- This node is PURELY additive: it only reads /camera + the lidar cloud + TF and publishes colour.
  rtabmap stays pure-lidar for localization (we do NOT feed RGB back into SLAM).
- The same projection gives each pixel a 3D/map coordinate -> reused later to place SAM3 detections.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
import tf2_ros


def quat_to_mat(tf):
    q, tr = tf.rotation, tf.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    M = np.eye(4)
    M[0, 0] = 1 - 2 * (y * y + z * z); M[0, 1] = 2 * (x * y - z * w);     M[0, 2] = 2 * (x * z + y * w)
    M[1, 0] = 2 * (x * y + z * w);     M[1, 1] = 1 - 2 * (x * x + z * z); M[1, 2] = 2 * (y * z - x * w)
    M[2, 0] = 2 * (x * z - y * w);     M[2, 1] = 2 * (y * z + x * w);     M[2, 2] = 1 - 2 * (x * x + y * y)
    M[0, 3] = tr.x; M[1, 3] = tr.y; M[2, 3] = tr.z
    return M


class Colorize(Node):
    def __init__(self):
        super().__init__('lidar_colorize')
        self.declare_parameter('cloud_topic', '/utlidar/cloud_filtered')
        self.declare_parameter('voxel', 0.08)
        self.declare_parameter('map_frame', 'map')
        self.voxel = float(self.get_parameter('voxel').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.K = None          # (fx, fy, cx, cy)
        self.cam_frame = None
        self.img = None
        self.W = self.H = 0
        self.T_lc = None       # lidar -> camera_link (static)
        self.vox = {}          # (ix,iy,iz) -> (r,g,b) accumulated map
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.ci_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.img_cb, qos_profile_sensor_data)
        ctop = self.get_parameter('cloud_topic').value
        self.create_subscription(PointCloud2, ctop, self.cloud_cb, qos_profile_sensor_data)
        self.pub_live = self.create_publisher(PointCloud2, '/points_colored', 1)
        self.pub_map = self.create_publisher(PointCloud2, '/colored_map', 1)
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info(f'lidar_colorize up: {ctop} + /camera -> /points_colored + /colored_map')

    def ci_cb(self, m):
        if self.K is None:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])
            self.W, self.H = m.width, m.height
            self.cam_frame = m.header.frame_id
            self.get_logger().info(f'camera_info: {self.W}x{self.H} K=({self.K}) frame={self.cam_frame}')

    def img_cb(self, m):
        self.img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)  # rgb8

    def cloud_cb(self, m):
        if self.K is None or self.img is None:
            return
        if self.T_lc is None:
            try:
                t = self.tf.lookup_transform(self.cam_frame, m.header.frame_id, rclpy.time.Time())
                self.T_lc = quat_to_mat(t.transform)
            except Exception:
                return
        pts = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return
        pts = pts.astype(np.float64)
        # lidar -> camera_link
        Pc = (self.T_lc @ np.hstack([pts, np.ones((pts.shape[0], 1))]).T).T[:, :3]
        xl, yl, zl = Pc[:, 0], Pc[:, 1], Pc[:, 2]
        # camera_link (x-fwd,y-left,z-up) -> optical (x-right,y-down,z-fwd)
        xo, yo, zo = -yl, -zl, xl
        fx, fy, cx, cy = self.K
        with np.errstate(divide='ignore', invalid='ignore'):
            u = fx * xo / zo + cx
            v = fy * yo / zo + cy
        inb = (zo > 0.05) & (u >= 0) & (u < self.W) & (v >= 0) & (v < self.H)
        if not np.any(inb):
            return
        ui = u[inb].astype(np.int32)
        vi = v[inb].astype(np.int32)
        cols = self.img[vi, ui]              # Nx3 RGB
        xyz = pts[inb]                       # lidar-frame coords of the colored points
        self.pub_live.publish(self.make_cloud(m.header, xyz, cols))
        # accumulate in the map frame (voxel-hashed)
        try:
            tm = self.tf.lookup_transform(self.map_frame, m.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        Pm = (quat_to_mat(tm.transform) @ np.hstack([xyz, np.ones((xyz.shape[0], 1))]).T).T[:, :3]
        keys = np.floor(Pm / self.voxel).astype(np.int64)
        for k, c in zip(map(tuple, keys), cols):
            self.vox[k] = (int(c[0]), int(c[1]), int(c[2]))

    def make_cloud(self, header, xyz, cols):
        n = xyz.shape[0]
        rgb_u = (cols[:, 0].astype(np.uint32) << 16) | (cols[:, 1].astype(np.uint32) << 8) | cols[:, 2].astype(np.uint32)
        arr = np.zeros(n, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'f4')])
        arr['x'] = xyz[:, 0]; arr['y'] = xyz[:, 1]; arr['z'] = xyz[:, 2]
        arr['rgb'] = rgb_u.view(np.float32)   # standard packed-RGB-as-float32 (RViz RGB8 transformer)
        f = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
             PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]
        msg = PointCloud2()
        msg.header = header
        msg.height = 1; msg.width = n; msg.is_dense = True; msg.is_bigendian = False
        msg.fields = f; msg.point_step = 16; msg.row_step = 16 * n
        msg.data = arr.tobytes()
        return msg

    def publish_map(self):
        if not self.vox:
            return
        keys = np.array(list(self.vox.keys()), dtype=np.int64)
        cols = np.array(list(self.vox.values()), dtype=np.uint8)
        xyz = (keys.astype(np.float32) + 0.5) * self.voxel
        h = Header(); h.frame_id = self.map_frame; h.stamp = self.get_clock().now().to_msg()
        self.pub_map.publish(self.make_cloud(h, xyz, cols))


def main():
    rclpy.init()
    n = Colorize()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
