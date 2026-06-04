#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import numpy as np
import time


class OnlineMapServer(Node):

    def __init__(self):
        super().__init__('online_map_server')

        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('cloud_topic', '/Laser_map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('z_min', -0.2)
        self.declare_parameter('z_max', 2.0)
        self.declare_parameter('publish_rate', 1.0)  # Hz

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        map_topic = self.get_parameter('map_topic').value
        self._map_pub = self.create_publisher(OccupancyGrid, map_topic, map_qos)

        cloud_topic = self.get_parameter('cloud_topic').value
        self._cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self._cloud_callback, 1)

        self._latest_cloud = None
        self._last_publish_time = 0.0

        self.get_logger().info(
            f'OnlineMapServer started: {cloud_topic} -> {map_topic}')

    def _cloud_callback(self, msg):
        self._latest_cloud = msg

    def _pointcloud2_to_xyz(self, msg):
        data = np.frombuffer(msg.data, dtype=np.float32)
        return data.reshape(-1, msg.point_step // 4)[:, :3]

    def _build_map(self, cloud_msg):
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value
        resolution = self.get_parameter('resolution').value
        frame_id = self.get_parameter('frame_id').value

        xyz = self._pointcloud2_to_xyz(cloud_msg)
        mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
        xy = xyz[mask][:, :2]

        if len(xy) < 10:
            return None

        x_min, y_min = xy.min(axis=0) - resolution * 10
        x_max, y_max = xy.max(axis=0) + resolution * 10

        width = int(np.ceil((x_max - x_min) / resolution))
        height = int(np.ceil((y_max - y_min) / resolution))

        grid = np.full((height, width), -1, dtype=np.int8)

        ix = np.floor((xy[:, 0] - x_min) / resolution).astype(np.int32)
        iy = np.floor((xy[:, 1] - y_min) / resolution).astype(np.int32)

        valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        grid[iy[valid], ix[valid]] = 100

        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.map_load_time = self.get_clock().now().to_msg()
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = x_min
        msg.info.origin.position.y = y_min
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        return msg

    def spin_once(self):
        if self._latest_cloud is None:
            return
        now = time.time()
        rate = self.get_parameter('publish_rate').value
        if now - self._last_publish_time < 1.0 / rate:
            return
        self._last_publish_time = now

        map_msg = self._build_map(self._latest_cloud)
        if map_msg is not None:
            self._map_pub.publish(map_msg)
            self.get_logger().info(
                f'Published map: {map_msg.info.width}x{map_msg.info.height}')


def main():
    rclpy.init()
    node = OnlineMapServer()
    rate = node.create_rate(10)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.spin_once()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
