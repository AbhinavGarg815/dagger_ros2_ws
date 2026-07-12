#!/usr/bin/env python3

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
import tf2_ros


# Match rslidar_sdk / mrg_slam prefiltering: RELIABLE, VOLATILE.
# depth=1 keeps only the freshest cloud so latency can't accumulate if a
# callback is momentarily slower than the 10 Hz input (stale clouds are dropped).
CLOUD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


_NP_TYPE = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


class SelfFilter(Node):

    def __init__(self):
        super().__init__("self_filter")

        self.declare_parameter("input_topic", "/rslidar_points")
        self.declare_parameter("output_topic", "/rslidar_points_filtered")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("min_x", -0.90)
        self.declare_parameter("max_x", 0.45)
        self.declare_parameter("min_y", -0.60)
        self.declare_parameter("max_y", 0.60)
        self.declare_parameter("min_z", -0.30)
        self.declare_parameter("max_z", 1.30)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.box = np.array([
            self.get_parameter("min_x").value, self.get_parameter("max_x").value,
            self.get_parameter("min_y").value, self.get_parameter("max_y").value,
            self.get_parameter("min_z").value, self.get_parameter("max_z").value,
        ], dtype=np.float64)

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.R = None
        self.T = None
        self.dtype = None
        self.warned = False
        self.n = 0
        self.acc = 0.0
        self.acc_pub = 0.0

        self.pub = self.create_publisher(PointCloud2, self.output_topic, CLOUD_QOS)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cb, CLOUD_QOS)

        self.get_logger().info(
            f"self_filter: {self.input_topic} -> {self.output_topic} | "
            f"box(base={self.base_frame}) x[{self.box[0]},{self.box[1]}] "
            f"y[{self.box[2]},{self.box[3]}] z[{self.box[4]},{self.box[5]}]"
        )

    def _lookup(self, cloud_frame, stamp):
        try:
            t = self.buf.lookup_transform(self.base_frame, cloud_frame, rclpy.time.Time())
        except Exception as e:
            if not self.warned:
                self.get_logger().warn(f"waiting for TF {self.base_frame}<-{cloud_frame}: {e}")
                self.warned = True
            return False
        q = t.transform.rotation
        tr = t.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        self.R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
        ], dtype=np.float64)
        self.T = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
        # Pre-map the base_footprint box into the (static) sensor frame as an
        # axis-aligned box, so per-cloud we filter raw sensor xyz with no transform.
        # sensor = R^T (bf - T)  ->  for row corners V: S = (V - T) @ R
        bx, by, bz = self.box[0:2], self.box[2:4], self.box[4:6]
        corners = np.array([[X, Y, Z] for X in bx for Y in by for Z in bz],
                           dtype=np.float64)
        S = (corners - self.T) @ self.R
        self.sbox = np.array([S[:, 0].min(), S[:, 0].max(),
                              S[:, 1].min(), S[:, 1].max(),
                              S[:, 2].min(), S[:, 2].max()], dtype=np.float32)
        self.get_logger().info(
            f"cached static TF {self.base_frame}<-{cloud_frame}; "
            f"sensor-frame box x[{self.sbox[0]:.2f},{self.sbox[1]:.2f}] "
            f"y[{self.sbox[2]:.2f},{self.sbox[3]:.2f}] z[{self.sbox[4]:.2f},{self.sbox[5]:.2f}]"
        )
        return True

    def _build_dtype(self, msg):
        names, formats, offsets = [], [], []
        for f in msg.fields:
            names.append(f.name)
            formats.append(_NP_TYPE[f.datatype])
            offsets.append(f.offset)
        self.dtype = np.dtype({
            "names": names, "formats": formats,
            "offsets": offsets, "itemsize": msg.point_step,
        })

    def cb(self, msg):
        if self.R is None and not self._lookup(msg.header.frame_id, msg.header.stamp):
            return
        if self.dtype is None:
            self._build_dtype(msg)

        t0 = time.perf_counter()
        arr = np.frombuffer(msg.data, dtype=self.dtype)
        if arr.shape[0] == 0:
            self.pub.publish(msg)
            return

        x = arr["x"]
        y = arr["y"]
        z = arr["z"]
        s = self.sbox
        inside = ((x >= s[0]) & (x <= s[1]) &
                  (y >= s[2]) & (y <= s[3]) &
                  (z >= s[4]) & (z <= s[5]))
        keep = ~inside

        out_arr = arr[keep]
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(out_arr.shape[0])
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * out.width
        out.is_dense = msg.is_dense
        out.data = out_arr.tobytes()
        t1 = time.perf_counter()
        self.pub.publish(out)
        t2 = time.perf_counter()

        self.acc += t1 - t0
        self.acc_pub += t2 - t1
        self.n += 1
        if self.n % 50 == 0:
            self.get_logger().info(
                f"self_filter: {arr.shape[0]}->{out.width} pts, "
                f"compute {1000.0*self.acc/50:.1f} ms, publish {1000.0*self.acc_pub/50:.1f} ms"
            )
            self.acc = 0.0
            self.acc_pub = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = SelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
