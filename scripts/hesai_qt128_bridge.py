#!/usr/bin/env python3
# Bridge between the p-steer rig (Hesai QT128 + Xsens) and FAST-LIO. Lives in
# FAST_LIO/scripts and is started by launch/mapping_hesai_qt128.launch.
#
# Input side (FAST-LIO has no Hesai type):
#   /hesai/points -> /hesai/points_lio
#     QT128 cloud (x/y/z/intensity f32, ring u16, timestamp f64 ABSOLUTE seconds,
#     header = scan start) re-packed as FAST-LIO velodyne points: x/y/z/intensity/
#     time f32 + ring u16, time = seconds since the header stamp. Cropped to a
#     +/-BOX m box and sorted by time.
#   /imu/data -> /imu/data_lio
#     The Xsens driver stamps samples on receive, in bursts of ~5 stamped
#     microseconds apart every ~12 ms. Re-stamped causally as
#       t_new = max(t_raw - BURST_SPAN, t_prev + 1 / IMU_RATE).
#     Gaps > GAP (e.g. rosbag-record buffer overflows) are filled with synthetic
#     samples: constant angular velocity from the Xsens orientation change across
#     the gap, acceleration interpolated. Without this FAST-LIO mis-registers the
#     next scan by ~4 deg of tilt and never recovers. Live it only fires on real gaps.
#
# Output side:
#   FAST-LIO /Odometry (camera_init -> body, body = IMU frame, camera_init = IMU
#   frame at start, NOT gravity-aligned) -> /lio_odom + odom->base_link TF:
#     T_odom_base = R_align * T_init_imu * inv(T_base_imu)
#   R_align rotates the gravity measured before the first pose onto +z; T_base_imu
#   is base_link->imu from the robot's static TF.
#   FAST-LIO has no loop closure, so map == odom: an identity map->odom static TF
#   is published (same map->odom->base_link tree GLIM produced), and /lio_pose
#   carries base_link in map as a PoseStamped (stand-in for /glim_ros/pose, which
#   was the IMU pose in map).
import copy

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf.transformations import (quaternion_from_matrix, quaternion_inverse,
                                quaternion_matrix, quaternion_multiply)

LIO_FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("intensity", 12, PointField.FLOAT32, 1),
    PointField("time", 16, PointField.FLOAT32, 1),
    PointField("ring", 20, PointField.UINT16, 1),
]
LIO_DTYPE = np.dtype({"names": ["x", "y", "z", "intensity", "time", "ring"],
                      "formats": ["<f4", "<f4", "<f4", "<f4", "<f4", "<u2"],
                      "offsets": [0, 4, 8, 12, 16, 20], "itemsize": 24})
IN_TYPES = {PointField.FLOAT32: "<f4", PointField.FLOAT64: "<f8",
            PointField.UINT16: "<u2", PointField.UINT8: "u1"}


def pose_to_mat(t, q):
    m = quaternion_matrix([q.x, q.y, q.z, q.w])
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def align_up(acc):
    # Smallest rotation taking the measured up direction onto +z.
    a = acc / np.linalg.norm(acc)
    z = np.array([0.0, 0.0, 1.0])
    v, c = np.cross(a, z), float(a @ z)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    m = np.eye(4)
    m[:3, :3] = np.eye(3) + vx + vx @ vx / (1.0 + c)
    return m


class Bridge(object):
    def __init__(self):
        self.box = rospy.get_param("~crop_box", 60.0)
        self.imu_rate = rospy.get_param("~imu_rate", 400.0)
        self.burst_span = rospy.get_param("~imu_burst_span", 0.012)
        self.gap = rospy.get_param("~imu_gap", 0.05)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.imu_frame = rospy.get_param("~imu_frame", "imu")

        self.in_dtype = None
        self.last_stamp = None
        self.prev_imu = None
        self.init_acc = []
        self.R_align = None
        self.T_imu_base = None

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        if self.map_frame:
            ident = TransformStamped()
            ident.header.stamp = rospy.Time.now()
            ident.header.frame_id = self.map_frame
            ident.child_frame_id = self.odom_frame
            ident.transform.rotation.w = 1.0
            self.static_br.sendTransform(ident)
        self.pose_pub = rospy.Publisher("/lio_pose", PoseStamped, queue_size=50)
        self.cloud_pub = rospy.Publisher("/hesai/points_lio", PointCloud2, queue_size=10)
        self.imu_pub = rospy.Publisher("/imu/data_lio", Imu, queue_size=2000)
        self.odom_pub = rospy.Publisher("/lio_odom", Odometry, queue_size=50)
        rospy.Subscriber("/hesai/points", PointCloud2, self.cloud_cb,
                         queue_size=10, buff_size=2 ** 27, tcp_nodelay=True)
        rospy.Subscriber("/imu/data", Imu, self.imu_cb, queue_size=2000, tcp_nodelay=True)
        rospy.Subscriber("/Odometry", Odometry, self.odom_cb, queue_size=50)

    # ---------------------------------------------------------------- LiDAR in
    def cloud_cb(self, msg):
        if self.in_dtype is None:
            f = {x.name: x for x in msg.fields}
            names = ["x", "y", "z", "intensity", "ring", "timestamp"]
            self.in_dtype = np.dtype({"names": names,
                                      "formats": [IN_TYPES[f[n].datatype] for n in names],
                                      "offsets": [f[n].offset for n in names],
                                      "itemsize": msg.point_step})
        pts = np.frombuffer(msg.data, dtype=self.in_dtype, count=msg.width * msg.height)
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1)
        keep = (np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0)
                & (np.abs(xyz) <= self.box).all(axis=1))
        pts = pts[keep]
        t = (pts["timestamp"] - msg.header.stamp.to_sec()).astype(np.float32)
        order = np.argsort(t, kind="stable")

        out = np.empty(len(pts), dtype=LIO_DTYPE)
        for n in ("x", "y", "z", "intensity", "ring"):
            out[n] = pts[n][order]
        out["time"] = t[order]

        o = PointCloud2()
        o.header = msg.header
        o.height, o.width = 1, len(out)
        o.fields = LIO_FIELDS
        o.is_bigendian = False
        o.point_step = LIO_DTYPE.itemsize
        o.row_step = o.point_step * o.width
        o.is_dense = True
        o.data = out.tobytes()
        self.cloud_pub.publish(o)

    # ------------------------------------------------------------------ IMU in
    def imu_cb(self, msg):
        if self.R_align is None:
            a = msg.linear_acceleration
            self.init_acc = (self.init_acc + [(a.x, a.y, a.z)])[-400:]
        prev, self.prev_imu = self.prev_imu, copy.deepcopy(msg)
        if prev is not None:
            gap = msg.header.stamp.to_sec() - prev.header.stamp.to_sec()
            if gap > self.gap:
                self.fill_gap(prev, msg, gap)
        self.restamp_publish(msg)

    def fill_gap(self, a, b, gap):
        qa = [a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w]
        qb = [b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w]
        dq = np.array(quaternion_multiply(quaternion_inverse(qa), qb))  # body-frame a -> b
        dq *= np.sign(dq[3])
        angle = 2.0 * np.arctan2(np.linalg.norm(dq[:3]), dq[3])
        w = dq[:3] / max(np.linalg.norm(dq[:3]), 1e-12) * angle / gap
        acc_a = np.array([a.linear_acceleration.x, a.linear_acceleration.y, a.linear_acceleration.z])
        acc_b = np.array([b.linear_acceleration.x, b.linear_acceleration.y, b.linear_acceleration.z])
        n = int(gap * self.imu_rate)
        rospy.logwarn("IMU gap of %.3f s: filling %d samples, rotation %.1f deg",
                      gap, n - 1, np.degrees(angle))
        for i in range(1, n):
            f = i / float(n)
            m = copy.deepcopy(a)
            m.header.stamp = rospy.Time.from_sec(a.header.stamp.to_sec() + f * gap)
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = w
            (m.linear_acceleration.x, m.linear_acceleration.y,
             m.linear_acceleration.z) = (1 - f) * acc_a + f * acc_b
            self.restamp_publish(m)

    def restamp_publish(self, msg):
        t_new = msg.header.stamp.to_sec() - self.burst_span
        if self.last_stamp is not None:
            t_new = max(t_new, self.last_stamp + 1.0 / self.imu_rate)
        self.last_stamp = t_new
        msg.header.stamp = rospy.Time.from_sec(t_new)
        self.imu_pub.publish(msg)

    # ------------------------------------------------------------- odometry out
    def odom_cb(self, msg):
        if self.T_imu_base is None:
            try:
                tf = self.tf_buf.lookup_transform(self.base_frame, self.imu_frame, rospy.Time(0))
            except tf2_ros.TransformException as e:
                rospy.logwarn_throttle(5, "waiting for %s -> %s: %s", self.base_frame, self.imu_frame, e)
                return
            self.T_imu_base = np.linalg.inv(pose_to_mat(tf.transform.translation, tf.transform.rotation))
        if self.R_align is None:
            if not self.init_acc:
                return
            self.R_align = align_up(np.mean(self.init_acc, axis=0))
            rospy.loginfo("gravity alignment from %d IMU samples", len(self.init_acc))

        p = msg.pose.pose
        T = self.R_align @ pose_to_mat(p.position, p.orientation) @ self.T_imu_base
        q = quaternion_from_matrix(T)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = T[:3, 3]
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = q
        self.odom_pub.publish(out)
        if self.map_frame:
            ps = PoseStamped()
            ps.header.stamp = out.header.stamp
            ps.header.frame_id = self.map_frame
            ps.pose = out.pose.pose
            self.pose_pub.publish(ps)

        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = T[:3, 3]
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = q
        self.br.sendTransform(tf)


if __name__ == "__main__":
    rospy.init_node("hesai_qt128_bridge")
    Bridge()
    rospy.spin()
