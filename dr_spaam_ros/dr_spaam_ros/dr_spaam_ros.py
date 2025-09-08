import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker

from dr_spaam.detector import Detector


class DrSpaamROS(Node):
    """ROS2 node to detect pedestrian using DROW3 or DR-SPAAM."""

    def __init__(self):
        super().__init__("dr_spaam_ros")

        self._read_params()
        self._detector = Detector(
            self.weight_file,
            model=self.detector_model,
            gpu=True,
            stride=self.stride,
            panoramic_scan=self.panoramic_scan,
        )
        self._init()

    def _read_params(self):
        """
        @brief      Reads parameters from ROS server.
        """
        self.declare_parameter("weight_file")
        self.declare_parameter("conf_thresh", 0.5)
        self.declare_parameter("stride", 1)
        self.declare_parameter("detector_model", "dr_spaam")
        self.declare_parameter("panoramic_scan", False)

        # publisher/subscriber parameters
        self.declare_parameter("subscriber.scan.topic", "/scan")
        self.declare_parameter("subscriber.scan.queue_size", 10)

        self.declare_parameter("publisher.detections.topic", "/detections")
        self.declare_parameter("publisher.detections.queue_size", 10)
        self.declare_parameter("publisher.detections.latch", False)

        self.declare_parameter("publisher.rviz.topic", "/rviz")
        self.declare_parameter("publisher.rviz.queue_size", 1)
        self.declare_parameter("publisher.rviz.latch", False)

        # read them back
        self.weight_file = self.get_parameter("weight_file").value
        self.conf_thresh = self.get_parameter("conf_thresh").value
        self.stride = self.get_parameter("stride").value
        self.detector_model = self.get_parameter("detector_model").value
        self.panoramic_scan = self.get_parameter("panoramic_scan").value

    def _init(self):
        """
        @brief      Initialize ROS connection.
        """
        # Publishers with QoS handling
        det_topic = self.get_parameter("publisher.detections.topic").value
        det_qsize = self.get_parameter("publisher.detections.queue_size").value
        det_latch = self.get_parameter("publisher.detections.latch").value

        rviz_topic = self.get_parameter("publisher.rviz.topic").value
        rviz_qsize = self.get_parameter("publisher.rviz.queue_size").value
        rviz_latch = self.get_parameter("publisher.rviz.latch").value

        qos_profile_det = QoSProfile(
            depth=det_qsize,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=(
                QoSDurabilityPolicy.TRANSIENT_LOCAL
                if det_latch
                else QoSDurabilityPolicy.VOLATILE
            ),
        )
        self._dets_pub = self.create_publisher(PoseArray, det_topic, qos_profile_det)

        qos_profile_rviz = QoSProfile(
            depth=rviz_qsize,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=(
                QoSDurabilityPolicy.TRANSIENT_LOCAL
                if rviz_latch
                else QoSDurabilityPolicy.VOLATILE
            ),
        )
        self._rviz_pub = self.create_publisher(Marker, rviz_topic, qos_profile_rviz)

        # Subscriber
        scan_topic = self.get_parameter("subscriber.scan.topic").value
        scan_qsize = self.get_parameter("subscriber.scan.queue_size").value

        qos_profile_sub = QoSProfile(
            depth=scan_qsize, reliability=QoSReliabilityPolicy.RELIABLE
        )
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, qos_profile_sub
        )

    def _scan_callback(self, msg):
        if (
            self._dets_pub.get_subscription_count() == 0
            and self._rviz_pub.get_subscription_count() == 0
        ):
            return

        # TODO check the computation here
        if not self._detector.is_ready():
            self._detector.set_laser_fov(
                np.rad2deg(msg.angle_increment * len(msg.ranges))
            )

        scan = np.array(msg.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        # t = time.time()
        dets_xy, dets_cls, _ = self._detector(scan)
        # print("[DrSpaamROS] End-to-end inference time: %f" % (t - time.time()))

        # confidence threshold
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        # convert to ros msg and publish
        dets_msg = detections_to_pose_array(dets_xy, dets_cls)
        dets_msg.header = msg.header
        self._dets_pub.publish(dets_msg)

        rviz_msg = detections_to_rviz_marker(dets_xy, dets_cls)
        rviz_msg.header = msg.header
        self._rviz_pub.publish(rviz_msg)


def detections_to_rviz_marker(dets_xy, dets_cls):
    """
    @brief     Convert detection to RViz marker msg. Each detection is marked as
               a circle approximated by line segments.
    """
    msg = Marker()
    msg.action = Marker.ADD
    msg.ns = "dr_spaam_ros"
    msg.id = 0
    msg.type = Marker.LINE_LIST

    # set quaternion so that RViz does not give warning
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    msg.scale.x = 0.03  # line width
    # red color
    msg.color.r = 1.0
    msg.color.a = 1.0

    # circle
    r = 0.4
    ang = np.linspace(0, 2 * np.pi, 20)
    xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

    # to msg
    for d_xy, d_cls in zip(dets_xy, dets_cls):
        for i in range(len(xy_offsets) - 1):
            # start point of a segment
            p0 = Point()
            p0.x = d_xy[0] + xy_offsets[i, 0]
            p0.y = d_xy[1] + xy_offsets[i, 1]
            p0.z = 0.0
            msg.points.append(p0)

            # end point
            p1 = Point()
            p1.x = d_xy[0] + xy_offsets[i + 1, 0]
            p1.y = d_xy[1] + xy_offsets[i + 1, 1]
            p1.z = 0.0
            msg.points.append(p1)

    return msg


def detections_to_pose_array(dets_xy, dets_cls):
    pose_array = PoseArray()
    for d_xy, d_cls in zip(dets_xy, dets_cls):
        # Detector uses following frame convention:
        # x forward, y rightward, z downward, phi is angle w.r.t. x-axis
        p = Pose()
        p.position.x = d_xy[0]
        p.position.y = d_xy[1]
        p.position.z = 0.0
        pose_array.poses.append(p)

    return pose_array
