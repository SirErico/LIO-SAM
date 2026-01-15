#!/usr/bin/env python3
"""
Ground Truth Odometry Publisher for LIO-SAM
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math


def quaternion_to_euler(q):
    """Convert quaternion to euler angles (roll, pitch, yaw)"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class GroundTruthOdomPublisher(Node):

    def __init__(self):
        super().__init__('ground_truth_odom_publisher')
        
        # Declare parameters
        self.declare_parameter('robot_frame', 'j100_0000/robot')
        self.declare_parameter('odom_topic', 'odometry/imu')
        self.declare_parameter('publish_tf', True)
        
        self.robot_frame = self.get_parameter('robot_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # QoS for IMU-like topics (what LIO-SAM expects)
        qos_imu = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2000
        )
        
        # Subscribe to Gazebo TF
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        
        # Publisher for odometry (what LIO-SAM's imageProjection expects)
        # LIO-SAM subscribes to odomTopic + "_incremental"
        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic + '_incremental',
            qos_imu)
        
        # Also publish a fake IMU for orientation initialization
        # This is needed because imageProjection checks for IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/j100_0000/sensors/imu_0/data',
            self.imu_callback,
            qos_imu)
        
        # TF broadcaster for map -> base_link
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        self.last_pose = None
        self.last_time = None
        
        self.get_logger().info(f'Ground Truth Odom Publisher started')
        self.get_logger().info(f'  Robot frame: {self.robot_frame}')
        self.get_logger().info(f'  Publishing to: {self.odom_topic}_incremental')
        self.get_logger().info(f'  Publishing TF: {self.publish_tf}')

    def imu_callback(self, msg):
        """Just pass through IMU - LIO-SAM needs it for orientation init"""
        pass  # The real IMU is still being used for orientation

    def tf_callback(self, msg):
        for transform in msg.transforms:
            # Check if this is the ground truth transform from Gazebo
            if transform.child_frame_id == self.robot_frame:
                self.publish_odometry(transform)
                
                if self.publish_tf:
                    self.publish_map_to_base_link(transform)
                break

    def publish_odometry(self, transform: TransformStamped):
        """Publish ground truth as odometry for LIO-SAM"""
        odom = Odometry()
        odom.header.stamp = transform.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z
        
        # Orientation
        odom.pose.pose.orientation = transform.transform.rotation
        
        # Covariance (low values = high confidence)
        odom.pose.covariance[0] = 0.001   # x
        odom.pose.covariance[7] = 0.001   # y
        odom.pose.covariance[14] = 0.001  # z
        odom.pose.covariance[21] = 0.001  # roll
        odom.pose.covariance[28] = 0.001  # pitch
        odom.pose.covariance[35] = 0.001  # yaw
        
        # Compute velocity if we have previous pose
        current_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        if self.last_pose is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                odom.twist.twist.linear.x = (transform.transform.translation.x - self.last_pose[0]) / dt
                odom.twist.twist.linear.y = (transform.transform.translation.y - self.last_pose[1]) / dt
                odom.twist.twist.linear.z = (transform.transform.translation.z - self.last_pose[2]) / dt
        
        self.last_pose = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        )
        self.last_time = current_time
        
        self.odom_pub.publish(odom)

    def publish_map_to_base_link(self, transform: TransformStamped):
        """Broadcast map -> base_link TF"""
        t = TransformStamped()
        t.header.stamp = transform.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform = transform.transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
