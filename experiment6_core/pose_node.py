"""Publish standard ROS pose/odometry/TF from measured simulator feedback."""
import copy
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from .geometry import quaternion_matrix


class PoseNode(Node):
    def __init__(self):
        super().__init__('r1_pose_publisher')
        self.publisher = self.create_publisher(Odometry, '/r1/odom', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/r1/pose', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/r1/sim/odom', self.receive, 10)

    def receive(self, source):
        if source.header.frame_id != 'odom' or source.child_frame_id != 'base_link':
            self.get_logger().error('Raw odometry must use odom -> base_link')
            return
        p, q = source.pose.pose.position, source.pose.pose.orientation
        v, w = source.twist.twist.linear, source.twist.twist.angular
        if not all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z)):
            return
        try:
            quaternion_matrix(q.x,q.y,q.z,q.w)
        except ValueError:
            return
        message = copy.deepcopy(source)
        q = message.pose.pose.orientation
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm
        self.publisher.publish(message)
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.pose_publisher.publish(pose)
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = message.child_frame_id
        p = message.pose.pose.position
        transform.transform.translation.x = p.x
        transform.transform.translation.y = p.y
        transform.transform.translation.z = p.z
        transform.transform.rotation = message.pose.pose.orientation
        self.tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = PoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
