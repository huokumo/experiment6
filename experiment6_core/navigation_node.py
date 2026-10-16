"""ROS 2 adapter for the depth-guided teaching state machine."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from experiment6_navigation.msg import ObstacleSummary

from .navigation import NavigationConfig, WallBypass, Pose2, Obstacles


class NavigationNode(Node):
    def __init__(self):
        super().__init__('r1_wall_bypass')
        defaults = NavigationConfig()
        values = {key: self.declare_parameter(key, value).value
                  for key, value in defaults.__dict__.items()}
        self.controller = WallBypass(NavigationConfig(**values))
        self.pose = None
        self.obstacles = None
        self.collision = False
        self.publisher = self.create_publisher(Twist, '/r1/cmd_vel', 1)
        self.status_publisher = self.create_publisher(String, '/r1/navigation_status', 10)
        self.create_subscription(Odometry, '/r1/odom', self.on_odom, 10)
        self.create_subscription(ObstacleSummary, '/r1/obstacle_summary', self.on_obstacles, 10)
        self.create_subscription(Bool, '/r1/sim/collision_blocked', self.on_collision, 10)
        self.create_timer(0.1, self.tick)
        self.last_state = None

    def on_odom(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            self.pose = None
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.pose = Pose2(p.x, p.y, yaw, time.monotonic())

    def on_obstacles(self, message):
        self.obstacles = Obstacles(message.data_valid, tuple(message.distance_m),
                                   tuple(message.state), time.monotonic())

    def on_collision(self, message):
        self.collision = message.data

    def tick(self):
        command = self.controller.step(time.monotonic(), self.pose, self.obstacles,
                                       self.collision)
        message = Twist()
        message.linear.x = command.vx
        message.angular.z = command.omega
        self.publisher.publish(message)
        status = {'state': self.controller.state, 'reason': self.controller.reason,
                  'goal': [self.controller.config.goal_x, self.controller.config.goal_y],
                  'side': self.controller.config.side,
                  'pose': None if self.pose is None else
                  [round(self.pose.x, 3), round(self.pose.y, 3), round(self.pose.yaw, 3)],
                  'front_distance': None if self.obstacles is None or
                  not math.isfinite(self.obstacles.distances[1]) else
                  round(self.obstacles.distances[1], 3)}
        report = String()
        report.data = json.dumps(status, ensure_ascii=False)
        self.status_publisher.publish(report)
        if self.controller.state != self.last_state:
            self.get_logger().info(report.data)
            self.last_state = self.controller.state

    def stop(self):
        for _ in range(5):
            self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
