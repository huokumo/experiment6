"""Safe ROS starter: data wiring is complete; the student fills controller.py."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from experiment6_navigation.msg import ObstacleSummary
from .controller import Pose2, Obstacles, decide_motion


class StudentNode(Node):
    def __init__(self):
        super().__init__('r1_student_wall_bypass')
        self.goal_x = self.declare_parameter('goal_x', 4.2).value
        self.goal_y = self.declare_parameter('goal_y', 0.0).value
        self.pose = self.obstacles = None
        self.pose_at = self.obstacles_at = float('-inf')
        self.state = 'TODO'
        self.publisher = self.create_publisher(Twist, '/r1/cmd_vel', 1)
        self.status_publisher = self.create_publisher(String, '/r1/navigation_status', 10)
        self.create_subscription(Odometry, '/r1/odom', self.on_odom, 10)
        self.create_subscription(ObstacleSummary, '/r1/obstacle_summary', self.on_obstacles, 10)
        self.create_timer(0.1, self.tick)

    def on_odom(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.pose = Pose2(p.x, p.y, yaw)
        self.pose_at = time.monotonic()

    def on_obstacles(self, message):
        self.obstacles = Obstacles(
            message.data_valid,
            tuple(message.distance_m),
            tuple(message.state),
            time.monotonic(),
        )
        self.obstacles_at = self.obstacles.received_at

    def tick(self):
        now = time.monotonic()
        command = Twist()
        healthy = (self.pose is not None and self.obstacles is not None and
                   now-self.pose_at <= 1.5 and now-self.obstacles_at <= 1.5 and
                   self.obstacles.data_valid)
        if healthy and self.state not in ('DONE', 'FAILED'):
            next_state, velocity = decide_motion(self.pose, self.obstacles,
                                                 self.state, self.goal_x, self.goal_y)
            self.state = next_state
            if (math.isfinite(velocity.vx) and math.isfinite(velocity.omega) and
                    abs(velocity.vx) <= 0.25 and abs(velocity.omega) <= 0.6):
                command.linear.x = velocity.vx
                command.angular.z = velocity.omega
        self.publisher.publish(command)
        status = String()
        status.data = json.dumps({'state': self.state if healthy else 'WAIT_SENSORS',
                                  'goal': [self.goal_x, self.goal_y]})
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = StudentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):
                node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
