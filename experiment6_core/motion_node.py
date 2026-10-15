"""Simulation velocity adapter with independent wall-clock watchdogs."""
import json
import math
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from experiment6_navigation.msg import ObstacleSummary
from .geometry import clamp_velocity


class MotionNode(Node):
    def __init__(self):
        super().__init__('r1_motion_adapter')
        defaults = {'command_timeout':0.4, 'pose_timeout':0.5,
                    'depth_timeout':1.5, 'max_linear':0.25, 'max_angular':0.6,
                    'require_depth':False}
        self.settings = {key:self.declare_parameter(key,value).value for key,value in defaults.items()}
        for key in ('command_timeout','pose_timeout','depth_timeout','max_linear','max_angular'):
            if not math.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.last_command = self.last_pose = self.last_depth = float('-inf')
        self.target = (0.,0.,0.)
        self.depth_valid = False
        self.collision = False
        self.publisher = self.create_publisher(Twist, '/r1/sim/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/r1/motion_status', 10)
        self.create_subscription(Twist, '/r1/cmd_vel', self.command, 1)
        self.create_subscription(Odometry, '/r1/odom', self.pose, 10)
        self.create_subscription(ObstacleSummary, '/r1/obstacle_summary', self.depth, 1)
        self.create_subscription(Bool, '/r1/sim/collision_blocked', self.blocked, 1)
        self.last_status = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.tick, clock=self.steady_clock)

    def command(self, msg):
        values = (msg.linear.x,msg.linear.y,msg.angular.z)
        # Unsupported axes must not silently produce an unintended movement.
        extra = (msg.linear.z,msg.angular.x,msg.angular.y)
        if not all(math.isfinite(v) for v in values+extra) or any(abs(v)>1e-9 for v in extra):
            self.target=(0.,0.,0.);self.last_command=float('-inf')
            return
        self.target = clamp_velocity(*values,self.settings['max_linear'],self.settings['max_angular'])
        self.last_command = time.monotonic()

    def pose(self, msg):
        self.last_pose = time.monotonic()

    def depth(self, msg):
        self.last_depth = time.monotonic()
        self.depth_valid = msg.data_valid

    def blocked(self, msg):
        self.collision = msg.data

    def tick(self):
        now = time.monotonic()
        reason = 'ready'
        if now-self.last_command > self.settings['command_timeout']: reason='command_timeout'
        elif now-self.last_pose > self.settings['pose_timeout']: reason='pose_timeout'
        elif self.settings['require_depth'] and (
            now-self.last_depth > self.settings['depth_timeout'] or not self.depth_valid):
            reason='depth_unavailable'
        msg = Twist()
        if reason == 'ready':
            msg.linear.x,msg.linear.y,msg.angular.z=self.target
        self.publisher.publish(msg)
        status = (reason,self.collision)
        if status != self.last_status:
            payload=String()
            payload.data=json.dumps({'backend':'simulation','state':reason,
                                     'collision_blocked':self.collision},ensure_ascii=False)
            self.status_pub.publish(payload)
            self.get_logger().info(payload.data)
            self.last_status=status

    def stop(self):
        for _ in range(3): self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): node.stop()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
