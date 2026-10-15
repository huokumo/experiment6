#!/usr/bin/python3
"""Publish one bounded-duration planar command, then explicit zero commands."""
import argparse
import math
import time
import rclpy
from geometry_msgs.msg import Twist


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vx',type=float,default=0.)
    parser.add_argument('--vy',type=float,default=0.)
    parser.add_argument('--yaw-rate',type=float,default=0.)
    parser.add_argument('--seconds',type=float,default=1.)
    args,ros_args=parser.parse_known_args()
    if not all(math.isfinite(v) for v in (args.vx,args.vy,args.yaw_rate,args.seconds)) or not 0<args.seconds<=30:
        parser.error('Use finite velocities and 0 < seconds <= 30')
    rclpy.init(args=ros_args)
    node=rclpy.create_node('r1_bounded_velocity_command')
    pub=node.create_publisher(Twist,'/r1/cmd_vel',1)
    try:
        deadline=time.monotonic()+3
        while pub.get_subscription_count()==0 and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.05)
        if pub.get_subscription_count()==0:
            raise RuntimeError('No motion adapter is subscribed to /r1/cmd_vel')
        msg=Twist()
        msg.linear.x=args.vx;msg.linear.y=args.vy;msg.angular.z=args.yaw_rate
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline and rclpy.ok():
            pub.publish(msg)
            rclpy.spin_once(node,timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(6):
                pub.publish(Twist());rclpy.spin_once(node,timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
