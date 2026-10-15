"""Run after colcon build/source, with an isolated ROS_DOMAIN_ID.

This test only publishes ROS messages; it does not start the hardware backend.
"""
import importlib.util
import math
import os
import threading
import time
import unittest

HAS_ROS=importlib.util.find_spec('rclpy') is not None
ISOLATED=os.environ.get('ROS_DOMAIN_ID')=='173'
if HAS_ROS and ISOLATED:
    import numpy as np
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from experiment6_navigation.msg import ObstacleSummary
    from experiment6_core.pose_node import PoseNode
    from experiment6_core.motion_node import MotionNode
    from experiment6_core.depth_node import DepthNode


@unittest.skipUnless(HAS_ROS and ISOLATED,'Requires ROS 2 and isolated ROS_DOMAIN_ID=173')
class RosIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.harness=rclpy.create_node('experiment6_test_harness')
        cls.pose_node=PoseNode();cls.motion_node=MotionNode();cls.depth_node=DepthNode()
        cls.executor=MultiThreadedExecutor(num_threads=4)
        for node in (cls.harness,cls.pose_node,cls.motion_node,cls.depth_node):cls.executor.add_node(node)
        cls.thread=threading.Thread(target=cls.executor.spin,daemon=True);cls.thread.start()
        cls.raw_pub=cls.harness.create_publisher(Odometry,'/r1/sim/odom',10)
        cls.cmd_pub=cls.harness.create_publisher(Twist,'/r1/cmd_vel',1)
        cls.info_pub=cls.harness.create_publisher(CameraInfo,'/r1/depth/camera_info',qos_profile_sensor_data)
        cls.image_pub=cls.harness.create_publisher(Image,'/r1/depth/image',qos_profile_sensor_data)
        cls.pose_messages=[];cls.commands=[];cls.summaries=[];cls.clouds=[]
        cls.subscriptions=[
            cls.harness.create_subscription(PoseStamped,'/r1/pose',cls.pose_messages.append,10),
            cls.harness.create_subscription(Twist,'/r1/sim/cmd_vel',cls.commands.append,10),
            cls.harness.create_subscription(ObstacleSummary,'/r1/obstacle_summary',cls.summaries.append,10),
            cls.harness.create_subscription(PointCloud2,'/r1/obstacle_points',cls.clouds.append,qos_profile_sensor_data)]
        cls.static=StaticTransformBroadcaster(cls.harness)
        transform=TransformStamped()
        transform.header.stamp=cls.harness.get_clock().now().to_msg()
        transform.header.frame_id='base_link';transform.child_frame_id='depth_camera_optical_frame'
        transform.transform.translation.x=.15;transform.transform.translation.z=1.
        q=transform.transform.rotation;q.x=-.5;q.y=.5;q.z=-.5;q.w=.5
        cls.static.sendTransform(transform)
        time.sleep(.5)

    @classmethod
    def tearDownClass(cls):
        cls.motion_node.stop();cls.executor.shutdown();cls.thread.join(timeout=3)
        for node in (cls.harness,cls.pose_node,cls.motion_node,cls.depth_node):node.destroy_node()
        rclpy.shutdown()

    def send_pose(self):
        msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_link'
        msg.header.stamp=self.harness.get_clock().now().to_msg()
        msg.pose.pose.position.x=1.2;msg.pose.pose.orientation.w=1.
        self.raw_pub.publish(msg)

    def wait_for(self,predicate,pump=None,timeout=4):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if pump:pump()
            if predicate():return
            time.sleep(.03)
        self.fail('Timed out waiting for ROS message contract')

    def test_01_measured_pose_is_republished(self):
        self.pose_messages.clear()
        self.wait_for(lambda:any(abs(m.pose.position.x-1.2)<1e-8 for m in self.pose_messages),self.send_pose)

    def test_02_depth_produces_base_frame_obstacles(self):
        self.summaries.clear();self.clouds.clear()
        info=CameraInfo();info.header.frame_id='depth_camera_optical_frame';info.width=160;info.height=120
        info.k=[138.564,0.,80.,0.,138.564,60.,0.,0.,1.];info.d=[0.]*5;info.distortion_model='plumb_bob'
        image=Image();image.header.frame_id=info.header.frame_id;image.width=160;image.height=120
        image.step=160*4;image.encoding='32FC1';image.data=np.full((120,160),2.,dtype='<f4').tobytes()
        def pump():
            stamp=self.harness.get_clock().now().to_msg();info.header.stamp=stamp;image.header.stamp=stamp
            self.info_pub.publish(info);self.image_pub.publish(image)
        self.wait_for(lambda:any(m.data_valid and m.obstacle_points>0 for m in self.summaries),pump)
        self.assertTrue(self.clouds)
        self.assertEqual(self.clouds[-1].header.frame_id,'base_link')

    def test_03_command_limits_and_watchdog(self):
        self.commands.clear()
        cmd=Twist();cmd.linear.x=1.
        def pump():self.send_pose();self.cmd_pub.publish(cmd)
        self.wait_for(lambda:any(abs(m.linear.x-.25)<1e-8 for m in self.commands),pump)
        time.sleep(.6)
        self.assertAlmostEqual(self.commands[-1].linear.x,0.)


if __name__=='__main__':unittest.main()
