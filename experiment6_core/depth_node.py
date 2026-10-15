"""Convert depth images into base-frame obstacle points and sector summaries."""
import math
import time
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException
from experiment6_navigation.msg import ObstacleSummary
from .geometry import quaternion_matrix
from .perception import (PerceptionConfig, UNKNOWN, decode_depth, pixel_rays,
                         extract_obstacles)


class DepthNode(Node):
    def __init__(self):
        super().__init__('r1_depth_obstacles')
        values = {key:self.declare_parameter(key,value).value
                  for key,value in PerceptionConfig().__dict__.items()}
        self.config = PerceptionConfig(**values)
        self.config.validate()
        self.stride = self.declare_parameter('pixel_stride',2).value
        self.timeout = self.declare_parameter('depth_timeout',1.5).value
        self.publish_debug = self.declare_parameter('publish_debug',True).value
        if self.stride < 1 or not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('Invalid stride or depth timeout')
        self.info = None
        self.ray_key = self.rays = None
        self.last_success = float('-inf')
        self.last_error = ''
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.points_pub = self.create_publisher(PointCloud2,'/r1/obstacle_points',qos_profile_sensor_data)
        self.summary_pub = self.create_publisher(ObstacleSummary,'/r1/obstacle_summary',10)
        self.debug_pub = self.create_publisher(Image,'/r1/depth/debug',qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/r1/depth/camera_info',self.camera_info,qos_profile_sensor_data)
        self.create_subscription(Image,'/r1/depth/image',self.image,qos_profile_sensor_data)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.25,self.watchdog,clock=self.steady_clock)

    def camera_info(self,msg):
        self.info=msg

    def unknown(self,status):
        result=ObstacleSummary()
        result.header.stamp=self.get_clock().now().to_msg()
        result.header.frame_id='base_link'
        result.data_valid=False
        result.status=status
        result.distance_m=[float('nan')]*3
        result.valid_ratio=[0.0]*3
        result.state=[UNKNOWN]*3
        self.summary_pub.publish(result)
        if status != self.last_error:
            self.get_logger().warning(status)
            self.last_error=status

    def watchdog(self):
        if time.monotonic()-self.last_success>self.timeout:
            self.unknown('depth_not_received_or_stale')

    def image(self,msg):
        try:
            if self.info is None: raise ValueError('waiting_for_depth_camera_info')
            info=self.info
            if (info.width,info.height)!=(msg.width,msg.height):
                raise ValueError('depth_image_and_camera_info_resolution_mismatch')
            if not msg.header.frame_id or info.header.frame_id!=msg.header.frame_id:
                raise ValueError('depth_image_and_camera_info_frame_mismatch')
            if info.distortion_model not in ('','plumb_bob'):
                raise ValueError('unsupported_depth_distortion_model')
            if info.binning_x>1 or info.binning_y>1 or info.roi.x_offset or info.roi.y_offset:
                raise ValueError('binned_or_cropped_camera_info_not_supported')
            age=(self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
            if age>self.timeout or age < -0.2:
                raise ValueError('depth_timestamp_is_stale_or_in_future')
            depth=decode_depth(msg.data,msg.width,msg.height,msg.step,msg.encoding,msg.is_bigendian)
            key=(msg.width,msg.height,tuple(info.k),tuple(info.d),self.stride)
            if key!=self.ray_key:
                self.rays=pixel_rays(msg.width,msg.height,info.k,info.d,self.stride)
                self.ray_key=key
            transform=self.tf_buffer.lookup_transform(
                'base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.02))
            q=transform.transform.rotation
            t=transform.transform.translation
            rotation=quaternion_matrix(q.x,q.y,q.z,q.w)
            points,distances,ratios,states=extract_obstacles(
                depth[::self.stride,::self.stride].ravel(),self.rays,rotation,
                [t.x,t.y,t.z],self.config)
            header=Header()
            header.stamp=msg.header.stamp
            header.frame_id='base_link'
            cloud=PointCloud2()
            cloud.header=header;cloud.height=1;cloud.width=len(points)
            cloud.fields=[PointField(name=name,offset=index*4,datatype=PointField.FLOAT32,count=1)
                          for index,name in enumerate(('x','y','z'))]
            cloud.is_bigendian=False;cloud.point_step=12;cloud.row_step=12*len(points)
            cloud.is_dense=True;cloud.data=points.astype('<f4',copy=False).tobytes()
            self.points_pub.publish(cloud)
            result=ObstacleSummary()
            result.header=header
            result.data_valid=bool(np.all(states!=UNKNOWN))
            result.status='complete' if result.data_valid else 'partial_or_insufficient_depth'
            result.distance_m=distances.tolist();result.valid_ratio=ratios.tolist()
            result.state=[int(value) for value in states]
            result.obstacle_points=len(points)
            self.summary_pub.publish(result)
            if self.publish_debug: self.debug(msg,depth,distances,states)
            self.last_success=time.monotonic()
            self.last_error=''
        except (ValueError,TransformException) as error:
            self.unknown(str(error))

    def debug(self,source,depth,distances,states):
        import cv2
        safe=np.nan_to_num(depth,nan=0,posinf=0,neginf=0)
        scaled=(np.clip(safe,0,self.config.max_range)*255/self.config.max_range).astype(np.uint8)
        image=cv2.applyColorMap(255-scaled,cv2.COLORMAP_TURBO)
        image[~np.isfinite(depth)|(depth<=0)]=0
        image=cv2.resize(image,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
        # Labels are base-frame sectors, ordered left/front/right, not raw columns.
        for i,name in enumerate(('L','F','R')):
            label=f'{name}:unknown' if states[i]==UNKNOWN else f'{name}:{distances[i]:.2f}m'
            cv2.putText(image,label,(5+i*image.shape[1]//3,22),cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,(255,255,255),1,cv2.LINE_AA)
        output=Image()
        output.header=source.header
        output.height,output.width=image.shape[:2]
        output.encoding='bgr8';output.is_bigendian=False
        output.step=output.width*3;output.data=image.tobytes()
        self.debug_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node=DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
