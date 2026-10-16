"""Numerical contracts that can be checked without ROS or Gazebo."""
import math
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from experiment6_core.geometry import quaternion_matrix, clamp_velocity
from experiment6_core.perception import (
    UNKNOWN, CLEAR, BLOCKED, PerceptionConfig, decode_depth, pixel_rays, extract_obstacles)
from experiment6_core.scene import generate_scene, world_obstacles


class DepthContracts(unittest.TestCase):
    def test_float_rows_respect_padding_and_endianness(self):
        data=struct.pack('>ff',1.25,2.5)+b'PAD!'+struct.pack('>ff',3.75,4.0)+b'PAD!'
        image=decode_depth(data,2,2,12,'32FC1',True)
        np.testing.assert_allclose(image,[[1.25,2.5],[3.75,4.0]])

    def test_uint16_is_converted_from_mm_to_m(self):
        image=decode_depth(struct.pack('<HH',1500,0),2,1,4,'16UC1')
        np.testing.assert_allclose(image,[[1.5,0]])

    def test_invalid_buffers_are_rejected(self):
        for data,step,encoding in ((b'1234',2,'32FC1'),(b'1234',8,'32FC1'),(b'1234',4,'rgb8')):
            with self.assertRaises(ValueError): decode_depth(data,2,1,step,encoding)

    def test_camera_rays_use_depth_intrinsics(self):
        rays=pixel_rays(3,3,[2,0,1,0,2,1,0,0,1],stride=1)
        np.testing.assert_allclose(rays[4],[0,0,1])
        np.testing.assert_allclose(rays[0],[-.5,-.5,1])

    def setUp(self):
        # Optical axes: right, down, forward -> base: forward, left, up.
        self.rotation=np.array([[0,0,1],[-1,0,0],[0,-1,0]],dtype=float)
        self.rays=np.array([[.4,0,1],[0,0,1],[-.4,0,1]],dtype=float)
        self.config=PerceptionConfig(min_sector_pixels=1)

    def test_sector_order_is_robot_left_front_right(self):
        points,distance,ratio,state=extract_obstacles(
            [.5,2,3],self.rays,self.rotation,[0,0,.8],self.config)
        # Image left is robot left: last ray is a farther LEFT obstacle.
        np.testing.assert_allclose(distance,[math.hypot(3,1.2),2,math.hypot(.5,.2)])
        self.assertEqual(state.tolist(),[CLEAR,CLEAR,BLOCKED])
        np.testing.assert_allclose(ratio,[1,1,1])
        self.assertEqual(points.shape,(3,3))

    def test_invalid_depth_is_unknown_not_clear(self):
        points,distance,ratio,state=extract_obstacles(
            [np.nan,np.inf,0],self.rays,self.rotation,[0,0,.8],self.config)
        self.assertEqual(len(points),0)
        self.assertTrue(np.isnan(distance).all())
        self.assertEqual(state.tolist(),[UNKNOWN]*3)

    def test_floor_is_removed_after_coordinate_transform(self):
        rays=np.array([[0,.5,1],[0,0,1]])
        points,_,_,_=extract_obstacles([2,2],rays,self.rotation,[0,0,1],self.config)
        self.assertEqual(len(points),1)
        np.testing.assert_allclose(points[0],[2,0,1])

    def test_close_hit_survives_low_coverage(self):
        rays=np.repeat([[0,0,1]],10,axis=0)
        depths=[.5]+[np.nan]*9
        _,distance,ratios,state=extract_obstacles(depths,rays,self.rotation,[0,0,.8],self.config)
        self.assertEqual(state[1],BLOCKED)
        self.assertAlmostEqual(distance[1],.5)
        self.assertAlmostEqual(ratios[1],.1)

    def test_unobserved_sector_is_unknown(self):
        _,_,_,state=extract_obstacles([2],[[0,0,1]],self.rotation,[0,0,.8],self.config)
        self.assertEqual(state.tolist(),[UNKNOWN,CLEAR,UNKNOWN])

    def test_floor_only_does_not_fabricate_obstacle_points(self):
        _,distance,_,state=extract_obstacles([2],[[0,.5,1]],self.rotation,[0,0,1],self.config)
        self.assertEqual(state[1],CLEAR)
        self.assertEqual(distance[1],self.config.max_range)


class GeometryContracts(unittest.TestCase):
    def test_nonunit_quaternion_is_normalized(self):
        np.testing.assert_allclose(quaternion_matrix(0,0,0,2),np.eye(3))

    def test_invalid_quaternion_rejected(self):
        with self.assertRaises(ValueError): quaternion_matrix(0,0,0,0)

    def test_planar_velocity_has_vector_limit(self):
        vx,vy,w=clamp_velocity(1,1,3,.2,.5)
        self.assertAlmostEqual(math.hypot(vx,vy),.2)
        self.assertEqual(w,.5)
        self.assertEqual(clamp_velocity(float('nan'),1,0,.2,.5),(0.,0.,0.))


class SceneContracts(unittest.TestCase):
    def fixtures(self,root):
        (root/'worlds').mkdir()
        (root/'models/unitree_r1').mkdir(parents=True)
        world='''<sdf version="1.8"><world name="old"><model name="wall"><static>true</static>
        <pose>2.5 0 .35 0 0 0</pose><link name="link"><collision name="c"><geometry>
        <box><size>.1 4 .7</size></box></geometry></collision></link></model>
        <model name="floor"><static>true</static><pose>0 0 -.05 0 0 0</pose>
        <link name="l"><collision name="c"><geometry><box><size>12 8 .1</size></box>
        </geometry></collision></link></model></world></sdf>'''
        model='''<sdf version="1.8"><model name="unitree_r1"><static>true</static><link name="camera">
        <sensor name="camera" type="depth_camera"><update_rate>2</update_rate></sensor>
        </link></model></sdf>'''
        (root/'worlds/depth_camera.world').write_text(world)
        (root/'models/unitree_r1/model_depth.sdf').write_text(model)

    def test_scene_is_generated_without_modifying_experiment5(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);self.fixtures(root)
            before=(root/'worlds/depth_camera.world').read_bytes()
            output=generate_scene(root,root/'runtime/world.sdf',spawn=(1,0,0,.2))
            world=ET.parse(output).getroot().find('world')
            self.assertEqual(world.get('name'),'experiment6_navigation')
            robot=world.find("model[@name='r1']")
            self.assertEqual(float(robot.findtext('.//update_rate')),10)
            self.assertEqual(len(robot.findall('plugin/obstacle')),1)
            floor=world.find("model[@name='floor']")
            self.assertEqual(floor.findtext('.//geometry/box/size'),'16.0 16.0 0.1')
            self.assertEqual(before,(root/'worlds/depth_camera.world').read_bytes())

    def test_overlapping_spawn_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);self.fixtures(root)
            with self.assertRaises(ValueError):generate_scene(root,root/'out.sdf',spawn=(2.4,0,0,0))

    def test_unsupported_collision_is_rejected(self):
        world=ET.fromstring('''<world><model name="thing"><static>true</static><link name="l">
        <collision name="c"><geometry><sphere><radius>1</radius></sphere></geometry></collision>
        </link></model></world>''')
        with self.assertRaises(ValueError):world_obstacles(world)


if __name__=='__main__':unittest.main()
