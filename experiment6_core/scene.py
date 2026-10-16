"""Generate an experiment-6 scene from the installed experiment-5 assets.

    Never modifies experiment5_depth. The collision guard supports the fixed,
    level, static box obstacles used by experiment 5; unsupported geometry is
    rejected explicitly rather than silently permitting penetration.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET


def numbers(text, count):
    values = [float(x) for x in (text or '').split()]
    if len(values) != count or not all(math.isfinite(x) for x in values):
        raise ValueError(f'Expected {count} finite numbers, got {text!r}')
    return values


def pose(element):
    child = element.find('pose')
    if child is None:
        return (0.0,) * 6
    if child.get('relative_to'):
        raise ValueError('World collision relative_to frames require explicit support')
    values = numbers(child.text, 6)
    if abs(values[3]) > 1e-8 or abs(values[4]) > 1e-8:
        raise ValueError('Tilted world collision boxes are not supported')
    return values


def compose(a, b):
    c, s = math.cos(a[5]), math.sin(a[5])
    return (a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], a[2]+b[2],
            0.0, 0.0, a[5]+b[5])


def world_obstacles(world):
    boxes = []
    for model in world.findall('model'):
        if model.find('model') is not None:
            raise ValueError('Nested world obstacles are unsupported')
        if model.find('link/collision') is None:
            continue
        if (model.findtext('static') or 'false').strip().lower() not in ('true', '1'):
            raise ValueError('The planar simulator requires static scene obstacles')
        mp = pose(model)
        for link in model.findall('link'):
            lp = compose(mp, pose(link))
            for collision in link.findall('collision'):
                cp = compose(lp, pose(collision))
                size_node = collision.find('geometry/box/size')
                if size_node is None:
                    raise ValueError('The planar collision guard currently supports boxes only')
                size = numbers(size_node.text, 3)
                if min(size) <= 0:
                    raise ValueError('World collision box sizes must be positive')
                # Keep the ground visual/physics but do not treat it as a wall.
                if cp[2] + size[2]/2 <= 0.025:
                    continue
                boxes.append((cp[0], cp[1], cp[5], size[0]/2, size[1]/2))
    if world.find('include') is not None:
        raise ValueError('World includes require expanding their collision geometry first')
    return boxes


def generate_scene(exp5_share, destination, spawn=(0.0, 0.0, 0.0, 0.0),
                   depth_rate=10.0, radius=0.45, gui=True):
    share, destination = Path(exp5_share), Path(destination)
    if not (0 < depth_rate <= 30 and 0 < radius < 1):
        raise ValueError('Invalid depth frequency or footprint radius')
    if len(spawn) != 4 or not all(math.isfinite(x) for x in spawn):
        raise ValueError('Spawn must contain x, y, z, yaw')
    if abs(spawn[2]) > 1e-8:
        raise ValueError('This planar teaching simulator requires robot_z=0')
    root = ET.parse(share / 'worlds/depth_camera.world')
    world = root.getroot().find('world')
    if world is None:
        raise ValueError('Experiment 5 world is missing')
    world.set('name', 'experiment6_navigation')
    # The experiment-5 floor ends at x=+/-6, y=+/-4.  Near the wall end and
    # goal, depth rays then leave the floor and become invalid.  Extend only
    # the generated teaching scene's ground plane;
    # keep the wall, robot, and planar movement bounds exactly as before.
    floor = world.find("model[@name='floor']")
    if floor is None:
        raise ValueError('Experiment 5 floor is missing')
    for size_node in floor.findall('.//geometry/box/size'):
        sx, sy, sz = numbers(size_node.text, 3)
        size_node.text = f'{max(sx,16.0)} {max(sy,16.0)} {sz}'
    if not gui:
        for child in world.findall('gui'):
            world.remove(child)
    boxes = world_obstacles(world)
    x, y, z, yaw = spawn
    if not (-5.5+radius < x < 5.5-radius and -3.5+radius < y < 3.5-radius):
        raise ValueError('Spawn is outside the teaching workspace')
    for bx, by, angle, hx, hy in boxes:
        c, s = math.cos(angle), math.sin(angle)
        if abs(c*(x-bx)+s*(y-by)) <= hx+radius and abs(-s*(x-bx)+c*(y-by)) <= hy+radius:
            raise ValueError('Robot spawn overlaps a scene obstacle')
    model = ET.parse(share / 'models/unitree_r1/model_depth.sdf').getroot().find('model')
    if model is None:
        raise ValueError('Experiment 5 R1 model is missing')
    model.set('name', 'r1')
    model_pose = model.find('pose')
    if model_pose is None:
        model_pose = ET.SubElement(model, 'pose')
    model_pose.text = f'{x} {y} {z} 0 0 {yaw}'
    static = model.find('static')
    if static is None:
        static = ET.SubElement(model, 'static')
    static.text = 'true'
    sensors = model.findall(".//sensor[@type='depth_camera']")
    if len(sensors) != 1:
        raise ValueError('Expected exactly one experiment-5 depth camera')
    sensors[0].find('update_rate').text = str(depth_rate)
    plugin = ET.SubElement(model, 'plugin',
        filename='libr1_planar_motion_system.so', name='experiment6::PlanarMotion')
    for key, value in {'footprint_radius':radius, 'command_timeout':0.35,
                       'publish_frequency':30.0, 'max_linear':0.35, 'max_angular':0.8}.items():
        ET.SubElement(plugin, key).text = str(value)
    for box in boxes:
        obstacle = ET.SubElement(plugin, 'obstacle')
        for key, value in zip(('x','y','yaw','half_x','half_y'), box):
            ET.SubElement(obstacle, key).text = str(value)
    world.append(model)
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    root.write(destination, encoding='utf-8', xml_declaration=True)
    return destination
