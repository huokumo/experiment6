"""Deterministic controller checks; these do not replace Fortress validation."""
import math
import unittest

from experiment6_core.navigation import (CLEAR, BLOCKED, Pose2, Obstacles,
                                         NavigationConfig, WallBypass)


def virtual_depth(x, y, yaw, now):
    # A small synthetic counterpart to the experiment-5 wall: x=2.45,
    # y in [-2, 2].  The controller never receives these coordinates.
    hits = []
    for offset in (-0.17, -0.12, -0.06, 0.0, 0.06, 0.12, 0.17):
        angle = yaw + offset
        if math.cos(angle) > 0.01 and x < 2.45:
            distance = (2.45-x)/math.cos(angle)
            if -2.0 <= y+distance*math.sin(angle) <= 2.0:
                hits.append(distance)
    front = min(hits, default=4.0)
    state = BLOCKED if front <= 0.85 else CLEAR
    return Obstacles(True, (4.0, front, 4.0), (CLEAR, state, CLEAR), now)


class NavigationTests(unittest.TestCase):
    def test_default_scene_reaches_goal_without_crossing_wall(self):
        navigation = WallBypass()
        x = y = yaw = 0.0
        for tick in range(1800):
            now = tick*0.1
            pose = Pose2(x, y, yaw, now)
            cmd = navigation.step(now, pose, virtual_depth(x, y, yaw, now))
            next_x = x + cmd.vx*math.cos(yaw)*0.1
            next_y = y + cmd.vx*math.sin(yaw)*0.1
            self.assertFalse(2.0 < next_x < 3.0 and abs(next_y) < 2.45,
                             'the robot footprint approached the wall')
            x, y = next_x, next_y
            yaw = math.atan2(math.sin(yaw+cmd.omega*0.1),
                             math.cos(yaw+cmd.omega*0.1))
            if navigation.state in ('DONE', 'FAILED'):
                break
        self.assertEqual(navigation.state, 'DONE', navigation.reason)
        self.assertLess(math.hypot(4.2-x, y), 0.25)

    def test_unknown_depth_stops_then_fails(self):
        navigation = WallBypass()
        pose = Pose2(0.0, 0.0, 0.0, 0.0)
        good = virtual_depth(0.0, 0.0, 0.0, 0.0)
        navigation.step(0.0, pose, good)
        self.assertEqual(navigation.step(0.1, pose, None).vx, 0.0)
        self.assertEqual(navigation.step(8.2, pose, None).vx, 0.0)
        self.assertEqual(navigation.state, 'FAILED')

    def test_invalid_configuration(self):
        with self.assertRaises(ValueError):
            WallBypass(NavigationConfig(side='back'))


if __name__ == '__main__':
    unittest.main()
