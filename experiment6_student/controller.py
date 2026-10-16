"""Student TODO: fill the four decisions that produce /r1/cmd_vel.

The delivered starter is deliberately stationary.  Work only with Pose2 and
Obstacles received from public ROS topics.  Never read Gazebo wall geometry.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2:
    x: float                    # odom-frame metres
    y: float
    yaw: float                  # radians; positive turns left


@dataclass(frozen=True)
class Obstacles:
    data_valid: bool
    distance_m: tuple           # [LEFT, FRONT, RIGHT], metres
    state: tuple                # UNKNOWN=0, CLEAR=1, BLOCKED=2


@dataclass(frozen=True)
class Velocity:
    vx: float = 0.0             # forward m/s
    omega: float = 0.0          # positive left, rad/s


def goal_reached(pose: Pose2, goal_x: float, goal_y: float) -> bool:
    """TODO 1: return True when the robot is within 0.25 m of the goal."""
    # Hint: use math.hypot(goal_x-pose.x, goal_y-pose.y).
    return False


def front_is_safe(obstacles: Obstacles, clearance: float) -> bool:
    """TODO 2: require valid data, known CLEAR front, finite distance and margin."""
    # Never interpret UNKNOWN or NaN as free space.
    return False


def turn_toward(current_yaw: float, target_yaw: float) -> Velocity:
    """TODO 3: return a bounded angular command; stop within 0.07 rad."""
    # Hint: atan2(sin(error), cos(error)) wraps to [-pi, pi].
    # Bound |omega| to 0.35 rad/s; keep vx=0 while turning.
    return Velocity()


def decide_motion(pose: Pose2, obstacles: Obstacles, state: str,
                  goal_x: float, goal_y: float) -> tuple:
    """TODO 4: return (next_state, Velocity) at every 0.1 s tick.

    Suggested states: ADVANCE, TURN_SIDE, SIDE_ADVANCE, SCAN, CROSS,
    RETURN_GOAL, DONE, FAILED.  Save progress in a class if needed.  Turn the
    camera toward each travel direction; stop during a scan until a fresh
    obstacle summary arrives.  Stop on stale/unknown data in the ROS wrapper.
    """
    return 'TODO', Velocity()
