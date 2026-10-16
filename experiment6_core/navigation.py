"""Small depth-guided wall bypass for the Experiment 6 teaching scene.

The controller sees odometry and the public obstacle summary only.  It never
reads the simulator's collision boxes.  Every motion direction is observed by
turning the depth camera toward it before advancing.
"""
from dataclasses import dataclass
import math

UNKNOWN, CLEAR, BLOCKED = 0, 1, 2
LEFT, FRONT, RIGHT = 0, 1, 2


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float
    received_at: float


@dataclass(frozen=True)
class Obstacles:
    valid: bool
    distances: tuple
    states: tuple
    received_at: float


@dataclass(frozen=True)
class Command:
    vx: float = 0.0
    omega: float = 0.0


@dataclass(frozen=True)
class NavigationConfig:
    goal_x: float = 4.2
    goal_y: float = 0.0
    goal_yaw: float = 0.0
    side: str = 'left'
    approach_distance: float = 1.45
    travel_clearance: float = 1.0
    wall_absent_distance: float = 3.0
    side_step: float = 0.42
    edge_clearance_step: float = 0.55
    max_side_travel: float = 3.0
    cross_margin: float = 0.85
    goal_tolerance: float = 0.25
    linear_speed: float = 0.15
    angular_speed: float = 0.35
    sensor_timeout: float = 1.5
    sensor_grace: float = 8.0
    max_duration: float = 600.0
    stall_timeout: float = 5.0

    def validate(self):
        if self.side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        numeric = [value for key, value in self.__dict__.items() if key != 'side']
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError('navigation parameters must be finite')
        if not (0 < self.goal_tolerance < self.travel_clearance and
                self.travel_clearance < self.approach_distance < self.wall_absent_distance and
                0 < self.side_step < self.max_side_travel <= 3.0 and
                0.32 <= self.edge_clearance_step <= 0.65 and
                0 < self.cross_margin and 0 < self.linear_speed <= 0.25 and
                0 < self.angular_speed <= 0.6 and 0 < self.sensor_timeout < self.sensor_grace and
                self.max_duration > self.sensor_grace and self.stall_timeout > 0):
            raise ValueError('invalid navigation limits')


class WallBypass:
    """Advance, probe a wall edge, cross, and return to a chosen goal.

    `step` is deterministic for a supplied time, pose and depth summary.  A
    zero command is returned on sensor loss, blocked travel, or completion.
    """
    def __init__(self, config=None):
        self.config = config or NavigationConfig()
        self.config.validate()
        self.state = 'WAIT_SENSORS'
        self.reason = 'waiting_for_pose_and_depth'
        self.started_at = None
        self.entered_at = None
        self.gap_since = None
        self.last_progress_at = None
        self.last_progress_pose = None
        self.approach_heading = 0.0
        self.cross_target = 0.0
        self.segment_start = None
        self.lateral_total = 0.0
        self.clear_scans = 0

    def enter(self, state, now, reason=''):
        self.state = state
        self.entered_at = now
        self.reason = reason or state.lower()
        self.last_progress_at = now
        self.last_progress_pose = None

    def fail(self, now, reason):
        self.enter('FAILED', now, reason)
        return Command()

    def heading_command(self, pose, target):
        error = wrap(target - pose.yaw)
        if abs(error) <= 0.07:
            return None
        return Command(0.0, clamp(1.5 * error, self.config.angular_speed))

    def advance_command(self, pose, obstacles, heading, now, collision):
        if collision:
            return self.fail(now, 'collision_guard_blocked')
        front = obstacles.distances[FRONT]
        if (obstacles.states[FRONT] != CLEAR or not math.isfinite(front) or
                front < self.config.travel_clearance):
            return self.fail(now, 'travel_direction_blocked_or_unknown')
        correction = wrap(heading - pose.yaw)
        if abs(correction) > 0.20:
            return self.heading_command(pose, heading)
        speed = min(self.config.linear_speed, max(0.06, 0.35 * (front - self.config.travel_clearance)))
        return Command(speed, clamp(1.3 * correction, self.config.angular_speed))

    def check_progress(self, command, pose, now):
        if command.vx == 0.0 and command.omega == 0.0:
            self.last_progress_at = now
            self.last_progress_pose = pose
            return False
        previous = self.last_progress_pose
        if (previous is None or math.hypot(pose.x-previous.x, pose.y-previous.y) >= 0.035 or
                abs(wrap(pose.yaw-previous.yaw)) >= 0.035):
            self.last_progress_at = now
            self.last_progress_pose = pose
        return now - self.last_progress_at > self.config.stall_timeout

    def step(self, now, pose, obstacles, collision=False):
        if self.state in ('DONE', 'FAILED'):
            return Command()
        if self.started_at is None:
            self.started_at = now
        if now - self.started_at > self.config.max_duration:
            return self.fail(now, 'task_timeout')
        healthy = (pose is not None and obstacles is not None and obstacles.valid and
                   now-pose.received_at <= self.config.sensor_timeout and
                   now-obstacles.received_at <= self.config.sensor_timeout and
                   len(obstacles.distances) == 3 and len(obstacles.states) == 3 and
                   all(state != UNKNOWN for state in obstacles.states))
        if not healthy:
            if self.gap_since is None:
                self.gap_since = now
            if now-self.gap_since > self.config.sensor_grace:
                return self.fail(now, 'pose_or_depth_unavailable')
            self.reason = 'waiting_for_fresh_pose_and_depth'
            return Command()
        self.gap_since = None
        if self.state == 'WAIT_SENSORS':
            self.enter('ADVANCE', now)
            return Command()
        cfg = self.config
        goal_distance = math.hypot(cfg.goal_x-pose.x, cfg.goal_y-pose.y)
        front = obstacles.distances[FRONT]
        if self.state == 'ADVANCE':
            if goal_distance <= cfg.goal_tolerance:
                self.enter('ALIGN_GOAL', now)
                return Command()
            if not math.isfinite(front) or obstacles.states[FRONT] == UNKNOWN:
                return self.fail(now, 'front_observation_unknown')
            if front <= cfg.approach_distance:
                self.approach_heading = math.atan2(cfg.goal_y-pose.y, cfg.goal_x-pose.x)
                forward = pose.x*math.cos(self.approach_heading)+pose.y*math.sin(self.approach_heading)
                self.cross_target = forward+front+cfg.cross_margin
                self.enter('TURN_SIDE', now)
                return Command()
            target = math.atan2(cfg.goal_y-pose.y, cfg.goal_x-pose.x)
            command = self.heading_command(pose, target)
            if command is None:
                command = self.advance_command(pose, obstacles, target, now, collision)
        elif self.state == 'TURN_SIDE':
            side_heading = wrap(self.approach_heading+(math.pi/2 if cfg.side == 'left' else -math.pi/2))
            command = self.heading_command(pose, side_heading)
            if command is None:
                self.segment_start = (pose.x, pose.y)
                self.enter('SIDE_ADVANCE', now)
                return Command()
        elif self.state == 'SIDE_ADVANCE':
            side_heading = wrap(self.approach_heading+(math.pi/2 if cfg.side == 'left' else -math.pi/2))
            dx, dy = pose.x-self.segment_start[0], pose.y-self.segment_start[1]
            progress = dx*math.cos(side_heading)+dy*math.sin(side_heading)
            step_length = (min(cfg.edge_clearance_step, cfg.max_side_travel-self.lateral_total)
                           if self.clear_scans else cfg.side_step)
            if progress >= step_length:
                self.lateral_total += progress
                self.enter('TURN_SCAN', now)
                return Command()
            command = self.advance_command(pose, obstacles, side_heading, now, collision)
        elif self.state == 'TURN_SCAN':
            command = self.heading_command(pose, self.approach_heading)
            if command is None:
                self.enter('SCAN', now)
                return Command()
        elif self.state == 'SCAN':
            # Wait for a depth frame acquired after the camera finished turning.
            if now-self.entered_at < 0.35 or obstacles.received_at <= self.entered_at:
                return Command()
            if obstacles.states[FRONT] == CLEAR and math.isfinite(front) and front >= cfg.wall_absent_distance:
                self.clear_scans += 1
            else:
                self.clear_scans = 0
            if self.clear_scans >= 2:
                self.enter('CROSS', now)
            elif ((self.clear_scans and cfg.max_side_travel-self.lateral_total < 0.32) or
                  (not self.clear_scans and self.lateral_total+cfg.side_step > cfg.max_side_travel)):
                return self.fail(now, 'wall_edge_not_found_within_side_limit')
            else:
                self.enter('TURN_SIDE', now)
            return Command()
        elif self.state == 'CROSS':
            forward = pose.x*math.cos(self.approach_heading)+pose.y*math.sin(self.approach_heading)
            if forward >= self.cross_target:
                self.enter('RETURN_GOAL', now)
                return Command()
            command = self.heading_command(pose, self.approach_heading)
            if command is None:
                command = self.advance_command(pose, obstacles, self.approach_heading, now, collision)
        elif self.state == 'RETURN_GOAL':
            if goal_distance <= cfg.goal_tolerance:
                self.enter('ALIGN_GOAL', now)
                return Command()
            target = math.atan2(cfg.goal_y-pose.y, cfg.goal_x-pose.x)
            command = self.heading_command(pose, target)
            if command is None:
                command = self.advance_command(pose, obstacles, target, now, collision)
        elif self.state == 'ALIGN_GOAL':
            command = self.heading_command(pose, cfg.goal_yaw)
            if command is None:
                self.enter('DONE', now, 'goal_reached')
                return Command()
        else:
            return self.fail(now, 'invalid_controller_state')
        if self.check_progress(command, pose, now):
            return self.fail(now, 'no_pose_progress')
        return command
