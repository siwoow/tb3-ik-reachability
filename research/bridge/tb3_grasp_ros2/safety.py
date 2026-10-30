"""Pure movement/sensor guards, testable without ROS or hardware."""

from dataclasses import dataclass
import math


class SafetyError(RuntimeError):
    """A latched stop condition; never automatically retried."""


def wrap(value):
    """Normalize radians."""
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Limits:
    """Hard upper bounds intentionally cannot be relaxed with CLI flags."""

    step_m: float = 0.045
    step_yaw_rad: float = math.radians(10)
    linear_mps: float = 0.020
    angular_rps: float = 0.100
    max_steps: int = 12
    max_distance_m: float = 0.30
    max_rotation_rad: float = math.radians(90)
    odom_timeout_s: float = 0.30
    scan_timeout_s: float = 0.50
    command_timeout_s: float = 0.15
    operation_timeout_s: float = 180.0

    def validate(self):
        """Reject nonfinite, unsafe or invalid user limits."""
        values = list(self.__dict__.values())
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('All limits must be finite and positive')
        if (self.step_m > 0.05 or self.linear_mps > 0.025 or
                self.angular_rps > 0.12 or self.step_yaw_rad > math.radians(10) or
                self.max_steps > 20 or self.max_distance_m > 0.50 or
                self.max_rotation_rad > math.pi or
                self.odom_timeout_s > 1 or self.scan_timeout_s > 1):
            raise ValueError('Requested limits exceed the fixed safety caps')


@dataclass(frozen=True)
class Step:
    """One forward translation or in-place rotation, followed by reobservation."""

    kind: str
    amount: float
    heading: float


def choose_step(current, goal, limits, remaining_distance):
    """Use forward-only scan_move semantics; never execute an entire base goal."""
    x, y, yaw = current
    gx, gy, gyaw = goal
    if not all(math.isfinite(v) for v in (*current, *goal)):
        raise SafetyError('Nonfinite base pose')
    distance = math.hypot(gx - x, gy - y)
    if distance <= 0.008:
        error = wrap(gyaw - yaw)
        if abs(error) <= math.radians(2):
            return None
    else:
        heading = math.atan2(gy - y, gx - x)
        error = wrap(heading - yaw)
        if abs(error) <= math.radians(2):
            amount = min(limits.step_m, distance, remaining_distance)
            if amount <= 0.004:
                raise SafetyError('Translation budget exhausted')
            return Step('translate', amount, yaw)
    angle = max(-limits.step_yaw_rad, min(limits.step_yaw_rad, error))
    return Step('rotate', angle, wrap(yaw + angle))


def step_command(step, start, pose, limits):
    """Return (linear, angular, done), with odometry overshoot/drift guards."""
    dx, dy = pose[0] - start[0], pose[1] - start[1]
    if step.kind == 'translate':
        traveled = dx * math.cos(start[2]) + dy * math.sin(start[2])
        lateral = -dx * math.sin(start[2]) + dy * math.cos(start[2])
        if math.hypot(dx, dy) > 0.05 + 1e-9:
            raise SafetyError('Measured step exceeded 5 cm; stopped, no retry')
        if traveled < -0.005 or abs(lateral) > 0.015:
            raise SafetyError('Reverse motion or lateral odometry deviation')
        error = wrap(step.heading - pose[2])
        if abs(error) > math.radians(10):
            raise SafetyError('Excessive heading drift')
        remaining = step.amount - traveled
        if remaining <= 0.002:
            return 0.0, 0.0, True
        return min(limits.linear_mps, max(0.005, remaining * 0.8)), max(-0.08, min(0.08, error)), False
    if step.kind != 'rotate':
        raise SafetyError('Unknown step type')
    if math.hypot(dx, dy) > 0.015:
        raise SafetyError('Unexpected translation during rotation')
    traveled = wrap(pose[2] - start[2])
    if abs(traveled) > limits.step_yaw_rad + math.radians(2):
        raise SafetyError('Rotation step overshoot')
    error = wrap(step.heading - pose[2])
    if abs(error) <= math.radians(1):
        return 0.0, 0.0, True
    return 0.0, max(-limits.angular_rps, min(limits.angular_rps, error * 0.8)), False


class Budget:
    """Count measured path length and absolute rotation, including settling."""

    def __init__(self, limits):
        self.limits = limits
        self.distance = 0.0
        self.rotation = 0.0
        self.steps = 0
        self.previous = None

    def observe(self, pose):
        """Accumulate the odometry path, not just start/end displacement."""
        if self.previous is not None:
            dx = math.hypot(pose[0] - self.previous[0], pose[1] - self.previous[1])
            dyaw = abs(wrap(pose[2] - self.previous[2]))
            if dx > 0.025 or dyaw > math.radians(15):
                raise SafetyError('Odometry jump, reset, or another motion source')
            self.distance += dx
            self.rotation += dyaw
        self.previous = pose
        if self.distance > self.limits.max_distance_m + 1e-9:
            raise SafetyError('Total translation budget exceeded')
        if self.rotation > self.limits.max_rotation_rad + 1e-9:
            raise SafetyError('Total rotation budget exceeded')

    def start_step(self, step):
        """Reserve the next step within all remaining bounds."""
        if self.steps >= self.limits.max_steps:
            raise SafetyError('Movement count limit reached')
        if step.kind == 'rotate' and self.rotation + abs(step.amount) > self.limits.max_rotation_rad:
            raise SafetyError('Insufficient remaining rotation budget')
        if step.kind == 'translate' and self.distance + step.amount > self.limits.max_distance_m + 1e-9:
            raise SafetyError('Insufficient remaining distance budget')
        self.steps += 1


def check_scan(ranges, angle_min, angle_increment, range_min, range_max,
               rotation, translation, turning=False):
    """Apply scan_move's 45/30 cm thresholds using actual scan->base TF too.

    Retain the sensor-range thresholds and check base-frame distances as well.
    Unknown/invalid sectors fail closed. This is a planar local guard, not a path
    collision proof or a sensor for table tops/drop-offs.
    """
    if not ranges or not math.isfinite(angle_increment) or angle_increment == 0:
        raise SafetyError('Invalid LiDAR scan')
    selected, valid, distances, bins = 0, 0, [], set()
    for i, distance in enumerate(ranges):
        a = angle_min + i * angle_increment
        unit = (math.cos(a), math.sin(a), 0)
        direction = [sum(rotation[j][k] * unit[k] for k in range(3)) for j in range(3)]
        bearing = wrap(math.atan2(direction[1], direction[0]))
        if not turning and abs(bearing) > math.radians(25):
            continue
        selected += 1
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        valid += 1
        p = [translation[j] + distance * direction[j] for j in range(3)]
        distances.append(min(distance, math.hypot(p[0], p[1])))
        bins.add(int((bearing + math.pi) / (2 * math.pi) * 12) % 12)
    if selected < 3 or valid < 3 or valid / selected < 0.8:
        raise SafetyError('Insufficient valid LiDAR coverage')
    if turning and len(bins) < 12:
        raise SafetyError('Full-circle LiDAR coverage required before rotation')
    if min(distances) < (0.30 if turning else 0.45):
        raise SafetyError('LiDAR obstacle inside scan_move safety threshold')


def permission_gate(execute_motion, execute_grasp, reachable):
    """Only live IK reachable=True permits grasp; it always ends base motion."""
    if reachable:
        return 'grasp' if execute_grasp else 'reachable_stop'
    return 'move' if execute_motion else 'dry_run_plan'
