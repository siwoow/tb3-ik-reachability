"""Bounded stop-observe-evaluate loop; dry-run creates no hardware publishers."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import math
import multiprocessing
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger

from .planner_adapter import load_model, native_screen_worker, plan_worker, refine_goal_seeds
from .safety import Budget, Limits, SafetyError, check_scan, choose_step, step_command


def rotation(q):
    """Validate and convert a ROS quaternion without scipy."""
    x, y, z, w = q.x, q.y, q.z, q.w
    if not np.isfinite([x, y, z, w]).all() or abs(x*x+y*y+z*z+w*w-1) > .01:
        raise SafetyError('Invalid quaternion')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class Bridge(Node):
    """Reuse audited existing services; never launch existing motion scripts."""

    def __init__(self, options):
        super().__init__('tb3_grasp_bridge')
        from cleanup_interfaces.srv import CaptureObjects, EvaluateGrasp
        self.options = options
        self.limits = Limits(max_steps=options.max_steps, max_distance_m=options.max_distance)
        self.limits.validate()
        self.model = load_model(options.planner_root)
        self.budget = Budget(self.limits)
        self.lock = threading.RLock()
        self.fault = None
        self.moving = False
        self.tracking = False
        self.started = time.monotonic()
        self.last_command = self.started
        self.odom = self.scan = self.joints = None
        self.received = {}
        self.anchor = None
        self.stationary_pose = None
        self.last_stop_ros = 0
        self.pick_requested = False
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.log_path = Path(options.log).expanduser()
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.pool = ProcessPoolExecutor(1, mp_context=multiprocessing.get_context('spawn'))
        self.capture_type, self.eval_type = CaptureObjects, EvaluateGrasp
        self.capture_client = self.create_client(CaptureObjects, self.name('/cleanup/capture_objects'))
        self.eval_client = self.create_client(EvaluateGrasp, self.name('/cleanup/evaluate_grasp'))
        self.parameter_client = self.create_client(GetParameters, self.name('/pick_and_place_action_node/get_parameters'))
        self.create_subscription(Odometry, self.name('/odom'), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.name('/scan'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(JointState, self.name('/joint_states'), self.on_joints, qos_profile_sensor_data)
        self.command_pub = self.create_publisher(Twist, self.name('/cmd_vel'), 1) if options.execute_motion else None
        self.target_pub = self.pick_client = None
        self.stop_thread = threading.Event()
        self.watchdog = threading.Thread(target=self.watch, daemon=True)
        self.watchdog.start()
        self.mission_id = 'tb3_bridge_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.record('START', execute_motion=options.execute_motion, execute_grasp=options.execute_grasp,
                    limits=asdict(self.limits), collision_backend=self.model.collision_backend)

    def name(self, suffix):
        """Allow a private topic prefix for hardware-free integration tests."""
        return self.options.topic_prefix.rstrip('/') + suffix

    def record(self, event, **values):
        """Write an append-only local audit trail."""
        row = dict(event=event, monotonic_s=time.monotonic(), **values)
        with self.log_path.open('a') as stream:
            stream.write(json.dumps(row, ensure_ascii=False) + '\n')
        console = values
        if event == 'PLANNER':
            console = {'status': values['result']['status'], 'candidates': len(values['result']['candidates'])}
        self.get_logger().info(event + ' ' + json.dumps(console, ensure_ascii=False))

    def stamp_age(self, stamp):
        """Reject zero, future and stale ROS timestamps."""
        if stamp.sec == 0 and stamp.nanosec == 0:
            raise SafetyError('Zero sensor timestamp')
        return (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9

    def latch(self, reason):
        """A safety fault prevents every later nonzero velocity."""
        with self.lock:
            self.fault = self.fault or str(reason)
            self.moving = False
            if self.command_pub is not None:
                self.command_pub.publish(Twist())

    def on_odom(self, msg):
        """Track actual odometry distance and rotation, including settling."""
        try:
            if msg.header.frame_id.lstrip('/') != 'odom' or msg.child_frame_id.lstrip('/') not in ('base_footprint', 'base_link'):
                raise SafetyError('Unexpected odometry frames')
            r = rotation(msg.pose.pose.orientation)
            p = msg.pose.pose.position
            pose = (p.x, p.y, math.atan2(r[1, 0], r[0, 0]))
            if not np.isfinite([*pose, p.z]).all():
                raise SafetyError('Nonfinite odometry')
            with self.lock:
                if self.stationary_pose is not None and not self.moving and not self.tracking:
                    if (math.hypot(pose[0]-self.stationary_pose[0], pose[1]-self.stationary_pose[1]) > .005 or
                            abs(math.atan2(math.sin(pose[2]-self.stationary_pose[2]), math.cos(pose[2]-self.stationary_pose[2]))) > math.radians(1)):
                        raise SafetyError('Unexpected base motion while observing/planning/grasping')
                if self.tracking:
                    self.budget.observe(pose)
                else:
                    self.budget.previous = pose
                self.odom = (msg, pose)
                self.received['odom'] = time.monotonic()
        except Exception as error:
            self.latch(error)

    def on_scan(self, msg):
        """Receive scan data; safety validation uses its timestamped TF."""
        self.scan = msg
        self.received['scan'] = time.monotonic()

    def on_joints(self, msg):
        """Receive joint feedback without issuing arm commands."""
        self.joints = msg
        self.received['joints'] = time.monotonic()
        if self.moving and any(not math.isfinite(v) or abs(v) > .03 for n, v in zip(msg.name, msg.velocity) if n in ('joint1', 'joint2', 'joint3', 'joint4')):
            self.latch('Arm moved during a base step')

    def watch(self):
        """Independent heartbeat watchdog keeps stopped phases at zero velocity."""
        while not self.stop_thread.wait(.05):
            try:
                with self.lock:
                    if self.command_pub is None:
                        continue
                    now = time.monotonic()
                    if self.moving and (now-self.last_command > self.limits.command_timeout_s or
                            now-self.received.get('odom', 0) > self.limits.odom_timeout_s or
                            now-self.received.get('scan', 0) > self.limits.scan_timeout_s):
                        self.fault = self.fault or 'Watchdog: command/odometry/LiDAR timeout'
                        self.moving = False
                    if not self.moving or self.fault:
                        self.command_pub.publish(Twist())
            except Exception as error:
                self.fault = self.fault or 'Watchdog publisher failure: ' + str(error)

    def pump(self, sensor_check=True):
        """Service waiting never blocks sensor processing."""
        rclpy.spin_once(self, timeout_sec=.02)
        if self.fault:
            raise SafetyError(self.fault)
        if not rclpy.ok() or time.monotonic()-self.started > self.limits.operation_timeout_s:
            raise SafetyError('ROS shutdown or operation time limit')
        if sensor_check:
            self.fresh()

    def fresh(self):
        """Require fresh receipt times AND source timestamps."""
        now = time.monotonic()
        for key, timeout in (('odom', self.limits.odom_timeout_s), ('scan', self.limits.scan_timeout_s), ('joints', .25)):
            if now-self.received.get(key, 0) > timeout:
                raise SafetyError(key + ' stream stale/unavailable')
        for stamp, limit in ((self.odom[0].header.stamp, self.limits.odom_timeout_s),
                             (self.scan.header.stamp, self.limits.scan_timeout_s),
                             (self.joints.header.stamp, .25)):
            if not -.02 <= self.stamp_age(stamp) <= limit:
                raise SafetyError('Stale or future sensor source timestamp')

    def call(self, client, request, timeout=10):
        """Do not retry failed calls, especially non-cancellable pick services."""
        deadline = time.monotonic() + timeout
        while not client.service_is_ready():
            self.pump()
            if time.monotonic() > deadline:
                raise SafetyError('Service unavailable: ' + client.srv_name)
        future = client.call_async(request)
        while not future.done():
            self.pump()
            if time.monotonic() > deadline:
                future.cancel()
                raise SafetyError('Service timeout: ' + client.srv_name)
        result = future.result()
        if result is None:
            raise SafetyError('No service response: ' + client.srv_name)
        return result

    def lookup(self, target, source, stamp):
        """Use the observation timestamp; never fall back to latest camera TF."""
        deadline = time.monotonic() + .5
        while not self.tf.can_transform(target, source, Time.from_msg(stamp)):
            self.pump()
            if time.monotonic() > deadline:
                raise SafetyError(f'Timestamped TF missing: {source}->{target}')
        return self.tf.lookup_transform(target, source, Time.from_msg(stamp))

    def pose_and_mount(self):
        """Use real TF for base_footprint and verify the modeled arm mount."""
        msg, odom_pose = self.odom
        base = self.lookup('odom', 'base_footprint', msg.header.stamp).transform
        r = rotation(base.rotation)
        pose = (base.translation.x, base.translation.y, math.atan2(r[1, 0], r[0, 0]))
        if np.linalg.norm(r[2, :2]) > .02 or np.linalg.norm(np.array(pose[:2])-odom_pose[:2]) > .02:
            raise SafetyError('Nonlevel floor or inconsistent odometry/TF')
        mount = self.lookup('base_footprint', 'link1', msg.header.stamp).transform
        actual = np.array([mount.translation.x, mount.translation.y, mount.translation.z])
        expected = self.model.fk([[0, 0, 0, 0]])['link1'][0]
        if np.linalg.norm(actual-expected[:3, 3]) > .003 or not np.allclose(rotation(mount.rotation), expected[:3, :3], atol=.003):
            raise SafetyError('Live arm mount differs from analysis URDF; regenerate model')
        return pose, base.translation.z, actual

    def laser_safe(self, turning=False):
        """Preserve scan_move thresholds with actual scan extrinsics."""
        self.fresh()
        scan = self.scan
        tf = self.lookup('base_footprint', scan.header.frame_id, scan.header.stamp).transform
        check_scan(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
                   rotation(tf.rotation), (tf.translation.x, tf.translation.y, tf.translation.z), turning)

    def arm_state(self):
        """Keep real executor limits; never widen them to offline URDF bounds."""
        self.fresh()
        values = dict(zip(self.joints.name, self.joints.position))
        q = [values['joint' + str(i)] for i in range(1, 5)]
        opening = values['gripper_left_joint']
        velocities = dict(zip(self.joints.name, self.joints.velocity))
        if any(not math.isfinite(velocities.get('joint'+str(i), math.inf)) or abs(velocities['joint'+str(i)]) > .03 for i in range(1, 5)):
            raise SafetyError('Fresh stationary arm velocity feedback required before base motion')
        if not np.isfinite([*q, opening]).all():
            raise SafetyError('Invalid arm feedback')
        lower = np.array([-2.8, -1.75, -.92, -1.75])
        upper = np.array([2.8, 1.55, 1.35, 2.0])
        if np.any(q < lower) or np.any(q > upper):
            raise SafetyError('Arm outside unchanged executor limits')
        return q, opening

    def stop_and_settle(self):
        """Repeated zero commands, then measured stationary feedback."""
        with self.lock:
            self.moving = False
            if self.command_pub is not None:
                self.command_pub.publish(Twist())
        stable = None
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            self.pump()
            twist = self.odom[0].twist.twist
            speed = math.hypot(twist.linear.x, twist.linear.y)
            if not math.isfinite(speed) or not math.isfinite(twist.angular.z):
                raise SafetyError('Invalid velocity feedback')
            if speed < .004 and abs(twist.angular.z) < .02:
                stable = stable or time.monotonic()
                if time.monotonic()-stable >= .5:
                    self.tracking = False
                    self.stationary_pose = self.odom[1]
                    self.last_stop_ros = self.get_clock().now().nanoseconds
                    self.laser_safe(turning=False)
                    return
            else:
                stable = None
        raise SafetyError('Base failed to settle after stop')

    def initialize(self):
        """Read actual execution parameters and audited source fingerprints."""
        deadline = time.monotonic() + 8
        while any(x is None for x in (self.odom, self.scan, self.joints)):
            self.pump(sensor_check=False)
            if time.monotonic() > deadline:
                raise SafetyError('Initial sensor timeout')
        self.stop_and_settle()
        names = ['use_candidate_grasp', 'grasp_forward_offset', 'grasp_min_body_depth', 'target_topic', 'floor_height']
        response = self.call(self.parameter_client, GetParameters.Request(names=names))
        if len(response.values) != 5 or [v.type for v in response.values] != [1, 3, 3, 4, 3]:
            raise SafetyError('Unexpected executor parameter contract')
        candidate, offset, depth, topic, floor = response.values
        if not candidate.bool_value or not 0 <= offset.double_value <= .030 or not .004 <= depth.double_value <= .020:
            raise SafetyError('Candidate executor configuration is incompatible')
        self.forward_offset, self.min_depth, self.floor = offset.double_value, depth.double_value, floor.double_value
        if abs(self.floor) > .001:
            raise SafetyError('Only the audited flat floor z=0 configuration is supported')
        self.target_topic = topic.string_value
        if self.target_topic != self.name('/cleanup/pick_target'):
            raise SafetyError('Unexpected executor target topic')
        contract = json.loads(Path(self.options.contract).expanduser().read_text())
        source_root = Path(self.options.executor_source_root).expanduser()
        for relative, expected in contract['executor_header_sha256'].items():
            if hashlib.sha256((source_root/relative).read_bytes()).hexdigest() != expected:
                raise SafetyError('Executor headers changed; audit again: ' + relative)
        executable = Path(self.options.native_probe).expanduser()
        if not executable.is_file():
            raise SafetyError('Build the read-only native_probe first')
        manifest = json.loads(executable.with_suffix('.manifest.json').read_text())
        if (manifest['binary_sha256'] != hashlib.sha256(executable.read_bytes()).hexdigest() or
                manifest['executor_header_sha256'] != contract['executor_header_sha256']):
            raise SafetyError('Native helper build/source fingerprints do not match')
        self.native_probe = executable
        if self.options.execute_grasp:
            self.target_pub = self.create_publisher(PointStamped, self.target_topic, 10)
            self.pick_client = self.create_client(Trigger, self.name('/execute_pick_and_place'))
        self.pose_and_mount()
        self.record('PREFLIGHT_OK', forward_offset_m=self.forward_offset, min_body_depth_m=self.min_depth,
                    target_topic=self.target_topic, actual_ik_equivalent=False)

    def observe(self):
        """Reuse grasp_trial capture; reject stale, ambiguous or changed targets."""
        request = self.capture_type.Request(mission_id=self.mission_id, station_name='tb3_bridge',
                                            heading_index=249, burst_frames=5, min_confirmations=1)
        response = self.call(self.capture_client, request, timeout=30)
        if not response.success:
            raise SafetyError('Capture failed: ' + response.message)
        candidates = [o for o in response.observations if o.class_name == 'banana' and o.grasp_valid
                      and o.grasp_strategy in ('candidate_body', 'candidate_approach')]
        if len(candidates) != 1:
            raise SafetyError(f'Expected exactly one candidate banana, got {len(candidates)}')
        obj = candidates[0]
        self.observation_strategy = obj.grasp_strategy
        if not 0 <= obj.position_uncertainty <= .03 or obj.confidence < .5:
            raise SafetyError('Detection confidence/uncertainty gate failed')
        target = PointStamped(header=obj.header, point=obj.centroid)
        if not target.header.frame_id or not 0 <= self.stamp_age(target.header.stamp) <= 2:
            raise SafetyError('Observation missing frame or too old')
        if Time.from_msg(target.header.stamp).nanoseconds < self.last_stop_ros:
            raise SafetyError('Observation predates the last completed stop')
        if target.header.frame_id != 'odom':
            target = do_transform_point(target, self.lookup('odom', target.header.frame_id, target.header.stamp))
            target.header.stamp = obj.header.stamp
        point = np.array([target.point.x, target.point.y, target.point.z])
        if not np.isfinite(point).all():
            raise SafetyError('Nonfinite transformed target')
        if self.anchor is not None and np.linalg.norm(point-self.anchor) > .04:
            raise SafetyError('Object identity/position changed after movement')
        self.anchor = point if self.anchor is None else self.anchor
        self.record('OBSERVATION', uuid=obj.object_uuid, source_frame=obj.header.frame_id,
                    target_odom=point.tolist(), strategy=obj.grasp_strategy)
        return target

    def native_check(self, target_xyz, pose, base_z, mount, q, opening):
        """Filter with the original C++ candidate IK and supplemental FCL checks."""
        from tb3_grasp.planner import transform_object_to_base
        local = transform_object_to_base(target_xyz, pose, base_z)-mount
        local[0] += self.forward_offset
        future = self.pool.submit(native_screen_worker, self.options.planner_root, str(self.native_probe),
                                  local, self.floor-base_z-mount[2], q, opening, self.min_depth, self.floor-base_z)
        deadline = time.monotonic() + 10
        while not future.done():
            self.pump()
            if time.monotonic() > deadline:
                future.cancel()
                raise SafetyError('Native path validation timed out')
        return future.result()

    def move_step(self, step):
        """Publish only a bounded segment; every segment returns through a stop."""
        if self.command_pub is None:
            raise SafetyError('Movement is disabled')
        self.laser_safe(step.kind == 'rotate')
        self.budget.start_step(step)
        start = self.odom[1]
        self.tracking = True
        self.budget.previous = start
        deadline = time.monotonic() + 12
        self.record('STEP_START', step=asdict(step), count=self.budget.steps)
        try:
            while True:
                self.pump()
                self.laser_safe(step.kind == 'rotate')
                if self.count_publishers(self.name('/cmd_vel')) != 1 or self.count_subscribers(self.name('/cmd_vel')) < 1:
                    raise SafetyError('Missing base subscriber or competing cmd_vel publisher')
                linear, angular, done = step_command(step, start, self.odom[1], self.limits)
                if done:
                    break
                if time.monotonic() > deadline:
                    raise SafetyError('Step timeout')
                if self.moving and time.monotonic()-self.last_command < .05:
                    continue
                with self.lock:
                    if self.fault:
                        raise SafetyError(self.fault)
                    self.moving = True
                    self.last_command = time.monotonic()
                    command = Twist()
                    command.linear.x, command.angular.z = linear, angular
                    self.command_pub.publish(command)
        finally:
            with self.lock:
                self.moving = False
                self.command_pub.publish(Twist())
        self.stop_and_settle()
        if math.hypot(self.odom[1][0]-start[0], self.odom[1][1]-start[1]) > .05:
            raise SafetyError('Post-stop translation exceeded 5 cm')
        self.record('STEP_STOPPED', distance_m=self.budget.distance, rotation_rad=self.budget.rotation)

    def run(self):
        """Execute stop -> scan -> new observation -> actual IK for every step."""
        self.initialize()
        while True:
            self.laser_safe()
            target = self.observe()
            evaluation = self.call(self.eval_client, self.eval_type.Request(target=target, require_candidate=True))
            if not evaluation.success:
                raise SafetyError('Actual IK evaluation failed: ' + evaluation.message)
            self.record('ACTUAL_IK', reachable=evaluation.reachable, message=evaluation.message)
            xyz = [target.point.x, target.point.y, target.point.z]
            pose, base_z, mount = self.pose_and_mount()
            q, opening = self.arm_state()
            if evaluation.reachable:
                self.stop_and_settle()
                if not self.options.execute_grasp:
                    self.record('REACHABLE_STOP', grasp_enabled=False)
                    return
                if self.observation_strategy != 'candidate_body':
                    raise SafetyError('candidate_approach is navigation-only; no grasp execution allowed')
                if self.native_check(xyz, pose, base_z, mount, q, opening) is None:
                    raise SafetyError('Supplemental native/FCL path check failed; no grasp requested')
                # New observation/evaluation after the final stop and verification.
                target = self.observe()
                if self.observation_strategy != 'candidate_body':
                    raise SafetyError('Final observation is not candidate_body; no grasp execution allowed')
                checked = self.call(self.eval_client, self.eval_type.Request(target=target, require_candidate=True))
                if not checked.success or not checked.reachable:
                    raise SafetyError('Final real IK changed; stopped, no grasp')
                pose, base_z, mount = self.pose_and_mount()
                q, opening = self.arm_state()
                if self.native_check([target.point.x, target.point.y, target.point.z], pose, base_z, mount, q, opening) is None:
                    raise SafetyError('Final target path fails supplemental collision check')
                if self.stamp_age(target.header.stamp) > 2:
                    raise SafetyError('Final grasp target expired')
                if self.target_pub.get_subscription_count() < 1:
                    raise SafetyError('Pick executor target subscription unavailable')
                self.target_pub.publish(target)
                deadline = time.monotonic() + .2
                while time.monotonic() < deadline:
                    self.pump()
                self.pick_requested = True
                result = self.call(self.pick_client, Trigger.Request(), timeout=45)
                if not result.success:
                    raise SafetyError('Existing pick executor failed: ' + result.message)
                self.record('PICK_RESULT', success=True, message=result.message)
                return
            future = self.pool.submit(plan_worker, self.options.planner_root, xyz, pose, base_z,
                                      self.forward_offset, self.options.object_width,
                                      self.limits.max_distance_m-self.budget.distance)
            deadline = time.monotonic() + 30
            while not future.done():
                self.pump()
                if time.monotonic() > deadline:
                    future.cancel()
                    raise SafetyError('Planner computation timeout')
            plan = future.result()
            self.record('PLANNER', result=plan)
            selected = None
            for candidate in refine_goal_seeds(plan, pose, self.limits.max_distance_m-self.budget.distance):
                goal = candidate['goal']
                if self.native_check(xyz, goal, base_z, mount, q, opening) is not None:
                    selected = goal
                    self.record('NATIVE_REFINED_GOAL', **candidate)
                    break
            if selected is None:
                raise SafetyError('No candidate passes both offline and unchanged real-executor IK/FCL')
            step = choose_step(pose, selected, self.limits, self.limits.max_distance_m-self.budget.distance)
            if step is None:
                raise SafetyError('Planner says arrived but real IK is false; no blind nudge')
            self.record('PROPOSED_STEP', base_goal=selected, step=asdict(step))
            if not self.options.execute_motion:
                self.record('DRY_RUN_COMPLETE', hardware_publishers_created=self.target_pub is not None,
                            execute_grasp=self.options.execute_grasp)
                return
            self.move_step(step)

    def close(self):
        """Stop, best-effort cancel an issued pick on error, and release resources."""
        if self.fault and self.pick_requested:
            from action_msgs.srv import CancelGoal
            for action in ('/arm_controller/follow_joint_trajectory', '/gripper_controller/gripper_cmd'):
                client = self.create_client(CancelGoal, self.name(action+'/_action/cancel_goal'))
                if client.service_is_ready():
                    client.call_async(CancelGoal.Request())
            # Trigger has no cancellation protocol. These action cancellations
            # are best effort, never a claimed hardware stop guarantee.
        self.moving = False
        if self.command_pub is not None and rclpy.ok():
            for _ in range(10):
                self.command_pub.publish(Twist())
                rclpy.spin_once(self, timeout_sec=.02)
        self.stop_thread.set()
        self.watchdog.join(timeout=1)
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.destroy_node()


def main(argv=None):
    """Real motion and grasp require separate explicit flags."""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-motion', action='store_true')
    parser.add_argument('--execute-grasp', action='store_true')
    parser.add_argument('--dry-run', action='store_true', help='Explicit default: no velocity/target publishers')
    parser.add_argument('--planner-root', default=str(root/'planner_runtime'))
    parser.add_argument('--executor-source-root', default='~/turtlebot3_ws/src/pick_and_place')
    parser.add_argument('--native-probe', default=str(root/'native_probe'))
    parser.add_argument('--contract', default=str(root/'executor_contract.json'))
    parser.add_argument('--max-steps', type=int, default=12)
    parser.add_argument('--max-distance', type=float, default=.30)
    parser.add_argument('--object-width', type=float, default=.030)
    parser.add_argument('--topic-prefix', default='')
    parser.add_argument('--log', default=str(root/'results/bridge_events.jsonl'))
    options = parser.parse_args(argv)
    if options.dry_run and (options.execute_motion or options.execute_grasp):
        parser.error('--dry-run cannot be combined with execution flags')
    rclpy.init(args=[])
    node = None
    status = 0
    try:
        node = Bridge(options)
        node.run()
    except (Exception, KeyboardInterrupt) as error:
        status = 1
        if node is not None:
            node.latch(error)
            node.record('STOP_ERROR', reason=str(error), no_automatic_retry=True)
        else:
            print('Preflight failed:', error)
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(status)


if __name__ == '__main__':
    main()
