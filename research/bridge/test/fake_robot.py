"""Private-domain mock robot; never use its topics to control hardware."""

import argparse
import json
import math
import time
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from cleanup_interfaces.msg import ObjectObservation
from cleanup_interfaces.srv import CaptureObjects, EvaluateGrasp
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue


class FakeRobot(Node):
    """Provide synthetic sensors/services on /bridge_test/* only."""

    def __init__(self, output, mode):
        super().__init__('fake_grasp_robot')
        self.output, self.mode = Path(output), mode
        self.x = self.y = self.yaw = self.v = self.w = 0.
        self.last = time.monotonic()
        self.first_motion = None
        self.nonzero = self.zero = self.picks = self.captures = self.evaluations = 0
        self.odom = self.create_publisher(Odometry, '/bridge_test/odom', qos_profile_sensor_data)
        self.scan = self.create_publisher(LaserScan, '/bridge_test/scan', qos_profile_sensor_data)
        self.joints = self.create_publisher(JointState, '/bridge_test/joint_states', qos_profile_sensor_data)
        self.create_subscription(Twist, '/bridge_test/cmd_vel', self.command, 1)
        self.create_subscription(__import__('geometry_msgs.msg', fromlist=['PointStamped']).PointStamped,
                                 '/bridge_test/cleanup/pick_target', lambda msg: None, 10)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        transforms = []
        for parent, child, xyz in [('base_footprint', 'link1', [-.092, 0, .101]),
                                    ('base_footprint', 'base_scan', [-.024, 0, .132]),
                                    ('base_footprint', 'mock_camera_optical', [.10, 0, .30]),
                                    ('base_footprint', 'base_link', [0, 0, .01])]:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.header.stamp = self.get_clock().now().to_msg()
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
            t.transform.rotation.w = 1.
            if child == 'mock_camera_optical':
                t.transform.rotation.x, t.transform.rotation.y = -.5, .5
                t.transform.rotation.z, t.transform.rotation.w = -.5, .5
            transforms.append(t)
        self.static.sendTransform(transforms)
        self.create_service(CaptureObjects, '/bridge_test/cleanup/capture_objects', self.capture)
        self.create_service(EvaluateGrasp, '/bridge_test/cleanup/evaluate_grasp', self.evaluate)
        self.create_service(Trigger, '/bridge_test/execute_pick_and_place', self.pick)
        self.create_service(GetParameters, '/bridge_test/pick_and_place_action_node/get_parameters', self.parameters)
        self.create_timer(.02, self.tick)

    def save(self):
        self.output.write_text(json.dumps(dict(x=self.x, y=self.y, yaw=self.yaw, linear=self.v,
                                               angular=self.w, nonzero_commands=self.nonzero,
                                               zero_commands=self.zero, picks=self.picks,
                                               captures=self.captures, evaluations=self.evaluations), indent=2))

    def command(self, msg):
        self.v, self.w = msg.linear.x, msg.angular.z
        if self.v or self.w:
            self.first_motion = self.first_motion or time.monotonic()
            self.nonzero += 1
        else:
            self.zero += 1
        self.save()

    def tick(self):
        now = time.monotonic()
        dt, self.last = min(now-self.last, .05), now
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.v, self.w
        odom_lost = self.mode == 'odom_loss' and self.first_motion and now-self.first_motion > .15
        if not odom_lost:
            self.odom.publish(odom)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = odom.header, 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation = odom.pose.pose.orientation
        if not odom_lost:
            self.tf.sendTransform(tf)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint']
        joints.position, joints.velocity = [0., -1., .7, .3, .019], [0.]*5
        self.joints.publish(joints)
        if not (self.mode == 'scan_loss' and self.first_motion and now-self.first_motion > .15):
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_scan'
            scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/360
            scan.range_min, scan.range_max, scan.ranges = .05, 10., [2.]*360
            self.scan.publish(scan)
        self.save()

    def capture(self, request, response):
        self.captures += 1
        obj = ObjectObservation()
        # Camera/TF conversion is covered separately; use an existing odom
        # observation as the production perception server normally returns.
        obj.header.frame_id, obj.header.stamp = 'odom', self.get_clock().now().to_msg()
        obj.class_name, obj.object_uuid = 'banana', 'synthetic_banana'
        obj.confidence, obj.position_uncertainty = .95, .005
        obj.grasp_valid, obj.grasp_strategy = True, 'candidate_body'
        obj.centroid.x, obj.centroid.z = (.17 if self.mode in ('reachable', 'nav_only') else .32), .051
        if self.mode == 'nav_only':
            obj.grasp_strategy = 'candidate_approach'
        if self.mode == 'camera_tf':
            dx, dy = .32-self.x, -self.y
            bx = math.cos(self.yaw)*dx + math.sin(self.yaw)*dy
            by = -math.sin(self.yaw)*dx + math.cos(self.yaw)*dy
            obj.header.frame_id = 'mock_camera_optical'
            obj.centroid.x, obj.centroid.y, obj.centroid.z = -by, .249, bx-.10
        response.success, response.observations = True, [obj]
        return response

    def evaluate(self, request, response):
        self.evaluations += 1
        response.success = request.require_candidate
        if self.mode == 'service_error':
            response.success = False
            response.message = 'synthetic evaluation communication failure'
            return response
        local = request.target.point.x - self.x + .122
        probe = Path(__file__).resolve().parents[1]/'reference/native_probe'
        result = subprocess.run([str(probe), str(local), '0', '-.05', '-.101',
                                 '0', '-1', '.7', '.3', '.006'],
                                capture_output=True, text=True, check=True, timeout=3)
        response.reachable = self.mode in ('reachable', 'nav_only') or json.loads(result.stdout)['feasible']
        response.message = 'synthetic actual IK only'
        return response

    def pick(self, request, response):
        self.picks += 1
        response.success, response.message = True, 'synthetic pick, no hardware'
        self.save()
        return response

    def parameters(self, request, response):
        values = {'use_candidate_grasp': ParameterValue(type=1, bool_value=True),
                  'grasp_forward_offset': ParameterValue(type=3, double_value=.030),
                  'grasp_min_body_depth': ParameterValue(type=3, double_value=.006),
                  'target_topic': ParameterValue(type=4, string_value='/bridge_test/cleanup/pick_target'),
                  'floor_height': ParameterValue(type=3, double_value=0.)}
        response.values = [values[name] for name in request.names]
        return response


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True)
    p.add_argument('--mode', default='normal')
    args = p.parse_args()
    rclpy.init(args=[])
    node = FakeRobot(args.output, args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
