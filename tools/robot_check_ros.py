#!/usr/bin/env python3
"""
ROS-side measurements for robot_check.py (runs inside the pixi environment).

Subscribes to the robot's core topics for a fixed window and prints one JSON
object: per-topic rates, IMU static statistics, and whether the
odom -> base_footprint transform resolves. Also offers small actions used by
robot_check.py's motion checks (arm nudge, gripper, base drive).

Usage (from robot_check.py, via ``pixi run python``)::

    robot_check_ros.py measure [seconds]
    robot_check_ros.py joints
    robot_check_ros.py nudge <S1|S2|S3> <delta_rad>
    robot_check_ros.py gripper <position_m>
    robot_check_ros.py drive <linear_x> <angular_z> <seconds>
    robot_check_ros.py pubtest <linear_x> <angular_z> <seconds>
"""

import json
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

TOPICS = {
    'odom': ('/odom', 'nav_msgs.msg', 'Odometry', False),
    'imu': ('/imu/data_raw', 'sensor_msgs.msg', 'Imu', True),
    'scan': ('/scan', 'sensor_msgs.msg', 'LaserScan', True),
    'joint_states': ('/joint_states', 'sensor_msgs.msg', 'JointState', False),
}
ARM_JOINTS = ['S1_joint', 'S2_joint', 'S3_joint']


def _msg_class(module, name):
    mod = __import__(module, fromlist=[name])
    return getattr(mod, name)


def measure(seconds):
    """Subscribe to every core topic at once and report rates and IMU stats."""
    node = rclpy.create_node('robot_check_measure')
    stamps = {k: [] for k in TOPICS}
    imu = {'ax': [], 'ay': [], 'az': [], 'gmax': []}

    def make_cb(key):
        def cb(msg):
            stamps[key].append(time.monotonic())
            if key == 'imu':
                a, g = msg.linear_acceleration, msg.angular_velocity
                imu['ax'].append(a.x)
                imu['ay'].append(a.y)
                imu['az'].append(a.z)
                imu['gmax'].append(max(abs(g.x), abs(g.y), abs(g.z)))
        return cb

    for key, (topic, module, name, sensor_qos) in TOPICS.items():
        qos = (QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200,
                          reliability=ReliabilityPolicy.BEST_EFFORT)
               if sensor_qos else 200)
        node.create_subscription(_msg_class(module, name), topic, make_cb(key), qos)

    from tf2_ros import Buffer, TransformListener
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    out = {'rates': {}}
    for key, ts in stamps.items():
        n = len(ts)
        out['rates'][key] = (n - 1) / (ts[-1] - ts[0]) if n > 1 and ts[-1] > ts[0] else 0.0
    if imu['ax']:
        mean = [sum(v) / len(v) for v in (imu['ax'], imu['ay'], imu['az'])]
        out['imu'] = {
            'accel_mean': mean,
            'accel_norm': math.sqrt(sum(m * m for m in mean)),
            'gyro_max': max(imu['gmax']),
            'gravity_axis': ['x', 'y', 'z'][max(range(3), key=lambda i: abs(mean[i]))],
            'gravity_sign': 1 if mean[max(range(3), key=lambda i: abs(mean[i]))] > 0 else -1,
        }
    try:
        from rclpy.time import Time
        out['tf_odom_base'] = tf_buffer.can_transform('odom', 'base_footprint', Time())
    except Exception:  # noqa: BLE001 - report as missing, never crash the check
        out['tf_odom_base'] = False
    node.destroy_node()
    return out


def joints(timeout=5.0):
    """Return the latest position of every joint, or {} on timeout."""
    from sensor_msgs.msg import JointState
    node = rclpy.create_node('robot_check_joints')
    got = {}
    node.create_subscription(
        JointState, '/joint_states', lambda m: got.update(dict(zip(m.name, m.position))), 10)
    end = time.monotonic() + timeout
    while time.monotonic() < end and not all(j in got for j in ARM_JOINTS):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    return got


def _send_goal(action_type, action_name, goal, timeout=30.0):
    node = rclpy.create_node('robot_check_action')
    client = ActionClient(node, action_type, action_name)
    if not client.wait_for_server(timeout_sec=5.0):
        node.destroy_node()
        return {'ok': False, 'error': f'{action_name} not available'}
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    handle = fut.result()
    if handle is None:
        node.destroy_node()
        return {'ok': False, 'error': 'no answer to the goal within 10 s'}
    if not handle.accepted:
        node.destroy_node()
        return {'ok': False, 'error': 'goal rejected'}
    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=timeout)
    res = res_fut.result()
    if res is None:
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
        node.destroy_node()
        return {'ok': False, 'error': 'timed out waiting for the result; goal cancelled'}
    node.destroy_node()
    return {'ok': res.status == 4, 'status': res.status, 'result': res.result}


def nudge(joint, delta):
    """Move one arm joint by delta rad over 2 s and report the measured change."""
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    before = joints()
    if not all(j in before for j in ARM_JOINTS):
        return {'ok': False, 'error': 'no /joint_states for the arm'}
    target = [before[j] + (delta if j == joint + '_joint' else 0.0) for j in ARM_JOINTS]
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ARM_JOINTS
    goal.trajectory.points = [
        JointTrajectoryPoint(positions=target, time_from_start=Duration(sec=2))]
    r = _send_goal(FollowJointTrajectory, '/controller_manager/follow_joint_trajectory', goal)
    time.sleep(0.5)
    after = joints()
    j = joint + '_joint'
    moved = after.get(j, float('nan')) - before[j]
    rest = [o for o in ARM_JOINTS if o != j]
    others = (float('nan') if any(o not in after for o in rest)
              else max(abs(after[o] - before[o]) for o in rest))
    return {'ok': r.get('ok', False), 'error': r.get('error', ''), 'commanded': delta,
            'moved': moved, 'others_moved': others}


def gripper(position):
    """Command the gripper and report the resulting joint position."""
    from control_msgs.action import GripperCommand
    goal = GripperCommand.Goal()
    goal.command.position = position
    goal.command.max_effort = 5.0
    r = _send_goal(GripperCommand, '/controller_manager/gripper_cmd', goal, timeout=15.0)
    time.sleep(0.5)
    pos = joints().get('gripper_left_joint', float('nan'))
    return {'ok': r.get('ok', False), 'error': r.get('error', ''), 'position': pos}


def drive(lin, ang, seconds):
    """Publish /cmd_vel for a while, then stop; report odometry change and watchdog stop."""
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    node = rclpy.create_node('robot_check_drive')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    odom = {}

    def on_odom(m):
        q = m.pose.pose.orientation
        odom.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                    yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                    vx=m.twist.twist.linear.x, wz=m.twist.twist.angular.z)
    node.create_subscription(Odometry, '/odom', on_odom, 10)
    end = time.monotonic() + 3.0
    while time.monotonic() < end and not odom:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not odom:
        node.destroy_node()
        return {'ok': False, 'error': 'no /odom'}
    start = dict(odom)
    msg = Twist()
    msg.linear.x, msg.angular.z = lin, ang
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
    moved = dict(odom)
    end = time.monotonic() + 2.0          # stop publishing: the 1 s watchdog must stop the base
    stopped_by_watchdog = None
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    stopped_by_watchdog = abs(odom['vx']) < 1e-3 and abs(odom['wz']) < 1e-3
    pub.publish(Twist())                  # explicit stop, whatever the watchdog did
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    dist = math.hypot(moved['x'] - start['x'], moved['y'] - start['y'])
    dyaw = math.atan2(math.sin(moved['yaw'] - start['yaw']), math.cos(moved['yaw'] - start['yaw']))
    return {'ok': True, 'distance': dist, 'yaw_change': dyaw,
            'stopped': stopped_by_watchdog}


def pubtest(lin, ang, seconds):
    """
    Publish /cmd_vel with progress markers on stdout, then let the watchdog act.

    robot_check.py samples the motor PWM channels and battery current at each
    marker (START, END, WATCHDOG, ZERO); this function only drives and reports
    how far the open-loop odometry moved.
    """
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    node = rclpy.create_node('robot_check_pubtest')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    odom = {}

    def on_odom(m):
        q = m.pose.pose.orientation
        odom.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                    yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    node.create_subscription(Odometry, '/odom', on_odom, 10)
    end = time.monotonic() + 5.0
    while time.monotonic() < end and (not odom or node.count_subscribers('/cmd_vel') == 0):
        rclpy.spin_once(node, timeout_sec=0.05)
    if not odom:
        node.destroy_node()
        return {'ok': False, 'error': 'no /odom from robot_controller'}
    start = dict(odom)
    msg = Twist()
    msg.linear.x, msg.angular.z = lin, ang
    print('MARK START', flush=True)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
    moved = dict(odom)
    print('MARK END', flush=True)
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    print('MARK WATCHDOG', flush=True)
    pub.publish(Twist())
    rclpy.spin_once(node, timeout_sec=0.3)
    print('MARK ZERO', flush=True)
    node.destroy_node()
    dyaw = math.atan2(math.sin(moved['yaw'] - start['yaw']), math.cos(moved['yaw'] - start['yaw']))
    return {'ok': True, 'distance': math.hypot(moved['x'] - start['x'], moved['y'] - start['y']),
            'yaw_change': dyaw}


def main():
    """Dispatch one sub-command and print its JSON result."""
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'measure'
    rclpy.init()
    try:
        if cmd == 'measure':
            out = measure(float(sys.argv[2]) if len(sys.argv) > 2 else 6.0)
        elif cmd == 'joints':
            out = joints()
        elif cmd == 'nudge':
            out = nudge(sys.argv[2], float(sys.argv[3]))
        elif cmd == 'gripper':
            out = gripper(float(sys.argv[2]))
        elif cmd == 'pubtest':
            out = pubtest(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
        elif cmd == 'drive':
            out = drive(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
        else:
            out = {'ok': False, 'error': f'unknown command {cmd}'}
    finally:
        rclpy.shutdown()
    print('RESULT ' + json.dumps(out, default=str), flush=True)


if __name__ == '__main__':
    main()
