#!/usr/bin/env python3
"""
JeTank robot state check: one run tells you what works on the robot today.

Runs the automatable checks of plans/sim2real-test-plan.md, part by part, and
prints a pass/fail table plus a JSON report. Check ids match the plan's step
numbers (0.5 = battery, 2.5 = servos, 9.2 = IMU rate, ...).

Layers:
  hardware   devices, battery (INA219), I2C devices, servo bus, lidar USB.
             Plain I2C/serial reads; no ROS needed; nothing moves.
  software   build present, system-ROS camera build up to date, code-state
             checks for known blockers, detection prerequisites.
             --tests also runs the full colcon test suite (slow).
  ros        starts the robot stack itself unless one is already running:
             camera under system ROS, the rest in pixi, servos MOCKED, all on
             a private ROS domain so nothing on the network can send it
             /cmd_vel; the web UI's drive output goes to a dead topic. The
             drive-motor node does run (it owns /odom), but nothing commands
             it. Measures rates/IMU/TF/controllers/web/camera, runs one mock
             grasp, then stops only the processes it started.
  motion     only with --motion and a terminal: asks first, then torques the
             real servos, moves each arm joint 0.1 rad and back, closes and
             opens the gripper, spins the tracks (wheels off the ground).
             Asks for a typed "yes" before each group.

Run with the SYSTEM python (it needs smbus2 + pyserial, which the pixi env
lacks):

    python3 src/jetank_ros_main/tools/robot_check.py            # read-only
    python3 src/jetank_ros_main/tools/robot_check.py --quick    # no mock grasp
    python3 src/jetank_ros_main/tools/robot_check.py --motion   # + motion checks
    pixi run check                                              # same as the first line

Exit code: 0 when nothing FAILs, 1 otherwise.
"""

import argparse
import datetime
import grp
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

WS = Path(__file__).resolve().parents[3]
SRC = WS / 'src'
HERE = Path(__file__).resolve().parent
PIXI = shutil.which('pixi') or str(Path.home() / '.pixi/bin/pixi')
SYS_SETUP = '/opt/ros/humble/setup.bash'
ENV = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID', '42'),
           RMW_IMPLEMENTATION=os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'))
ANSI = re.compile(r'\x1b\[[0-9;]*m')     # terminal colour codes in tool output
PRIVATE_DOMAIN = '87'          # stacks this script starts run here, isolated from the network
SERVO_PORT, SERVO_BAUD, SERVO_IDS = '/dev/ttyTHS1', 1_000_000, [1, 2, 3, 4, 5]
SERVO_NAMES = {1: 'S1 base', 2: 'S2 shoulder', 3: 'S3 elbow', 4: 'gripper', 5: 'camera tilt'}
STACK_PROGRAMS = {'robot_controller', 'ros2_control_node', 'move_group', 'web_control_node',
                  'icm20948_node', 'rplidar_node', 'stereo_camera_node', 'gzserver',
                  'ign', 'gz', 'robot_state_publisher', 'grasp_server'}

# DC drive motors on the PCA9685 (bus 7, 0x60), as motor.cpp drives them.
# Each side: one PWM channel plus forward/reverse direction channels. motor.cpp
# also writes channels 0-3, but the measured current shows nothing is wired to
# them (they keep their last duty after a stop with the motors drawing idle current).
MOTOR = {'left': {'pwm': 8, 'fwd': 9, 'rev': 10}, 'right': {'pwm': 13, 'fwd': 12, 'rev': 11}}
TRACK_WIDTH, MAX_LIN, MAX_ANG = 0.11, 1.0, 2.0   # jetank_motor_control/config/motor_params.yaml

RESULTS = []
STARTED = []          # Popen objects this run started; only these are ever stopped


# ----------------------------------------------------------------------------- plumbing
def record(cid, part, name, status, value='', expect=''):
    """Store one check result and print it immediately."""
    RESULTS.append({'id': cid, 'part': part, 'name': name, 'status': status,
                    'value': str(value), 'expect': str(expect)})
    colors = {'PASS': '32', 'FAIL': '31', 'WARN': '33', 'SKIP': '90', 'INFO': '36'}
    tag = f'\033[{colors[status]}m{status:4}\033[0m' if sys.stdout.isatty() else f'{status:4}'
    line = f'  {tag}  {cid:6} {name}'
    if value:
        line += f' — {value}'
    if expect and status in ('FAIL', 'WARN'):
        line += f'  (expected {expect})'
    print(line, flush=True)


def section(title):
    """Print a part heading."""
    print(f'\n== {title}', flush=True)


def guarded(cid, part, name):
    """Turn an unexpected exception inside one check group into a FAIL row."""
    def wrap(fn):
        def inner(*a, **kw):
            try:
                return fn(*a, **kw)
            except KeyboardInterrupt:
                raise
            except Exception as e:  # noqa: B902 - any crash becomes a recorded FAIL
                record(cid, part, name, 'FAIL', f'check crashed: {type(e).__name__}: {e}')
        return inner
    return wrap


def run(cmd, timeout=60, shell=False):
    """Run a command in its own process group; on timeout kill the whole group."""
    try:
        p = subprocess.Popen(cmd, shell=shell, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, env=ENV, cwd=WS, start_new_session=True,
                             executable='/bin/bash' if shell else None)
    except OSError as e:
        return 127, str(e)
    try:
        out, _ = p.communicate(timeout=timeout)
        return p.returncode, ANSI.sub('', out)
    except subprocess.TimeoutExpired:
        for sig in (signal.SIGINT, signal.SIGKILL):
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                break
            try:
                out, _ = p.communicate(timeout=3)
                return 124, out or ''
            except subprocess.TimeoutExpired:
                continue
        return 124, ''


def pixi_py(args, timeout=60):
    """Run tools/robot_check_ros.py inside the pixi env and return its JSON result."""
    rc, out = run([PIXI, 'run', '-q', 'python', str(HERE / 'robot_check_ros.py')] + args,
                  timeout=timeout)
    m = re.search(r'^RESULT (.*)$', out, re.M)
    if not m:
        tail = (out.strip().splitlines() or ['no output'])[-1][:200]
        return {'ok': False, 'error': 'timed out' if rc == 124 else tail}
    try:
        return json.loads(m.group(1))
    except ValueError:
        return {'ok': False, 'error': 'unreadable result'}


def running_programs():
    """
    Return the executable names of processes running on this machine.

    Read from /proc rather than ``ros2 node list``: the ROS 2 daemon keeps
    stale node names for a while after a stack stops, which made an empty
    system look like a running one.
    """
    names = set()
    for cmdline in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = cmdline.read_bytes().split(b'\0')
        except OSError:
            continue
        for arg in argv[:2]:
            if arg:
                names.add(os.path.basename(arg.decode(errors='replace')))
    return names


def read_text(path):
    """Return a file's text, or None when it does not exist."""
    p = Path(path)
    return p.read_text(errors='replace') if p.is_file() else None


# ----------------------------------------------------------------------------- hardware
def check_hardware(args):
    """Devices, battery, I2C devices, servos and lidar, without ROS."""
    section('Power and devices (no ROS, nothing moves)')
    names = set()
    for g in os.getgroups():
        try:
            names.add(grp.getgrgid(g).gr_name)
        except KeyError:
            continue
    missing = {'i2c', 'gpio', 'dialout'} - names
    record('0.3', 'pre-flight', 'user groups', 'FAIL' if missing else 'PASS',
           'missing ' + ','.join(sorted(missing)) if missing else 'i2c, gpio, dialout')
    for dev in ['/dev/i2c-1', '/dev/i2c-7', SERVO_PORT, '/dev/video0', '/dev/video1']:
        ok = os.path.exists(dev) and os.access(dev, os.R_OK | os.W_OK)
        record('0.4', 'pre-flight', f'device {dev}', 'PASS' if ok else 'FAIL',
               'present' if ok else 'missing or no access')
    rc, out = run(['nvpmodel', '-q'], timeout=10)
    mode = re.search(r'NV Power Mode:\s*(\S+)', out)
    record('0.6', 'pre-flight', 'Jetson power mode', 'INFO', mode.group(1) if mode else 'unknown')

    check_i2c()
    check_motor_idle()
    check_servos()

    by_id = Path('/dev/serial/by-id')
    lidar = list(by_id.glob('*CP2102N*')) if by_id.exists() else []
    record('10.1', 'lidar', 'RPLidar USB (CP2102N)', 'PASS' if lidar else 'FAIL',
           os.path.realpath(lidar[0]) if lidar else 'not enumerated — replug USB')


def check_i2c():
    """Battery voltage, PCA9685, OLED and IMU identity over I2C."""
    try:
        from smbus2 import SMBus
    except ImportError:
        record('0.5', 'power', 'I2C checks', 'SKIP', 'smbus2 not installed for this python')
        return
    try:
        with SMBus(7) as bus:
            try:
                raw = bus.read_i2c_block_data(0x41, 0x02, 2)
                volts = ((raw[0] << 8 | raw[1]) >> 3) * 0.004
                st = 'PASS' if volts >= 11.1 else 'WARN' if volts >= 10.5 else 'FAIL'
                record('0.5', 'power', 'battery pack voltage (INA219 0x41)', st,
                       f'{volts:.2f} V', '11.1–12.6 V')
            except OSError as e:
                record('0.5', 'power', 'battery monitor INA219 0x41', 'FAIL', f'no answer: {e}')
            for addr, what in [(0x60, 'PCA9685 motor PWM'), (0x3C, 'OLED')]:
                try:
                    bus.read_byte(addr)
                    record('2.1', 'buses', f'I2C bus 7 {hex(addr)} {what}', 'PASS', 'answers')
                except OSError:
                    record('2.1', 'buses', f'I2C bus 7 {hex(addr)} {what}',
                           'FAIL' if addr == 0x60 else 'WARN', 'no answer')
            try:
                record('2.6', 'buses', 'PCA9685 MODE1 register', 'PASS',
                       hex(bus.read_byte_data(0x60, 0x00)))
            except OSError as e:
                record('2.6', 'buses', 'PCA9685 MODE1 register', 'FAIL', str(e))
    except OSError as e:
        record('2.1', 'buses', 'I2C bus 7', 'FAIL', f'cannot open: {e}')
    if 'icm20948_node' in running_programs():
        record('9.1', 'imu', 'ICM-20948 WHO_AM_I', 'SKIP',
               'IMU driver is running (it switches register banks); see the IMU rate check')
        return
    try:
        with SMBus(1) as bus:
            who = bus.read_byte_data(0x68, 0x00)
            record('9.1', 'imu', 'ICM-20948 WHO_AM_I (bus 1 0x68)',
                   'PASS' if who == 0xEA else 'FAIL', hex(who), '0xea')
    except OSError as e:
        record('9.1', 'imu', 'ICM-20948 on bus 1', 'FAIL', f'no answer: {e}')


def pca_channel(bus, ch):
    """Return a PCA9685 channel's duty as 0.0-1.0 (full-off and full-on bits honoured)."""
    on_l, on_h, off_l, off_h = bus.read_i2c_block_data(0x60, 6 + 4 * ch, 4)
    if off_h & 0x10:
        return 0.0
    if on_h & 0x10:
        return 1.0
    return ((off_h & 0x0F) << 8 | off_l) / 4096


def motor_snapshot():
    """Read both motors' PWM/direction channels, channels 0-3 and the battery current proxy."""
    from smbus2 import SMBus
    with SMBus(7) as bus:
        raw = bus.read_i2c_block_data(0x41, 0x01, 2)
        shunt = (raw[0] << 8) | raw[1]
        shunt = shunt - 65536 if shunt & 0x8000 else shunt
        snap = {'shunt_mV': shunt * 0.01}
        for side, ch in MOTOR.items():
            for role, c in ch.items():
                snap[f'{side}_{role}'] = pca_channel(bus, c)
        snap['ch0_3'] = [pca_channel(bus, c) for c in range(4)]
    return snap


def motor_direction(snap, side):
    """Return 'fwd', 'rev', 'off' or 'both' for one motor from its direction channels."""
    fwd, rev = snap[f'{side}_fwd'] > 0.5, snap[f'{side}_rev'] > 0.5
    return 'both' if fwd and rev else 'fwd' if fwd else 'rev' if rev else 'off'


def check_motor_idle():
    """Read-only: with no drive node running, both motor outputs must be off."""
    try:
        snap = motor_snapshot()
    except (ImportError, OSError) as e:
        record('5.0', 'base', 'DC motor outputs', 'SKIP', f'cannot read PCA9685/INA219: {e}')
        return
    record('5.0', 'base', 'battery current at rest (INA219 shunt)', 'INFO',
           f"{snap['shunt_mV']:.2f} mV")
    if 'robot_controller' in running_programs():
        record('5.0', 'base', 'DC motor outputs idle', 'SKIP', 'drive node running')
        return
    live = [f'{s} {motor_direction(snap, s)} {snap[s + "_pwm"]:.0%}' for s in MOTOR
            if snap[f'{s}_pwm'] > 0 or motor_direction(snap, s) != 'off']
    record('5.0', 'base', 'DC motor outputs idle (PCA9685 ch 8-13)',
           'FAIL' if live else 'PASS', ', '.join(live) if live else 'both off',
           'off when no drive node runs')


def port_holders(dev):
    """Return the PIDs (other than ours) that have the device open."""
    pids = []
    for fd_dir in Path('/proc').glob('[0-9]*/fd'):
        try:
            if int(fd_dir.parent.name) == os.getpid():
                continue
            for fd in fd_dir.iterdir():
                if os.readlink(fd) == dev:
                    pids.append(int(fd_dir.parent.name))
                    break
        except (PermissionError, FileNotFoundError, ProcessLookupError, OSError):
            continue
    return pids


def check_servos():
    """Ping every servo and read position/voltage/temperature (read-only SCS protocol)."""
    holders = port_holders(SERVO_PORT)
    if holders or 'ros2_control_node' in running_programs():
        record('2.5', 'servos', 'servo bus', 'SKIP',
               f'{SERVO_PORT} in use by ros2_control (pid {holders or "?"}) — see the arm checks')
        return
    try:
        import serial
    except ImportError:
        record('2.5', 'servos', 'servo bus', 'SKIP', 'pyserial not installed for this python')
        return
    try:
        port = serial.Serial(SERVO_PORT, SERVO_BAUD, timeout=0.02, exclusive=True)
    except (serial.SerialException, OSError, ValueError) as e:
        record('2.5', 'servos', 'servo bus open', 'FAIL', str(e))
        return

    def tx(sid, ins, params=b''):
        """Send one instruction; return the validated reply params and error byte, or None."""
        body = bytes([sid, len(params) + 2, ins]) + params
        port.reset_input_buffer()
        port.write(b'\xff\xff' + body + bytes([(~sum(body)) & 0xFF]))
        port.flush()
        time.sleep(0.012)
        rx = port.read(64)
        sent = b'\xff\xff' + body + bytes([(~sum(body)) & 0xFF])
        skip = len(sent) if rx.startswith(sent) else 0   # half-duplex echo, when present
        start = rx.find(b'\xff\xff' + bytes([sid]), skip)
        if start < 0 or len(rx) < start + 4:
            return None
        length = rx[start + 3]
        pkt = rx[start + 2:start + 4 + length]          # id, len, err, params..., checksum
        if len(pkt) != length + 2 or (~sum(pkt[:-1])) & 0xFF != pkt[-1]:
            return None
        return pkt[2], pkt[3:-1]

    try:
        for sid in SERVO_IDS:
            label = f'servo {sid} ({SERVO_NAMES[sid]})'
            ok = sum(1 for _ in range(5) if tx(sid, 0x01) is not None)
            if not ok:
                record('2.5', 'servos', label, 'FAIL', '0/5 ping replies', '5/5, ≥ 6.0 V, err 0')
                continue
            reply = tx(sid, 0x02, bytes([0x38, 8]))   # pos(2) speed(2) load(2) volt(1) temp(1)
            if reply is None or len(reply[1]) != 8:
                record('2.5', 'servos', label, 'WARN', f'ping {ok}/5, status read failed')
                continue
            err, d = reply
            pos, volt, temp = d[0] << 8 | d[1], d[6] / 10, d[7]
            st = 'FAIL' if volt < 6.0 or err else 'WARN' if ok < 5 or temp > 60 else 'PASS'
            record('2.5', 'servos', label, st,
                   f'ping {ok}/5, pos {pos}, {volt:.1f} V, {temp} °C, err 0x{err:02x}',
                   '5/5, ≥ 6.0 V, err 0')
    finally:
        port.close()


# ----------------------------------------------------------------------------- software
def check_software(args):
    """Build present, camera build fresh, known blockers, detection prerequisites."""
    section('Software and code state')
    pkgs = sorted(p.name for p in SRC.iterdir() if (p / 'package.xml').exists())
    missing = [p for p in pkgs if not (WS / 'install' / p).exists()]
    record('1.1', 'build', f'{len(pkgs)} packages built in install/',
           'FAIL' if missing else 'PASS',
           'missing: ' + ', '.join(missing) if missing else 'all present')
    cam_bin = WS / 'install_sys/jetank_perception/lib/jetank_perception/stereo_camera_node'
    newest_src = max((f.stat().st_mtime for f in (SRC / 'jetank_perception').rglob('*')
                      if f.suffix in ('.cpp', '.hpp') and f.is_file()), default=0)
    if not cam_bin.exists():
        record('6.0', 'camera', 'system-ROS camera build (install_sys)', 'FAIL', 'not built')
    else:
        stale = cam_bin.stat().st_mtime < newest_src
        record('6.0', 'camera', 'system-ROS camera build (install_sys)',
               'WARN' if stale else 'PASS',
               'older than source — rebuild with colcon under /opt/ros/humble'
               if stale else 'up to date')
    if args.tests:
        run_tests()
    else:
        record('1.2', 'build', 'colcon test', 'SKIP', 'run with --tests')

    # Known blockers from the plan, checked in the code so a fix shows up here.
    bridge = read_text(SRC / 'jetank_web_control/jetank_web_control/cmd_vel_bridge.py')
    if bridge is None:
        record('B2', 'blockers', 'cmd_vel_bridge empty-topic guard', 'SKIP', 'file not found')
    else:
        ok = re.search(r'if\s+teleop_topic\s*:', bridge) is not None
        record('B2', 'blockers', 'cmd_vel_bridge guards an empty teleop topic',
               'PASS' if ok else 'FAIL', 'guarded' if ok
               else 'subscribes unconditionally; mobile_grasp_hw passes teleop_topic ""')
    hw = read_text(SRC / 'jetank_ros_main/launch/mobile_grasp_hw.launch.py')
    if hw is None:
        record('B4', 'blockers', 'mobile_grasp_hw camera', 'SKIP', 'file not found')
    else:
        ok = re.search(r'"enable_perception"\s*:\s*"false"', hw) is not None
        record('B4', 'blockers', 'mobile_grasp_hw keeps the pixi camera off',
               'PASS' if ok else 'FAIL', 'enable_perception false' if ok
               else 'unified starts a second (pixi) camera node')
    cfg = read_text(SRC / 'jetank_perception/config/stereo_camera_config.yaml')
    m = re.search(r'publishing\.raw_images\.compression\.mode:\s*"(\w+)"', cfg or '')
    raw_mode = m.group(1) if m else 'unknown'
    record('B3', 'blockers', 'camera publishes raw image_raw (detector input)',
           'WARN' if raw_mode != 'both' else 'PASS', f'raw mode {raw_mode}',
           'both, for detection runs')

    model = Path.home() / 'models/sock_real.pt'
    record('14.2', 'detection', 'sock model ~/models/sock_real.pt',
           'PASS' if model.exists() else 'FAIL', 'present' if model.exists() else 'missing')
    env = WS / '.pixi/envs/detect'
    record('14.1', 'detection', 'pixi detect environment', 'PASS' if env.exists() else 'WARN',
           'installed' if env.exists() else 'not installed — pixi install -e detect')


def run_tests():
    """Run the colcon test suite and judge it, ignoring the offline xmllint checks."""
    print('  …running pixi run test (several minutes)', flush=True)
    run([PIXI, 'run', 'test'], timeout=1800)
    rc, res = run([PIXI, 'run', '-q', 'colcon', 'test-result', '--verbose'], timeout=120)
    m = re.search(r'Summary: (\d+) tests, (\d+) errors, (\d+) failures', res)
    non_xml = [ln for ln in res.splitlines()
               if re.search(r'tests, \d+ errors, [1-9]', ln)
               and 'xmllint' not in ln and 'Test.xml' not in ln]
    if not m:
        record('1.2', 'build', 'colcon test', 'FAIL', 'no summary (build failed?)')
        return
    detail = (' (non-xmllint: ' + '; '.join(non_xml)[:150] + ')' if non_xml
              else ' (offline xmllint only)')
    record('1.2', 'build', 'colcon test', 'FAIL' if int(m.group(2)) or non_xml else 'PASS',
           f'{m.group(1)} tests, {m.group(2)} errors, {m.group(3)} failures' + detail)


# ----------------------------------------------------------------------------- ROS stack
def start(cmd, log, shell=False):
    """Start a long-running process in its own process group; remember it for teardown."""
    fh = open(log, 'w')
    p = subprocess.Popen(cmd, shell=shell, stdout=fh, stderr=subprocess.STDOUT, env=ENV, cwd=WS,
                         start_new_session=True, executable='/bin/bash' if shell else None)
    p.log_handle = fh
    STARTED.append(p)
    return p


def wait_for(log, patterns, fail_patterns, timeout):
    """Wait until every pattern is in the log (True) or a failure pattern appears (False)."""
    end = time.monotonic() + timeout
    text = ''
    while time.monotonic() < end:
        text = read_text(log) or ''
        if any(f in text for f in fail_patterns):
            return False, text
        if all(text.count(p) >= n for p, n in patterns):
            return True, text
        time.sleep(1)
    return False, text


def teardown():
    """
    Stop only what this run started: SIGINT each group, then TERM, then KILL.

    Signals that would interrupt teardown itself are ignored while it runs, and
    every group is signalled even if its leader already exited (its children may not have).
    """
    sigs = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    old = {s: signal.signal(s, signal.SIG_IGN) for s in sigs}
    try:
        for p in reversed(STARTED):
            for sig, wait in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
                try:
                    os.killpg(p.pid, sig)
                except ProcessLookupError:
                    break
                end = time.monotonic() + wait
                while time.monotonic() < end:
                    try:
                        os.killpg(p.pid, 0)
                    except ProcessLookupError:
                        break
                    time.sleep(0.3)
                else:
                    continue
                break
            try:
                p.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass
            p.log_handle.close()
        STARTED.clear()
    finally:
        for s, h in old.items():
            signal.signal(s, h)


def check_ros(args, logdir):
    """Start (or attach to) the robot stack and measure every runtime check."""
    section('Robot software running')
    running = running_programs()
    present = running & STACK_PROGRAMS
    attach = bool(running & {'robot_controller', 'ros2_control_node'})
    hardware = 'serial' if args.motion else 'mock'
    if present and not attach:
        record('R.0', 'stack', 'robot stack', 'FAIL',
               f'part of a stack is already running ({", ".join(sorted(present))}); '
               'stop it, or start it completely, then re-run')
        return
    if attach:
        record('R.0', 'stack', 'robot stack', 'INFO',
               f'already running on domain {ENV["ROS_DOMAIN_ID"]}'
               ' — measuring it, not starting one')
    else:
        ENV['ROS_DOMAIN_ID'] = PRIVATE_DOMAIN
        record('R.0', 'stack', 'robot stack', 'INFO',
               f'starting it on private ROS domain {PRIVATE_DOMAIN}, servos {hardware}; '
               'drive motor node live but nothing commands it')
        if Path(WS / 'install_sys/setup.bash').exists():
            start(f'source {SYS_SETUP} && source install_sys/setup.bash && exec ros2 launch '
                  'jetank_perception stereo_camera.launch.py namespace:=stereo_camera '
                  'publish_camera_transforms:=false', logdir / 'camera.log', shell=True)
        start([PIXI, 'run', 'ros2', 'launch', 'jetank_ros_main', 'unified.launch.py',
               f'hardware:={hardware}', 'enable_moveit:=true', 'enable_web_control:=false',
               'enable_navigation:=false', 'enable_perception:=false'], logdir / 'unified.log')
        start([PIXI, 'run', 'ros2', 'launch', 'jetank_web_control', 'web_control.launch.py',
               'cmd_vel_topic:=/robot_check/web_cmd_vel_disabled'], logdir / 'web.log')
        ok, text = wait_for(logdir / 'unified.log', [('Configured and activated', 3)],
                            ['did not respond to ping', 'Failed to set the initial state'], 150)
        why = (re.findall(r'(Servo id \d+ .*?ping)', text) or ['did not come up in 150 s'])[0]
        record('3a.1' if hardware == 'mock' else '3b.1', 'arm',
               f'ros2_control on {hardware} servos', 'PASS' if ok else 'FAIL',
               '3 controllers activated' if ok else why)
        if not ok:
            return
        died = re.findall(r'\[ERROR\] \[([\w_]+)-\d+\]: process has died', text)
        if died:
            record('R.1', 'stack', 'processes that died at start', 'FAIL',
                   ', '.join(sorted(set(died))))
        time.sleep(4)   # IMU/lidar/web settle

    measure_topics()
    rc, out = run([PIXI, 'run', '-q', 'ros2', 'control', 'list_controllers'], timeout=60)
    active = len(re.findall(r'\bactive\b', out))
    record('3a.2', 'arm', 'controllers active', 'PASS' if active >= 3 else 'FAIL',
           f'{active} active', '3')
    check_camera(logdir)
    check_web()
    if args.quick:
        record('3a.4', 'arm', 'preset grasp through MoveIt', 'SKIP', '--quick')
    elif hardware == 'mock' and not attach:
        mock_grasp(logdir)
    else:
        record('3a.4', 'arm', 'preset grasp through MoveIt', 'SKIP',
               'runs only on a stack this script started with mock servos')


def measure_topics():
    """Rates, IMU at rest, and the odom TF, from one measuring node."""
    m = pixi_py(['measure', '8'], timeout=120)
    if 'rates' not in m:
        record('R.2', 'stack', 'topic measurement', 'FAIL', m.get('error', 'no result'))
        return
    rt = m['rates']

    def rate(cid, part, name, key, nominal):
        v = rt.get(key, 0.0)
        st = 'PASS' if v >= 0.9 * nominal else 'WARN' if v >= 0.5 * nominal else 'FAIL'
        record(cid, part, name, st, f'{v:.1f} Hz', f'≈ {nominal} Hz')

    rate('5a.1', 'base', '/odom rate', 'odom', 30)
    record('5a.4', 'base', 'TF odom → base_footprint', 'PASS' if m.get('tf_odom_base') else 'FAIL',
           'resolves' if m.get('tf_odom_base') else 'missing')
    rate('3a.3', 'arm', '/joint_states rate', 'joint_states', 50)
    rate('9.2', 'imu', '/imu/data_raw rate', 'imu', 100)
    imu = m.get('imu')
    if imu:
        still = 9.3 <= imu['accel_norm'] <= 10.3 and imu['gyro_max'] < 0.1
        record('9.3', 'imu', 'IMU at rest', 'PASS' if still else 'WARN',
               f"|a| {imu['accel_norm']:.2f} m/s², gyro max {imu['gyro_max']:.3f} rad/s",
               '|a| 9.3–10.3, gyro < 0.1 (robot still)')
        ax = ('+' if imu['gravity_sign'] > 0 else '−') + imu['gravity_axis']
        record('9.4', 'imu', 'gravity axis in imu_link', 'PASS' if ax == '+z' else 'WARN',
               f'gravity on {ax}', '+z, or a URDF imu joint rotation matching the mount')
    rate('10.3', 'lidar', '/scan rate', 'scan', 10)


def check_camera(logdir):
    """Camera start-up and disparity/point-cloud rates via the system-ROS probe."""
    if not Path(WS / 'install_sys/setup.bash').exists():
        record('6.1', 'camera', 'stereo camera', 'SKIP', 'install_sys not built')
        return
    cam_log = logdir / 'camera.log'
    if cam_log.exists():
        ok, text = wait_for(cam_log, [('Started processing thread', 1)],
                            ['Failed to start'], 30)
        record('6.1', 'camera', 'CSI sensors open', 'PASS' if ok else 'FAIL',
               'started' if ok else 'failed to start (see camera.log)')
    probe = SRC / 'jetank_perception/scripts/topic_rate_probe.py'
    for kind in ('disparity', 'points'):
        rc, out = run(f'source {SYS_SETUP} && python3 {probe} {kind} reliable 8',
                      shell=True, timeout=60)
        mm = re.search(r'hz=([\d.]+) lat_p50_ms=(\d+)', out)
        hz, lat = (float(mm.group(1)), int(mm.group(2))) if mm else (0.0, -1)
        record('6.4', 'camera', f'{kind} rate (probe)', 'PASS' if hz >= 20 else 'FAIL',
               f'{hz:.1f} Hz, p50 {lat} ms' if mm else 'no messages', '≥ 20 Hz')


def check_web():
    """Check that the web UI answers with its own page."""
    try:
        with urllib.request.urlopen('http://localhost:8080/', timeout=5) as r:
            body = r.read(4096).decode(errors='replace')
            title = re.search(r'<title>([^<]*)', body)
            ours = bool(title and 'JeTank' in title.group(1))
            record('13.1', 'web', 'web UI on :8080',
                   'PASS' if r.status == 200 and ours else 'FAIL',
                   f'HTTP {r.status}, title "{title.group(1) if title else ""}"',
                   'HTTP 200, a JeTank page')
    except Exception as e:  # noqa: B902 - any failure to fetch is a FAIL row
        record('13.1', 'web', 'web UI on :8080', 'FAIL', f'{type(e).__name__}: {e}')


def mock_grasp(logdir):
    """One preset grasp through MoveIt and grasp_server on the mock servos."""
    gp = (WS / 'install/jetank_manipulation/share/jetank_manipulation'
          / 'config/grasp_poses.yaml')
    start([PIXI, 'run', 'ros2', 'run', 'jetank_manipulation', 'grasp_server',
           '--ros-args', '-r', '__node:=grasp_server', '--params-file', str(gp),
           '-p', 'use_sim_time:=false',
           '-p', 'gripper_action:=/controller_manager/gripper_cmd'],
          logdir / 'grasp_server.log')
    wait_for(logdir / 'grasp_server.log', [('grasp_object', 1)], ['Traceback'], 30)
    t0 = time.monotonic()
    rc, out = run([PIXI, 'run', '-q', 'ros2', 'action', 'send_goal', '/grasp_object',
                   'jetank_manipulation/action/GraspObject', "{object_hint: ''}"], timeout=150)
    dt = time.monotonic() - t0
    ok = 'SUCCEEDED' in out
    waits = (read_text(logdir / 'grasp_server.log') or '').count('not available')
    record('3a.4', 'arm', 'preset grasp through MoveIt (mock servos)',
           'PASS' if ok and not waits else 'FAIL',
           f'{"SUCCEEDED" if ok else "failed"} in {dt:.1f} s, {waits} gripper waits')


# ----------------------------------------------------------------------------- motion
def confirm(question):
    """Ask for a typed yes; refuse when there is no terminal."""
    if not sys.stdin.isatty():
        return False
    try:
        return input(f'\n  >>> {question} Type yes to continue: ').strip().lower() == 'yes'
    except EOFError:
        return False


def check_motion(args):
    """Tiny arm moves, gripper, and a wheels-off-ground drive on the real hardware."""
    section('Motion (real servos and tracks)')
    j = pixi_py(['joints'], timeout=60)
    if 'S1_joint' not in j:
        record('3b', 'arm', 'arm motion', 'SKIP', 'no /joint_states for the arm')
    elif not port_holders(SERVO_PORT):
        record('3b', 'arm', 'arm motion', 'SKIP',
               'the running stack does not hold the servo port (mock servos)'
               ' — nothing real to move')
    elif confirm('Arm will move each joint +0.1 rad and back (about 6 degrees).'
                 ' Arm clear, hand on the power cut?'):
        for joint in ('S1', 'S2', 'S3'):
            for delta in (0.1, -0.1):
                r = pixi_py(['nudge', joint, str(delta)], timeout=90)
                good = (r.get('ok') and abs(r.get('moved', 9) - delta) <= 0.05
                        and r.get('others_moved', 9) <= 0.02)
                detail = r.get('error') or (f"moved {r.get('moved', 0):+.3f} rad, "
                                            f"others {r.get('others_moved', 0):.3f}")
                record('3b.3', 'arm', f'{joint} {delta:+.1f} rad', 'PASS' if good else 'FAIL',
                       detail, f'{delta:+.2f} ± 0.05 rad')
        record('3b.3', 'arm', 'joint directions', 'INFO',
               'check by eye: +S1 = counter-clockwise from above')
        if confirm('Gripper will close, then open. Nothing between the jaws?'):
            for target in (0.0, 0.04):
                r = pixi_py(['gripper', str(target)], timeout=60)
                pos = r.get('position', float('nan'))
                good = r.get('ok') and abs(pos - target) <= 0.005
                record('4.2', 'arm', f'gripper to {target:.2f} m', 'PASS' if good else 'FAIL',
                       r.get('error') or f'at {pos:.3f} m', f'{target:.3f} ± 0.005 m')
        else:
            record('4.2', 'arm', 'gripper open/close', 'SKIP', 'not confirmed')
    else:
        record('3b.3', 'arm', 'tiny arm moves', 'SKIP', 'not confirmed')
    if confirm('Tracks will spin about 1.5 s each: forward, backward, turn left, turn right.'
               ' Tracks OFF the ground, hand on the power cut?'):
        motor_test()
    else:
        record('5a.2', 'base', 'DC motor test', 'SKIP', 'not confirmed')


def expected_duty(lin, ang):
    """Per-motor (signed) command as robot_controller computes it, and the PWM duty it writes."""
    left, right = lin - ang * TRACK_WIDTH / 2, lin + ang * TRACK_WIDTH / 2
    scale = max(abs(left), abs(right)) / MAX_LIN
    if scale > 1:
        left, right = left / scale, right / scale
    duty = {s: int(255 * min(1.0, abs(v) / MAX_LIN)) * 16 / 4096
            for s, v in (('left', left), ('right', right))}
    sign = {'left': left, 'right': right}
    return duty, sign


def motor_case(name, lin, ang):
    """Drive one cmd_vel case through robot_controller and judge PWM, direction, current, stop."""
    cmd = [PIXI, 'run', '-q', 'python', str(HERE / 'robot_check_ros.py'), 'pubtest',
           str(lin), str(ang), '1.5']
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                         env=ENV, cwd=WS, start_new_session=True)
    snaps, result = {'before': motor_snapshot()}, {}
    try:
        for line in p.stdout:
            if line.startswith('MARK START'):
                time.sleep(0.8)
                snaps['during'] = motor_snapshot()
            elif line.startswith('MARK END'):
                time.sleep(0.3)
                snaps['end'] = motor_snapshot()
            elif line.startswith('MARK WATCHDOG'):
                snaps['watchdog'] = motor_snapshot()
            elif line.startswith('RESULT '):
                result = json.loads(line[7:])
        p.wait(timeout=30)
    finally:
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
    if 'during' not in snaps or 'watchdog' not in snaps:
        record('5a.2', 'base', f'motors: {name}', 'FAIL', result.get('error', 'no drive markers'))
        return
    duty, sign = expected_duty(lin, ang)
    during, after = snaps['during'], snaps['watchdog']
    problems = []
    for side in MOTOR:
        want = 'fwd' if sign[side] > 0 else 'rev' if sign[side] < 0 else 'off'
        got = motor_direction(during, side)
        if got != want:
            problems.append(f'{side} direction {got}, expected {want}')
        if abs(during[f'{side}_pwm'] - duty[side]) > 0.02:
            problems.append(f"{side} PWM {during[f'{side}_pwm']:.0%}, expected {duty[side]:.0%}")
    record('5a.2', 'base', f'motors: {name} — driver outputs', 'FAIL' if problems else 'PASS',
           '; '.join(problems) or f"left {motor_direction(during, 'left')} "
           f"{during['left_pwm']:.0%}, right {motor_direction(during, 'right')} "
           f"{during['right_pwm']:.0%}")
    idle = snaps['before']['shunt_mV']
    peak = max(during['shunt_mV'], snaps.get('end', during)['shunt_mV'])
    rise = peak - idle
    low_duty = max(duty.values()) < 0.2
    record('5a.2', 'base', f'motors: {name} — current drawn',
           'PASS' if rise >= 1.0 else 'WARN',
           f'+{rise:.1f} mV on the shunt (idle {idle:.1f} mV)'
           + (f', only {max(duty.values()):.0%} duty' if low_duty else ''),
           '≥ +1.0 mV while driving; a weak rise means the tracks may not have turned')
    stopped = all(after[f'{s}_pwm'] == 0 and motor_direction(after, s) == 'off' for s in MOTOR)
    back = after['shunt_mV'] - idle < 1.0
    record('5a.3', 'base', f'motors: {name} — watchdog stop',
           'PASS' if stopped and back else 'FAIL',
           ('outputs off' if stopped else 'outputs still on')
           + (', current back to idle' if back else f", current still {after['shunt_mV']:.1f} mV"),
           'outputs off and idle current 2 s after the last command')
    if ang == 0:
        dist = result.get('distance', 0)
        record('5a.5', 'base', f'odometry integrates {name}', 'PASS' if dist > 0.3 else 'FAIL',
               f'{dist:.2f} m (open-loop, from the command)', '> 0.3 m')
    else:
        yaw = result.get('yaw_change', 0)
        ok = yaw * ang > 0 and abs(yaw) > 1.0
        record('5a.5', 'base', f'odometry integrates {name}', 'PASS' if ok else 'FAIL',
               f'{yaw:+.2f} rad (open-loop, from the command)', '|yaw| > 1.0 rad, same sign')


def motor_test():
    """Forward, backward and both spins through robot_controller, measured at the PCA9685."""
    try:
        motor_snapshot()
    except (ImportError, OSError) as e:
        record('5a.2', 'base', 'DC motor test', 'SKIP', f'cannot read PCA9685/INA219: {e}')
        return
    for name, lin, ang in (('forward', 0.4, 0.0), ('backward', -0.4, 0.0),
                           ('turn left', 0.0, MAX_ANG), ('turn right', 0.0, -MAX_ANG)):
        motor_case(name, lin, ang)
        time.sleep(1.0)
    _, sign = expected_duty(0.0, MAX_ANG)
    spin = abs(sign['left']) / MAX_LIN
    if spin < 0.2:
        record('5a.2', 'base', 'spin-in-place strength', 'WARN',
               f'max_angular_velocity {MAX_ANG} × track {TRACK_WIDTH} m gives only {spin:.0%} '
               'duty per track', '≥ 20 % so the tracks overcome friction')
    record('5a.2', 'base', 'track directions', 'INFO',
           'check by eye: forward, backward, then left and right spins')


# ----------------------------------------------------------------------------- not automated
def not_automated():
    """List the plan phases this script cannot judge yet, so a green run is not over-read."""
    section('Not automated yet (see plans/sim2real-test-plan.md)')
    for cid, name, why in [
        ('6.6', 'sock segmentation on a real sock', 'needs a sock in view and a detector model'),
        ('7–8', 'approach + grasp pipeline, full pick', 'needs detector model; blockers B2/B4'),
        ('9.4', 'IMU mounting by tilting the robot', 'needs a person to tilt the robot'),
        ('11', 'SLAM mapping', 'needs driving round a room'),
        ('12', 'Nav2 + AMCL', 'needs a saved map'),
        ('13.2', 'web teleop, map view, capture, grab', 'browser-side, manual'),
        ('15.4', 'thermal under 20 min load', 'long run, manual'),
    ]:
        record(cid, 'manual', name, 'SKIP', why)


# ----------------------------------------------------------------------------- main
def main():
    """Parse arguments, run the layers, print the summary, write the JSON report."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--motion', action='store_true',
                    help='also run motion checks on the real servos and tracks')
    ap.add_argument('--tests', action='store_true',
                    help='also run the full colcon test suite (slow)')
    ap.add_argument('--quick', action='store_true', help='skip the mock grasp')
    ap.add_argument('--no-ros', action='store_true', help='hardware and software layers only')
    ap.add_argument('--report-dir', default=str(Path.home() / 'jetank_checks'))
    args = ap.parse_args()

    if args.motion and args.no_ros:
        ap.error('--motion needs the ROS layer; drop --no-ros')
    if args.motion:
        if not sys.stdin.isatty():
            ap.error('--motion needs an interactive terminal for the typed confirmations')
        print('--motion: the real servos get torque for the whole run and go LIMP when it '
              'ends,\n  so the arm sinks under its own weight. Support the arm or keep it in its '
              'rest pose.')
        if not confirm('Torque the servos now?'):
            print('Not confirmed; running without --motion.')
            args.motion = False

    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    report_dir = Path(args.report_dir)
    logdir = report_dir / f'logs-{stamp}'
    try:
        logdir.mkdir(parents=True, exist_ok=True)
    except OSError as e:
        print(f'cannot create report directory {report_dir}: {e}', file=sys.stderr)
        return 2
    print(f'JeTank state check {stamp}  (logs: {logdir})')

    def on_signal(signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGHUP, on_signal)

    t0 = time.monotonic()
    layers = [(check_hardware, 'H', 'hardware'), (check_software, 'S', 'software')]
    if not args.no_ros:
        layers.append((lambda a: check_ros(a, logdir), 'R', 'ros'))
        if args.motion:
            layers.append((check_motion, 'M', 'motion'))
    try:
        for fn, cid, part in layers:
            guarded(cid, part, f'{part} layer')(fn)(args)
        not_automated()
    except KeyboardInterrupt:
        print('\n  interrupted — stopping what this run started', flush=True)
        record('X', 'run', 'run interrupted', 'FAIL', 'stopped early')
    finally:
        teardown()
        counts = {s: sum(1 for r in RESULTS if r['status'] == s)
                  for s in ('PASS', 'FAIL', 'WARN', 'SKIP', 'INFO')}
        fails = [r for r in RESULTS if r['status'] == 'FAIL']
        print(f"\n== Summary: {counts['PASS']} pass, {counts['FAIL']} fail, "
              f"{counts['WARN']} warn, {counts['SKIP']} skipped  "
              f"({time.monotonic() - t0:.0f} s)")
        for r in fails:
            print(f"  FAIL {r['id']:6} {r['name']} — {r['value']}")
        report = {'time': stamp, 'host': os.uname().nodename, 'args': vars(args),
                  'counts': counts, 'results': RESULTS}
        path = report_dir / f'check-{stamp}.json'
        try:
            path.write_text(json.dumps(report, indent=1))
            print(f'  report: {path}')
        except OSError as e:
            print(f'  report not written: {e}')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
