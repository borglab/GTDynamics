#!/usr/bin/env python3
"""ROS <-> UR10e, replacing the Simulink tcpUR10eBlock and UR10eSend.

State:    reads realtime packets from port 30003 and publishes the six arm
          joints on /ur/joint_states (position and velocity), and the
          controller's speed scaling on /ur/speed_scaling.
Command:  /ur/joint_velocity_command (Float64MultiArray, six rad/s), by one of
          three transports (parameter `stream`):

  rtde     (default) writes the velocities and a sequence counter to RTDE
           input registers (port 30004) and uploads ONE URScript program to
           30003 that reads them and runs speedj on the robot at 125 Hz. Every
           connection goes from this PC to the robot: no firewall change.
  program  the same persistent program, but it connects back to this PC
           (stream_port) for the velocities: needs inbound TCP to the PC.
  lines    sends a new one-line `speedj(...)` program per command, exactly as
           the Simulink model did. Each one replaces the running program, so
           the controller restarts a program 20 times a second.

The persistent programs (rtde, program) zero the arm by themselves when new
commands stop arriving for command_timeout, even if this process or the
network fails.

With commands enabled, the arm is supervised as a small state machine, from a
thread of its own (not the ROS executor, so a stalled callback cannot hold up
a stop). Measured joint speed (qd) and the controller's target (qd_target)
come from the realtime packet:

  stopping  zero velocity is resent every 0.1 s until the stop is VERIFIED:
            measured and target speed below 0.005 rad/s on every joint for
            0.2 s. If that takes longer than the deceleration allows, a fault
            latches and the program is ended: stopj over 30003, then the
            dashboard server's `stop` (29999).
  idle      nothing is commanded. Any joint moving over 0.02 rad/s for 0.1 s
            is a fault (another program, freedrive, a stream that did not
            zero) and is stopped as above.
  moving    commands flow. Each of these faults and stops: a gap in commands
            over command_timeout; stale UR state; robot mode not RUNNING or
            safety mode not NORMAL; speed scaling below 100%; another node
            publishing commands; the arm delivering under half of the
            commanded velocity for a second (it would lag the rail).

/ur/status says 'ready' only with commands enabled, no fault, the streaming
program running, fresh state, robot mode RUNNING, safety mode NORMAL, speed
scaling at 100% (while a program plays), no foreign command publisher, and a
verified still arm since start-up or the last reset.

Diagnostics: mode changes (robot, safety, program state) and speed scaling
are logged as they change. While commands flow, one line per second compares
what was commanded, what the controller is targeting (its qd_target), and
what the arm actually does, and says which of them disagrees:

  arm: 20 cmd/s | wrist_3 cmd +0.0500 target +0.0500 (100%) actual +0.0497 (99%) |
       scaling 1.00 | RUNNING (7) | NORMAL (1) | PLAYING (2) | stream connected

Safety behaviour:
  * enable_commands is false by default: shadow mode reads state and logs
    what it would send; it uploads nothing, stops nothing and supervises
    nothing, since the robot may be running someone else's program.
  * faults latch until ~/reset_fault, which re-verifies a still arm and
    restarts a lost streaming program.
  * only nodes named in allowed_command_nodes may publish commands ('any'
    lifts this, e.g. for a hand-run `ros2 topic pub` check).

The robot must be in Remote Control mode to run programs sent over the
network. A program sent to 30003 replaces the running one, so do not run this
alongside another program (pendant or ur_robot_driver) that must keep running.
"""

import socket
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64, Float64MultiArray, String
from std_srvs.srv import Trigger

from scate_ur10e_hw import hardware_protocol as wire
from scate_ur10e_hw.configurations import JOINT_NAMES
from scate_ur10e_hw.ros_params import declare

ARM_JOINT_NAMES = list(JOINT_NAMES[1:])
COMMAND_TOPIC = '/ur/joint_velocity_command'
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
STREAMS = ('rtde', 'program', 'lines')
PERSISTENT = ('rtde', 'program')
READY = ('connected', 'running')
STARTING = ('not started', 'connecting to RTDE', 'uploaded, waiting for PLAYING',
            'waiting for the robot')
# Stop verification: every joint, measured and target, below STOPPED_RAD_S
# for SETTLE_S.
STOPPED_RAD_S = 0.005
SETTLE_S = 0.2
STOP_RESEND_S = 0.1
# Idle monitor: motion nobody commanded, sustained so one noisy sample
# cannot trip it.
IDLE_RAD_S = 0.02
IDLE_PERSIST_S = 0.1
# UR state older than this is stale (packets arrive at 500 Hz).
STATE_TIMEOUT_S = 0.2
# A moving arm must deliver at least FOLLOW_MIN of the commanded velocity;
# judged only on commands of at least FOLLOW_MIN_COMMAND rad/s (vector norm),
# once a stream has run FOLLOW_AFTER_S, over FOLLOW_WINDOW_S.
FOLLOW_MIN = 0.5
FOLLOW_MIN_COMMAND = 0.02
FOLLOW_AFTER_S = 1.0
FOLLOW_WINDOW_S = 1.0
SAFETY_PERIOD_S = 0.02


def _receive_exactly(sock, count):
    data = bytearray()
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise ConnectionError('UR closed the realtime connection')
        data.extend(chunk)
    return bytes(data)


def detect_host_ip(robot_ip, port):
    """The local address this PC uses to reach the robot."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((robot_ip, port))
        return probe.getsockname()[0]
    finally:
        probe.close()


def follow_ratio(values, command):
    """How much of the commanded velocity `values` delivers (1.0 = all of it)."""
    if values is None:
        return None
    denominator = float(np.dot(command, command))
    if denominator < 1e-8:
        return None
    return float(np.dot(values, command)) / denominator


def allowed_nodes(text):
    """Node names allowed to publish commands; empty means any."""
    names = {name.strip() for name in text.split(',') if name.strip()}
    return set() if names == {'any'} else names


class URBridge(Node):

    def __init__(self):
        super().__init__('ur_bridge')
        self.robot_ip = declare(self, 'robot_ip', '192.168.7.8')
        self.port = declare(self, 'port', wire.UR_REALTIME_PORT)
        self.dashboard_port = declare(self, 'dashboard_port', wire.UR_DASHBOARD_PORT)
        self.enable_commands = declare(self, 'enable_commands', False)
        self.stream = declare(self, 'stream', 'rtde')
        self.rtde_port = declare(self, 'rtde_port', wire.UR_RTDE_PORT)
        self.host_ip = declare(self, 'host_ip', '')
        self.stream_port = declare(self, 'stream_port', 50010)
        self.acceleration = declare(self, 'acceleration', 1.0)
        self.stop_deceleration = declare(self, 'stop_deceleration', 2.0)
        self.speedj_time = declare(self, 'speedj_time', 0.2)
        self.command_timeout = declare(self, 'command_timeout', 0.25)
        self.max_joint_speed = declare(self, 'max_joint_speed', 0.5)
        self.publish_hz = declare(self, 'publish_hz', 125.0)
        self.connect_timeout = declare(self, 'connect_timeout', 5.0)
        # Used for the start-up stop, before any speed has been measured.
        self.default_stop_timeout = declare(self, 'stop_timeout', 5.0)
        self.speed_scaling_check = declare(self, 'speed_scaling_check', True)
        self.allowed_command_nodes = allowed_nodes(
            declare(self, 'allowed_command_nodes', 'scate_executor'))
        if self.stream not in STREAMS:
            raise ValueError(f'stream must be one of {STREAMS}')

        self.state_pub = self.create_publisher(JointState, '/ur/joint_states', 10)
        self.scaling_pub = self.create_publisher(Float64, '/ur/speed_scaling', 10)
        # 'ready', or why commands would not reach the arm; the executor
        # refuses to move anything unless this says 'ready'.
        self.status_pub = self.create_publisher(String, '/ur/status', LATCHED)
        self.published_status = None
        self.create_subscription(Float64MultiArray, COMMAND_TOPIC, self.on_command, 10)
        self.create_service(Trigger, '~/reset_fault', self.on_reset)
        self.create_timer(1.0, self.report_tracking)
        self.create_timer(0.5, self.check_command_sources)

        # self.lock guards the state below; self.send_lock only the command
        # sockets. A thread may take send_lock while holding lock, never the
        # other way round.
        self.lock = threading.RLock()
        self.send_lock = threading.Lock()
        self.latest = None
        self.latest_at = 0.0
        self.logged = {}
        self.command_sock = None      # lines mode
        self.stream_sock = None       # program mode: the robot's connection back
        self.rtde_sock = None         # rtde mode: this PC's connection to 30004
        self.rtde_recipe = None
        self.rtde_base = wire.RTDE_REGISTER_BASES[0]
        self.rtde_sequence = 0
        self.stream_started = 0.0
        self.server = None
        self.stream_state = 'not started'
        self.fault = None
        # Supervision: 'stopping', 'idle' or 'moving'. Enabled bridges start
        # by verifying the arm is still; shadow bridges supervise nothing.
        self.state = 'stopping' if self.enable_commands else 'idle'
        self.verified_once = False
        now = time.monotonic()
        self.stop_requested_at = now
        self.stop_deadline = now + self.default_stop_timeout + self.connect_timeout
        self.last_stop_sent = 0.0
        self.escalated = False
        self.still_since = None
        self.idle_motion_since = None
        self.moving_since = 0.0
        self.low_follow_since = None
        self.foreign_publishers = []
        self.last_command = 0.0
        self.last_vector = np.zeros(6)
        self.commands_this_period = 0
        self.streaming_since = 0.0
        self.running = True
        self.closed = False
        self.threads = [threading.Thread(target=target, daemon=True)
                        for target in (self.read_state, self.safety_loop)]
        for thread in self.threads:
            thread.start()

        if not self.enable_commands:
            mode = 'shadow mode: reading state only, nothing is sent to the robot'
        else:
            mode = f'COMMANDS ENABLED, stream={self.stream}'
        self.get_logger().info(
            f'UR bridge to {self.robot_ip}:{self.port}, {mode}; commands accepted from '
            f'{sorted(self.allowed_command_nodes) or "any node"}')
        if self.enable_commands and self.stream in PERSISTENT:
            self.start_stream()

    # --- state -------------------------------------------------------------

    def read_state(self):
        period = 1.0 / self.publish_hz
        while self.running:
            try:
                with socket.create_connection((self.robot_ip, self.port), timeout=2.0) as sock:
                    sock.settimeout(1.0)
                    self.get_logger().info(f'Receiving UR realtime state from {self.robot_ip}')
                    last_publish = 0.0
                    logged_length = False
                    while self.running:
                        header = _receive_exactly(sock, 4)
                        length = wire.ur_packet_length(header)
                        packet = header + _receive_exactly(sock, length - 4)
                        now = time.monotonic()
                        if now - last_publish < period:
                            continue
                        last_publish = now
                        state = wire.parse_ur_realtime(packet)
                        if not logged_length:
                            logged_length = True
                            self.get_logger().info(
                                f'UR realtime packet: {length} bytes, '
                                f'{(length - 4) // 8} values')
                        with self.lock:
                            self.latest, self.latest_at = state, now
                        if not self.running:
                            break
                        message = JointState()
                        message.header.stamp = self.get_clock().now().to_msg()
                        message.name = ARM_JOINT_NAMES
                        message.position = state['q'].tolist()
                        message.velocity = state['qd'].tolist()
                        self.publish(self.state_pub, message)
                        scaling = self.playing_scaling(state)
                        if scaling is not None:
                            self.publish(self.scaling_pub, Float64(data=scaling))
                        self.report_changes(state)
            except (OSError, ValueError, ConnectionError) as error:
                if self.running:
                    self.get_logger().warn(f'UR state connection: {error}; retrying',
                                           throttle_duration_sec=5.0)
                    time.sleep(1.0)

    def fresh_state(self):
        """The latest realtime state if it is recent, else None."""
        with self.lock:
            if self.latest is None or time.monotonic() - self.latest_at > STATE_TIMEOUT_S:
                return None
            return self.latest

    @staticmethod
    def playing_scaling(state):
        """Speed scaling, or None when no program is playing (it then reads 0)."""
        playing = (state['program_state'] is None
                   or int(round(state['program_state'])) == wire.UR_PLAYING)
        return state['speed_scaling'] if playing else None

    @staticmethod
    def mode_problem(state):
        """Why the controller would not run motion now, from its modes, or None."""
        robot, safety = state['robot_mode'], state['safety_mode']
        if robot is not None and int(round(robot)) != wire.UR_RUNNING:
            return (f'robot mode {wire.ur_mode_name(wire.UR_ROBOT_MODES, robot)}: power on '
                    'and release the brakes')
        if safety is not None and int(round(safety)) != wire.UR_SAFETY_NORMAL:
            return (f'safety mode {wire.ur_mode_name(wire.UR_SAFETY_MODES, safety)}: clear '
                    'the stop on the pendant')
        return None

    def report_changes(self, state):
        """Log robot mode, safety mode, program state and scaling when they change."""
        fields = (
            ('robot mode', wire.ur_mode_name(wire.UR_ROBOT_MODES, state['robot_mode']),
             ('RUNNING',)),
            ('safety mode', wire.ur_mode_name(wire.UR_SAFETY_MODES, state['safety_mode']),
             ('NORMAL',)),
            ('program state', wire.ur_mode_name(wire.UR_PROGRAM_STATES, state['program_state']),
             ('PLAYING', 'STOPPED')),
        )
        for label, value, normal in fields:
            if value is None or self.logged.get(label) == value:
                continue
            self.logged[label] = value
            if label == 'program state':
                self.track_program(value)
            # One call site per severity: rclpy raises if a single logging call
            # changes severity between calls, which used to drop the state link.
            if value.split()[0] in normal:
                self.get_logger().info(f'UR {label}: {value}')
            else:
                self.get_logger().warn(f'UR {label}: {value}')
        scaling = self.playing_scaling(state)
        if scaling is not None and abs(scaling - self.logged.get('scaling', -1.0)) > 0.01:
            self.logged['scaling'] = scaling
            if scaling >= 0.99:
                self.get_logger().info(f'UR speed scaling {scaling:.2f} while playing')
            else:
                self.get_logger().warn(
                    f'UR speed scaling {scaling:.2f} while playing: the arm runs at '
                    f'{scaling * 100:.0f}% of commanded speed (speed slider, reduced mode or '
                    'a safety limit)')

    def report_tracking(self):
        """Once a second while commanding: commanded vs. controller target vs. actual."""
        count, self.commands_this_period = self.commands_this_period, 0
        if not self.enable_commands or time.monotonic() - self.last_command > 1.0:
            return
        with self.lock:
            state = self.latest
        command = self.last_vector
        joint = int(np.argmax(np.abs(command)))
        parts = [f'arm: {count} cmd/s']
        target = actual = None
        if state is None:
            parts.append('no UR state received')
        else:
            target = follow_ratio(state['qd_target'], command)
            actual = follow_ratio(state['qd'], command)

            def percent(ratio):
                return 'n/a' if ratio is None else f'{ratio * 100:.0f}%'

            parts.append(f'{ARM_JOINT_NAMES[joint]} cmd {command[joint]:+.4f} '
                         f'target {state["qd_target"][joint]:+.4f} ({percent(target)}) '
                         f'actual {state["qd"][joint]:+.4f} ({percent(actual)})')
            scaling = self.playing_scaling(state)
            parts.append('scaling n/a (no program playing)' if scaling is None
                         else f'scaling {scaling:.2f}')
            for table, key in ((wire.UR_ROBOT_MODES, 'robot_mode'),
                               (wire.UR_SAFETY_MODES, 'safety_mode'),
                               (wire.UR_PROGRAM_STATES, 'program_state')):
                name = wire.ur_mode_name(table, state[key])
                if name is not None:
                    parts.append(name)
        if self.stream in PERSISTENT:
            parts.append(f'stream {self.stream_state}')
        self.get_logger().info(' | '.join(parts))
        # A stream that just started or just ended is not a slow stream.
        now = time.monotonic()
        steady = now - self.last_command < 0.2 and now - self.streaming_since > 1.5
        for problem in self.diagnose(count if steady else 20, state, target, actual):
            self.get_logger().warn(f'arm diagnosis: {problem}')

    def not_ready(self):
        """Why commands would not move the arm safely now, or None."""
        with self.lock:
            if not self.enable_commands:
                return 'shadow mode (enable_commands:=false): arm commands are not sent'
            if self.fault:
                return f'fault: {self.fault}'
            if self.stream in PERSISTENT and self.stream_state not in READY:
                return f'streaming program {self.stream_state}'
            state = self.fresh_state()
            if state is None:
                return 'no fresh UR state (realtime port 30003)'
            problem = self.mode_problem(state)
            if problem:
                return problem
            scaling = self.playing_scaling(state)
            if self.speed_scaling_check and scaling is not None and scaling < 0.99:
                return (f'speed scaling {scaling:.2f}: set the pendant speed slider to 100% '
                        'and leave reduced mode')
            if self.foreign_publishers:
                return f'other nodes publish {COMMAND_TOPIC}: {self.foreign_publishers}'
            if not self.verified_once:
                return 'verifying the arm is still'
            return None

    def status(self):
        return self.not_ready() or 'ready'

    def publish_status(self):
        status = self.status()
        if status != self.published_status:
            self.published_status = status
            self.publish(self.status_pub, String(data=status))
            self.get_logger().info(f'arm bridge status: {status}')

    def publish(self, publisher, message):
        try:
            publisher.publish(message)
        except Exception:
            # The ROS context is gone during shutdown; supervision carries on.
            pass

    def diagnose(self, count, state, target, actual):
        """Plain-language reasons why the arm is not following, if any."""
        problems = []
        if count < 0.75 * 20:
            problems.append(f'only {count} commands arrived in the last second '
                            '(expected about 20): executor or ROS transport is slow')
        if state is None or target is None:
            return problems
        scaling = self.playing_scaling(state)
        if scaling is not None and scaling < 0.99:
            problems.append(f'speed scaling {scaling:.2f} slows every arm motion')
        if target < 0.8:
            where = (f'the streaming program is {self.stream_state}'
                     if self.stream in PERSISTENT and self.stream_state not in READY
                     else 'the program may not be running; check program state and the UR log')
            problems.append(f'the controller targets only {target * 100:.0f}% of the '
                            f'commanded velocity: {where}')
        elif actual is not None and actual < 0.8:
            problems.append(f'the controller targets the command, but the arm delivers only '
                            f'{actual * 100:.0f}%: check safety mode and joint limits')
        return problems

    # --- dashboard server ------------------------------------------------------

    def dashboard(self, command):
        """One dashboard command and its reply line; never raises."""
        try:
            with socket.create_connection((self.robot_ip, self.dashboard_port),
                                          timeout=1.0) as sock:
                sock.settimeout(1.0)
                reader = sock.makefile('rb')
                reader.readline()                               # banner
                sock.sendall(command.encode('ascii') + b'\n')
                return reader.readline().decode('ascii', 'replace').strip()
        except OSError as error:
            return f'dashboard {self.robot_ip}:{self.dashboard_port} unreachable ({error})'

    # --- persistent streaming program ------------------------------------------

    def start_stream(self):
        if self.stream == 'rtde':
            self.start_rtde()
        else:
            self.start_program_stream()

    def connect_rtde(self, attempts=5):
        """A started RTDE session as (socket, recipe, register base).

        Tries the preferred register set, then the lower one the controller
        may be limited to; retries while a previous session holds them.
        """
        for base in wire.RTDE_REGISTER_BASES:
            for attempt in range(attempts):
                sock = socket.create_connection((self.robot_ip, self.rtde_port), timeout=2.0)
                try:
                    recipe = wire.rtde_setup_inputs(sock, on_message=self.log_rtde_message,
                                                    base=base)
                    sock.settimeout(None)
                    return sock, recipe, base
                except wire.RTDEInputsRefused as error:
                    sock.close()
                    if error.not_found and base != wire.RTDE_REGISTER_BASES[-1]:
                        self.get_logger().warn(
                            f'This controller has no RTDE input registers {base}..{base + 5}; '
                            'trying the lower register range')
                        break
                    if not error.in_use or attempt == attempts - 1:
                        raise
                    self.get_logger().warn('RTDE registers still held by a previous session; '
                                           'retrying in 0.5 s')
                    time.sleep(0.5)
                except BaseException:
                    sock.close()
                    raise
        raise ConnectionError('no usable RTDE input registers')

    def start_rtde(self):
        """Connect to RTDE (or reuse the link), zero the registers, upload the program."""
        self.stream_state = 'connecting to RTDE'
        try:
            if self.rtde_sock is None:
                self.close_stream()
                sock, recipe, base = self.connect_rtde()
                self.rtde_sock, self.rtde_recipe, self.rtde_base = sock, recipe, base
                names = wire.rtde_input_names(base)
                self.get_logger().info(
                    f'RTDE connected to {self.robot_ip}:{self.rtde_port}; writing arm velocities '
                    f'to {names[0]}..{base + 5} and a sequence counter to {names[6]}')
                reader = threading.Thread(target=self.read_rtde, args=(sock,), daemon=True)
                reader.start()
                self.threads.append(reader)
            else:
                self.get_logger().info('Reusing the RTDE connection')
            if not self.send_velocities(np.zeros(6)):
                raise ConnectionError('could not write the initial zero velocity')
        except (OSError, ConnectionError, ValueError, IndexError) as error:
            self.stream_state = 'RTDE failed'
            self.get_logger().error(
                f'RTDE setup with {self.robot_ip}:{self.rtde_port} failed: {error}. Check that '
                'RTDE is enabled on the robot (Settings > Security > Services) and that no '
                'other RTDE client (ur_robot_driver, a second bridge) owns the input '
                'registers named above.')
            self.trip('RTDE setup failed')
            return
        self.stream_state = 'uploaded, waiting for PLAYING'
        self.stream_started = time.monotonic()
        self.get_logger().info(
            f'Uploading the streaming program ros_speedj_rtde to {self.robot_ip}:{self.port}; '
            f'it runs speedj at 125 Hz and zeroes the arm after {self.command_timeout} s '
            'without a new command')
        if not self.upload(wire.rtde_program(self.acceleration, self.stop_deceleration,
                                             self.command_timeout, base=self.rtde_base)):
            self.stream_state = 'upload failed'
            self.trip('could not upload the streaming program')

    def check_rtde_started(self):
        """Notice the uploaded program playing, or report once that it never did."""
        if self.stream_state != 'uploaded, waiting for PLAYING':
            return
        with self.lock:
            state = self.latest
        elapsed = time.monotonic() - self.stream_started
        if (elapsed > 0.3 and state is not None and state['program_state'] is not None
                and int(round(state['program_state'])) == wire.UR_PLAYING):
            self.stream_state = 'running'
            self.get_logger().info('Streaming program running on the robot; ready for commands')
            return
        if elapsed > self.connect_timeout:
            self.stream_state = 'never started'
            remote = self.dashboard('is in remote control')
            self.get_logger().error(
                f'The streaming program did not start within {self.connect_timeout} s '
                f'(program state never PLAYING). Dashboard, asked "is in remote control": '
                f'{remote!r}. Check, in order: Remote Control mode on the pendant; the '
                'pendant Log tab for "ros_speedj_rtde" or a program error; robot powered on '
                'with brakes released (robot mode RUNNING).')
            self.trip('streaming program did not start')

    def track_program(self, value):
        """Program state changes: our program started, or it ended."""
        name = value.split()[0]
        if self.stream != 'rtde' or not self.enable_commands:
            return
        if name == 'PLAYING' and self.stream_state == 'uploaded, waiting for PLAYING':
            self.stream_state = 'running'
            self.get_logger().info('Streaming program running on the robot; ready for commands')
        elif name in ('STOPPING', 'STOPPED') and self.stream_state == 'running' and self.running:
            self.stream_state = 'stopped'
            self.trip('the streaming program on the robot ended (stopped from the pendant, '
                      'protective stop, or replaced by another program); '
                      'call ~/reset_fault to restart it')

    def log_rtde_message(self, text):
        self.get_logger().warn(f'RTDE message from the controller: {text}')

    def read_rtde(self, sock):
        """Drain RTDE output packets, log controller messages, notice a lost link."""
        try:
            while True:
                kind, payload = wire.rtde_read(sock)
                if kind == wire.RTDE_TEXT_MESSAGE:
                    self.log_rtde_message(wire.rtde_text(payload))
        except (OSError, ConnectionError):
            pass
        if self.rtde_sock is sock and self.running:
            self.rtde_sock = None
            self.stream_state = 'RTDE lost'
            self.trip('the RTDE connection to the robot closed; call ~/reset_fault to reconnect')

    def start_program_stream(self):
        """Listen, upload the program, and wait for the robot to connect back."""
        host = self.host_ip or detect_host_ip(self.robot_ip, self.port)
        program = wire.stream_program(host, self.stream_port, self.acceleration,
                                      self.stop_deceleration, self.command_timeout)
        self.close_stream()
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.stream_port))
        server.listen(1)
        server.settimeout(self.connect_timeout)
        self.server = server
        self.stream_state = 'waiting for the robot'
        self.get_logger().info(
            f'Uploading the streaming program to {self.robot_ip}:{self.port}; it connects back '
            f'to {host}:{self.stream_port}, runs speedj at 125 Hz and zeroes the arm after '
            f'{self.command_timeout} s without commands')
        if not self.upload(program):
            self.stream_state = 'upload failed'
            self.trip('could not upload the streaming program')
            return
        threading.Thread(target=self.accept_stream, args=(server, host), daemon=True).start()

    def upload(self, script):
        try:
            with socket.create_connection((self.robot_ip, self.port), timeout=2.0) as sock:
                sock.sendall(script)
            return True
        except OSError as error:
            self.get_logger().error(f'program upload to {self.robot_ip}:{self.port} failed: '
                                    f'{error}')
            return False

    def accept_stream(self, server, host):
        try:
            connection, peer = server.accept()
        except OSError:
            if self.server is server:
                self.stream_state = 'robot never connected'
                self.get_logger().error(
                    f'The robot did not connect back to {host}:{self.stream_port} within '
                    f'{self.connect_timeout} s. Check, in order: the UR log tab for '
                    '"ros_speedj_stream" messages or a program error; Remote Control mode; '
                    f'that {host} is this PC\'s address on the robot network (else set '
                    'host_ip); and the PC firewall for inbound TCP '
                    f'{self.stream_port} (sudo ufw status).')
                self.trip('streaming program did not connect')
            return
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.stream_sock = connection
        self.stream_state = 'connected'
        self.get_logger().info(f'Streaming program connected from {peer[0]}; ready for commands')
        threading.Thread(target=self.watch_stream, args=(connection,), daemon=True).start()

    def watch_stream(self, connection):
        """The robot never sends on this socket; EOF means its program ended."""
        try:
            while connection.recv(1024):
                pass
        except OSError:
            pass
        if self.stream_sock is connection and self.running:
            self.stream_sock = None
            self.stream_state = 'disconnected'
            self.trip('the streaming program on the robot ended (stopped from the pendant, '
                      'protective stop, or replaced by another program); '
                      'call ~/reset_fault to restart it')

    def close_stream(self):
        rtde, self.rtde_sock = self.rtde_sock, None
        if rtde is not None:
            try:
                rtde.sendall(wire.rtde_packet(wire.RTDE_CONTROL_PACKAGE_PAUSE))
            except OSError:
                pass
        for sock in (self.stream_sock, self.server, rtde):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self.stream_sock = self.server = None

    # --- commands ----------------------------------------------------------

    def send_velocities(self, velocities):
        """Deliver one velocity vector; return False, never raise, on failure."""
        if not self.enable_commands:
            vector = ', '.join(f'{v:+.4f}' for v in velocities)
            self.get_logger().info(f'[shadow] would command arm velocities [{vector}] rad/s',
                                   throttle_duration_sec=1.0)
            return True
        if self.stream == 'rtde':
            if self.rtde_sock is None:
                self.get_logger().error(f'no RTDE connection ({self.stream_state})',
                                        throttle_duration_sec=2.0)
                return False
            try:
                with self.send_lock:
                    self.rtde_sequence += 1
                    self.rtde_sock.sendall(wire.rtde_inputs(self.rtde_recipe, velocities,
                                                            self.rtde_sequence))
                return True
            except (OSError, AttributeError) as error:
                self.get_logger().error(f'RTDE send failed: {error}')
                return False
        if self.stream == 'program':
            if self.stream_sock is None:
                self.get_logger().error(f'no streaming program connection ({self.stream_state})',
                                        throttle_duration_sec=2.0)
                return False
            try:
                with self.send_lock:
                    self.stream_sock.sendall(wire.stream_vector(velocities))
                return True
            except (OSError, AttributeError) as error:
                self.get_logger().error(f'stream send failed: {error}')
                return False
        return self.send_line(wire.speedj_command(velocities, self.acceleration,
                                                  self.speedj_time))

    def send_line(self, line):
        with self.send_lock:
            try:
                if self.command_sock is None:
                    sock = socket.create_connection((self.robot_ip, self.port), timeout=1.0)
                    # Port 30003 streams state on every connection, this one too.
                    # Drain it, or the controller's send buffer backs up.
                    threading.Thread(target=self.drain, args=(sock,), daemon=True).start()
                    self.command_sock = sock
                self.command_sock.sendall(line)
                return True
            except OSError as error:
                self.close_command_socket_locked()
                self.get_logger().error(f'UR command send failed: {error}')
                return False

    @staticmethod
    def drain(sock):
        try:
            while sock.recv(65536):
                pass
        except OSError:
            pass

    def deliver_stop(self):
        """Zero velocity through the stream, or stopj when there is no stream."""
        self.last_vector = np.zeros(6)
        if (self.stream == 'program' and self.stream_sock is not None) or (
                self.stream == 'rtde' and self.rtde_sock is not None):
            if self.send_velocities(np.zeros(6)):
                return True
        if self.stream in PERSISTENT and self.stream_state in STARTING:
            # The program being uploaded replaces whatever runs and starts at
            # zero; a stopj now could replace it, or be taken for it.
            return True
        if self.send_line(wire.stopj_command(self.stop_deceleration)):
            return True
        self.get_logger().error('ARM STOP DELIVERY FAILED: use the pendant stop.',
                                throttle_duration_sec=1.0)
        return False

    def stop_timeout(self, speed):
        """How long a stop from `speed` rad/s may take before it counts as failed."""
        if speed is None:
            return self.default_stop_timeout
        deceleration = min(self.acceleration, self.stop_deceleration)
        return 1.5 * speed / deceleration + 1.0 + SETTLE_S

    @staticmethod
    def joint_speed(state):
        """Fastest joint, measured or targeted, in rad/s."""
        return float(max(np.max(np.abs(state['qd'])), np.max(np.abs(state['qd_target']))))

    def request_stop(self, reason, fault=False):
        """Zero the arm now; the supervisor resends until the stop is verified."""
        with self.lock:
            if fault and self.fault is None:
                self.fault = reason
                self.get_logger().error(f'UR bridge fault: {reason}')
            if not self.enable_commands:
                self.last_vector = np.zeros(6)
                self.get_logger().info(f'[shadow] stop ({reason})', throttle_duration_sec=1.0)
                return
            now = time.monotonic()
            if self.state != 'stopping':
                self.get_logger().info(f'arm stopping ({reason})')
                state = self.fresh_state()
                self.stop_requested_at = now
                self.stop_deadline = now + self.stop_timeout(
                    None if state is None else self.joint_speed(state))
                self.escalated = False
                self.state = 'stopping'
            self.still_since = None
            self.low_follow_since = None
            self.last_stop_sent = now
            self.deliver_stop()

    def trip(self, reason):
        self.request_stop(reason, fault=True)

    # --- supervision -------------------------------------------------------------

    def safety_loop(self):
        while self.running:
            try:
                self.supervise(time.monotonic())
            except Exception as error:  # the supervisor must never die
                self.get_logger().error(f'arm supervisor error: {error!r}')
                try:
                    self.trip(f'supervisor error: {error!r}')
                except Exception:
                    pass
            self.publish_status()
            time.sleep(SAFETY_PERIOD_S)

    def supervise(self, now):
        with self.lock:
            if self.enable_commands and self.stream == 'rtde':
                self.check_rtde_started()
            if not self.enable_commands:
                return
            state = self.fresh_state()
            if state is not None and self.joint_speed(state) < STOPPED_RAD_S:
                self.still_since = self.still_since or now
            else:
                self.still_since = None
            if self.state == 'stopping':
                self.supervise_stop(now, state)
            elif self.state == 'idle':
                self.supervise_idle(now, state)
            elif self.state == 'moving':
                self.supervise_move(now, state)

    def supervise_stop(self, now, state):
        if now - self.last_stop_sent >= STOP_RESEND_S:
            self.last_stop_sent = now
            self.deliver_stop()
        if self.still_since is not None and now - self.still_since >= SETTLE_S:
            self.state = 'idle'
            self.idle_motion_since = None
            if not self.verified_once:
                self.verified_once = True
                self.get_logger().info('arm verified still')
            return
        if now > self.stop_deadline and not self.escalated:
            self.escalated = True
            why = ('UR state is stale' if state is None else
                   f'fastest joint at {self.joint_speed(state):.3f} rad/s')
            # End whatever program drives the arm: stopj replaces it, and the
            # dashboard stops what remains.
            self.send_line(wire.stopj_command(self.stop_deceleration))
            reply = self.dashboard('stop')
            reason = (f'stop not verified {now - self.stop_requested_at:.1f} s after zeroing: '
                      f'{why}; sent stopj and dashboard stop ({reply!r})')
            if self.fault is None:
                self.fault = reason
            self.get_logger().error(f'{reason}. USE THE PENDANT E-STOP if the arm moves')

    def supervise_idle(self, now, state):
        if state is None:
            return
        speed = self.joint_speed(state)
        if speed <= IDLE_RAD_S:
            self.idle_motion_since = None
            return
        self.idle_motion_since = self.idle_motion_since or now
        if now - self.idle_motion_since >= IDLE_PERSIST_S:
            joint = int(np.argmax(np.maximum(np.abs(state['qd']), np.abs(state['qd_target']))))
            self.trip(f'arm moving without a command ({ARM_JOINT_NAMES[joint]} at '
                      f'{speed:.3f} rad/s): another program, freedrive, or a stream that '
                      'did not zero')

    def supervise_move(self, now, state):
        if state is None:
            self.trip('UR state stale while moving')
            return
        if now - self.last_command > self.command_timeout:
            self.trip(f'no arm command for {now - self.last_command:.2f} s while moving')
            return
        if self.foreign_publishers:
            self.trip(f'other nodes publish {COMMAND_TOPIC}: {self.foreign_publishers}')
            return
        problem = self.mode_problem(state)
        if problem:
            self.trip(f'while moving: {problem}')
            return
        scaling = self.playing_scaling(state)
        if self.speed_scaling_check and scaling is not None and scaling < 0.99:
            self.trip(f'speed scaling dropped to {scaling:.2f} while moving')
            return
        command = self.last_vector
        ratio = follow_ratio(state['qd'], command)
        if (ratio is None or np.linalg.norm(command) < FOLLOW_MIN_COMMAND
                or now - self.moving_since < FOLLOW_AFTER_S or ratio >= FOLLOW_MIN):
            self.low_follow_since = None
            return
        self.low_follow_since = self.low_follow_since or now
        if now - self.low_follow_since >= FOLLOW_WINDOW_S:
            self.trip(f'the arm delivers only {ratio * 100:.0f}% of the commanded velocity '
                      f'for {FOLLOW_WINDOW_S:.0f} s: it would lag the rail')

    def check_command_sources(self):
        """Names of nodes other than the allowed ones publishing arm commands."""
        foreign = []
        if self.allowed_command_nodes:
            try:
                infos = self.get_publishers_info_by_topic(COMMAND_TOPIC)
            except Exception as error:
                self.get_logger().warn(f'cannot list {COMMAND_TOPIC} publishers: {error}',
                                       throttle_duration_sec=10.0)
                infos = []
            foreign = sorted({info.node_name for info in infos
                              if info.node_name not in self.allowed_command_nodes})
        with self.lock:
            if foreign and foreign != self.foreign_publishers:
                self.get_logger().error(
                    f'{COMMAND_TOPIC} is also published by {foreign}; the arm will not move '
                    f'until only {sorted(self.allowed_command_nodes)} publish it')
            self.foreign_publishers = foreign
        return foreign

    # --- ROS interface -----------------------------------------------------------

    def on_command(self, message):
        velocities = np.asarray(message.data, dtype=float)
        foreign = self.check_command_sources()
        with self.lock:
            if self.fault:
                self.get_logger().warn(f'ignoring command, fault latched: {self.fault}',
                                       throttle_duration_sec=2.0)
                return
            if velocities.shape != (6,) or not np.all(np.isfinite(velocities)):
                self.trip(f'malformed arm command {list(message.data)}')
                return
            if np.max(np.abs(velocities)) > self.max_joint_speed:
                self.trip(f'arm command {np.round(velocities, 3).tolist()} exceeds '
                          f'{self.max_joint_speed} rad/s')
                return
            now = time.monotonic()
            if now - self.last_command > 0.5:
                self.streaming_since = now
            self.last_command = now
            self.commands_this_period += 1
            if not np.any(velocities):
                if self.state == 'moving':
                    self.request_stop('zero command')
                return
            if not self.enable_commands:
                self.last_vector = velocities
                self.send_velocities(velocities)
                return
            if foreign:
                self.trip(f'other nodes publish {COMMAND_TOPIC}: {foreign}')
                return
            if self.state == 'stopping':
                self.get_logger().warn('ignoring arm command until the stop is verified',
                                       throttle_duration_sec=2.0)
                return
            refusal = self.not_ready()
            if refusal:
                self.get_logger().warn(f'ignoring arm command: {refusal}',
                                       throttle_duration_sec=2.0)
                return
            if self.state == 'idle':
                self.state, self.moving_since = 'moving', now
                self.low_follow_since = None
            self.last_vector = velocities
            if not self.send_velocities(velocities):
                self.trip('arm command could not be delivered')

    def on_reset(self, request, response):
        with self.lock:
            previous = self.fault
            self.fault = None
            message = f'UR bridge fault cleared ({previous})'
            if self.enable_commands:
                self.verified_once = False
                self.request_stop('fault reset')
                message += '; verifying the arm is still'
        if self.enable_commands and self.stream in PERSISTENT and self.stream_state not in READY:
            self.start_stream()
            message += '; streaming program restarted, watch the log for "ready for commands"'
        response.success, response.message = True, message
        self.get_logger().info(message)
        return response

    def close_command_socket_locked(self):
        if self.command_sock is not None:
            try:
                self.command_sock.close()
            except OSError:
                pass
            self.command_sock = None

    def shutdown(self):
        """Stop, verified if possible; end the streaming program; release sockets."""
        if self.closed:
            return
        self.closed = True
        try:
            if self.enable_commands:
                self.request_stop('bridge shutting down')
                # At least a second, even when a stop already overran its deadline.
                wait = min(max(self.stop_deadline - time.monotonic(), 1.0), 3.0)
                deadline = time.monotonic() + wait
                while time.monotonic() < deadline and self.state != 'idle':
                    time.sleep(0.02)
                if self.state != 'idle':
                    self.get_logger().error('arm stop NOT verified at exit: use the pendant stop')
                if self.stream in PERSISTENT:
                    # Replace the streaming program with a stop, ending it.
                    self.upload(wire.stopj_command(self.stop_deceleration))
        finally:
            self.running = False
            self.close_stream()
            with self.send_lock:
                self.close_command_socket_locked()
            # Let the threads finish before the node (and its clock) goes away.
            for thread in self.threads:
                thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = URBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A signal can shut ROS down mid-spin; only real errors propagate.
        if rclpy.ok():
            raise
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
