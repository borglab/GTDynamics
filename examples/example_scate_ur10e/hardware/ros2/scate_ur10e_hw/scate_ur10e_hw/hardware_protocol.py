"""Wire formats for the UR10e and Parker rail, as the Simulink rig used them.

ROS-free so it can be tested without sockets or hardware. The formats are the
ones BaseFrameDetermination.slx sends and parses:

  UR10e  192.168.7.8:30003   realtime state packets in; URScript text out
         (speedj streamed at 20 Hz, one script line per command).
  Parker 192.168.7.6:5002    ASCII jog commands out (JOG VEL + JOG FWD/REV)
         192.168.7.6:5003    UDP fast-status feedback in (FSTAT0 = position
                             counts, FSTAT1 = current jog velocity).

Units at the boundary are converted here once. Inside ROS the rail is metres
along the URDF linear_rail_joint and the arm is radians, as everywhere else in
this package.
"""

import math
import re
import struct

import numpy as np

# --- UR10e realtime interface (port 30003) ---------------------------------

UR_REALTIME_PORT = 30003
# Dashboard server: one text command per line, one reply line. Used to ask
# whether the robot is in Remote Control and, as a last resort, to stop the
# running program.
UR_DASHBOARD_PORT = 29999
UR_RUNNING = 7                  # robot mode
UR_SAFETY_NORMAL = 1            # safety mode
UR_PLAYING = 2                  # program state
# Offsets into the packet's doubles, after the 4-byte length prefix. They are
# the 1-based Simulink selectors [1], [32:37] and [38:43], shifted to 0-based.
UR_TIME_INDEX = 0
UR_QD_TARGET = slice(7, 13)
UR_Q_ACTUAL = slice(31, 37)
UR_QD_ACTUAL = slice(37, 43)
# Scalars from the UR client-interface realtime layout. The rig's 1116-byte
# packet is that layout exactly (139 doubles). Not read by the Simulink model.
UR_ROBOT_MODE_INDEX = 94
UR_SAFETY_MODE_INDEX = 101
# Speed scaling: the fraction of programmed speed the controller is applying,
# i.e. the speed slider combined with reduced mode or any safety limit.
UR_SPEED_SCALING_INDEX = 117
UR_PROGRAM_STATE_INDEX = 131
UR_MIN_DOUBLES = 43

UR_ROBOT_MODES = {-1: 'NO_CONTROLLER', 0: 'DISCONNECTED', 1: 'CONFIRM_SAFETY', 2: 'BOOTING',
                  3: 'POWER_OFF', 4: 'POWER_ON', 5: 'IDLE', 6: 'BACKDRIVE', 7: 'RUNNING',
                  8: 'UPDATING_FIRMWARE'}
UR_SAFETY_MODES = {1: 'NORMAL', 2: 'REDUCED', 3: 'PROTECTIVE_STOP', 4: 'RECOVERY',
                   5: 'SAFEGUARD_STOP', 6: 'SYSTEM_EMERGENCY_STOP', 7: 'ROBOT_EMERGENCY_STOP',
                   8: 'VIOLATION', 9: 'FAULT', 10: 'VALIDATE_JOINT_ID', 11: 'UNDEFINED',
                   12: 'AUTOMATIC_MODE_SAFEGUARD_STOP', 13: 'SYSTEM_THREE_POSITION_ENABLING_STOP'}
UR_PROGRAM_STATES = {0: 'STOPPING', 1: 'STOPPED', 2: 'PLAYING', 3: 'PAUSING', 4: 'PAUSED',
                     5: 'RESUMING'}


def ur_mode_name(table, value):
    """'NAME (n)' for a mode value, or 'unknown (n)'; None stays None."""
    if value is None:
        return None
    number = int(round(value))
    return f'{table.get(number, "unknown")} ({number})'


UR_ARM_JOINTS = 6


def ur_packet_length(header):
    """Total packet length announced by the first four bytes."""
    if len(header) != 4:
        raise ValueError('UR packet header must be 4 bytes')
    length = struct.unpack('>I', header)[0]
    if length < 4 + 8 * UR_MIN_DOUBLES or length > 1 << 16:
        raise ValueError(f'implausible UR realtime packet length {length}')
    return length


def parse_ur_realtime(packet):
    """Controller time, actual joint positions and velocities from one packet.

    The packet is big-endian: a uint32 total length, then doubles. Its length
    differs between controller versions (1116 bytes on the Simulink rig), but
    the fields used here sit at the same offsets in all of them.
    """
    length = ur_packet_length(packet[:4])
    if len(packet) != length:
        raise ValueError(f'UR packet is {len(packet)} bytes, header says {length}')
    count = (length - 4) // 8
    values = struct.unpack_from(f'>{count}d', packet, 4)
    q = np.asarray(values[UR_Q_ACTUAL], dtype=float)
    qd = np.asarray(values[UR_QD_ACTUAL], dtype=float)
    if not (np.all(np.isfinite(q)) and np.all(np.isfinite(qd))):
        raise ValueError('UR packet has non-finite joint state')

    def scalar(index):
        return float(values[index]) if count > index else None

    return {'time': float(values[UR_TIME_INDEX]), 'q': q, 'qd': qd,
            'qd_target': np.asarray(values[UR_QD_TARGET], dtype=float),
            'speed_scaling': scalar(UR_SPEED_SCALING_INDEX),
            'robot_mode': scalar(UR_ROBOT_MODE_INDEX),
            'safety_mode': scalar(UR_SAFETY_MODE_INDEX),
            'program_state': scalar(UR_PROGRAM_STATE_INDEX),
            'length': length}


def _ur_vector(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (UR_ARM_JOINTS,) or not np.all(np.isfinite(values)):
        raise ValueError('UR command needs six finite joint values')
    return '[' + ','.join(f'{value:.6f}' for value in values) + ']'


def speedj_command(velocities, acceleration, duration):
    """One URScript speedj line.

    duration is how long the controller keeps this velocity if no newer
    command replaces it, so it doubles as the stream's dead-man timeout.
    """
    if not (math.isfinite(acceleration) and acceleration > 0):
        raise ValueError('speedj acceleration must be positive and finite')
    if not (math.isfinite(duration) and duration > 0):
        raise ValueError('speedj duration must be positive and finite')
    return (f'speedj({_ur_vector(velocities)},{acceleration:.4f},'
            f'{duration:.4f})\n').encode('ascii')


# One persistent program instead of one program per command. Sending a new
# script to 30003 replaces the running program, so streaming a speedj line
# per tick restarts the controller's program 20 times a second. This program
# instead connects back to the PC once, reads velocity vectors in a thread,
# and runs speedj in a tight loop on the robot. It has its own dead-man: the
# commanded velocity drops to zero when no vector has arrived for
# stale_cycles loop periods, even if the PC or network fails. A def...end
# block sent to 30003 is compiled and run by the controller as a program.
UR_STREAM_PROGRAM = '''def ros_speedj_stream():
  global qd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  global age = 1000000
  textmsg("ros_speedj_stream: connecting to {host}:{port}")
  if not socket_open("{host}", {port}, "ros_stream"):
    textmsg("ros_speedj_stream: cannot connect to {host}:{port}; stopping")
    halt
  end
  textmsg("ros_speedj_stream: connected; speedj every {cycle} s, zero after {stale} cycles")
  thread reader():
    while True:
      msg = socket_read_ascii_float(6, "ros_stream", {read_timeout})
      if msg[0] == 6:
        qd = [msg[1], msg[2], msg[3], msg[4], msg[5], msg[6]]
        age = 0
      end
    end
  end
  thrd = run reader()
  while True:
    if age > {stale}:
      speedj([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], {deceleration}, {cycle})
    else:
      speedj(qd, {acceleration}, {cycle})
    end
    age = age + 1
  end
end
'''


def stream_program(host, port, acceleration, deceleration, command_timeout, cycle=0.008):
    """The persistent streaming program, ready to send to port 30003."""
    for name, value in (('acceleration', acceleration), ('deceleration', deceleration),
                        ('command_timeout', command_timeout), ('cycle', cycle)):
        if not (math.isfinite(value) and value > 0):
            raise ValueError(f'stream {name} must be positive and finite')
    if not re.fullmatch(r'\d{1,3}(\.\d{1,3}){3}', host) or not 0 < int(port) < 65536:
        raise ValueError(f'stream endpoint {host}:{port} is not an IPv4 address and port')
    stale = max(1, int(math.ceil(command_timeout / cycle)))
    return UR_STREAM_PROGRAM.format(
        host=host, port=int(port), cycle=f'{cycle:.4f}', stale=stale,
        read_timeout=f'{min(command_timeout, 0.1):.3f}',
        acceleration=f'{acceleration:.4f}', deceleration=f'{deceleration:.4f}',
    ).encode('ascii')


def stream_vector(velocities):
    """One velocity vector for the streaming program: "(v1,...,v6)" and a newline."""
    return ('(' + _ur_vector(velocities)[1:-1] + ')\n').encode('ascii')


# --- RTDE (port 30004): the same persistent program, fed through registers ----
#
# Every connection goes from the PC to the robot: the PC writes six velocity
# registers and a sequence counter over RTDE, and the program reads them. No
# inbound connection to the PC is needed, so no firewall change either. The
# upper register range (24..47) is reserved for RTDE, clear of fieldbuses.

UR_RTDE_PORT = 30004
RTDE_PROTOCOL_VERSION = 2
RTDE_REQUEST_PROTOCOL_VERSION = 86      # 'V'
RTDE_TEXT_MESSAGE = 77                  # 'M'
RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79  # 'O'
RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73  # 'I'
RTDE_CONTROL_PACKAGE_START = 83         # 'S'
RTDE_CONTROL_PACKAGE_PAUSE = 80         # 'P'
RTDE_DATA_PACKAGE = 85                  # 'U'
# Register sets, preferred first: the upper range (24..47) only exists on
# newer controller software, which the rig's 1116-byte realtime packet
# suggests it may not have; the lower range (0..23) exists on all e-Series.
RTDE_REGISTER_BASES = (24, 18)


def rtde_input_names(base):
    """Six velocity registers from `base`, then the sequence counter at `base`."""
    return tuple(f'input_double_register_{base + i}' for i in range(6)) + (
        f'input_int_register_{base}',)


RTDE_INPUTS = rtde_input_names(RTDE_REGISTER_BASES[0])


def rtde_packet(kind, payload=b''):
    """RTDE framing: uint16 total size, uint8 type, payload."""
    return struct.pack('>HB', 3 + len(payload), kind) + payload


def rtde_read(sock):
    """One RTDE packet as (type, payload)."""
    header = b''
    while len(header) < 3:
        chunk = sock.recv(3 - len(header))
        if not chunk:
            raise ConnectionError('RTDE connection closed')
        header += chunk
    size, kind = struct.unpack('>HB', header)
    payload = b''
    while len(payload) < size - 3:
        chunk = sock.recv(size - 3 - len(payload))
        if not chunk:
            raise ConnectionError('RTDE connection closed')
        payload += chunk
    return kind, payload


def rtde_text(payload):
    """A text message from the controller, as readable text."""
    if not payload:
        return ''
    length = payload[0]
    return payload[1:1 + length].decode('ascii', 'replace') + ' ' + \
        payload[1 + length:].decode('ascii', 'replace').strip('\x00')


class RTDEInputsRefused(ConnectionError):
    """The controller refused the input recipe; `types` says why per field.

    IN_USE: another RTDE client owns the register. NOT_FOUND: this
    controller has no such register. Callers decide on `types`, never on the
    message text, which names both.
    """

    def __init__(self, types):
        self.types = list(types)
        super().__init__(f'RTDE input setup refused: {self.types} (IN_USE means another '
                         'RTDE client owns these registers; NOT_FOUND means an '
                         'unsupported controller version)')

    @property
    def in_use(self):
        return 'IN_USE' in self.types

    @property
    def not_found(self):
        return 'NOT_FOUND' in self.types


def rtde_setup_inputs(sock, on_message=None, output_hz=10.0, base=RTDE_REGISTER_BASES[0]):
    """Negotiate the protocol and the input recipe, start; return the recipe id.

    The controller only starts synchronizing once an output recipe exists
    too (seen on PolyScope 5.26), so a low-rate timestamp output is set up;
    the caller must keep reading, and may discard, those packets. Raises
    with the controller's own reason, e.g. registers owned by another RTDE
    client (IN_USE). Text messages sent meanwhile go to on_message.
    """
    sock.sendall(rtde_packet(RTDE_REQUEST_PROTOCOL_VERSION,
                             struct.pack('>H', RTDE_PROTOCOL_VERSION)))
    kind, payload = _rtde_reply(sock, RTDE_REQUEST_PROTOCOL_VERSION, on_message)
    if not payload or not payload[0]:
        raise ConnectionError(f'controller refused RTDE protocol {RTDE_PROTOCOL_VERSION}')
    sock.sendall(rtde_packet(RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS,
                             struct.pack('>d', output_hz) + b'timestamp'))
    kind, payload = _rtde_reply(sock, RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, on_message)
    if not payload or payload[0] == 0:
        raise ConnectionError('RTDE output setup refused')
    sock.sendall(rtde_packet(RTDE_CONTROL_PACKAGE_SETUP_INPUTS,
                             ','.join(rtde_input_names(base)).encode('ascii')))
    kind, payload = _rtde_reply(sock, RTDE_CONTROL_PACKAGE_SETUP_INPUTS, on_message)
    recipe, types = payload[0], payload[1:].decode('ascii', 'replace').split(',')
    expected = ['DOUBLE'] * 6 + ['INT32']
    if recipe == 0 or types != expected:
        raise RTDEInputsRefused(types)
    sock.sendall(rtde_packet(RTDE_CONTROL_PACKAGE_START))
    kind, payload = _rtde_reply(sock, RTDE_CONTROL_PACKAGE_START, on_message)
    if not payload or not payload[0]:
        raise ConnectionError('controller refused to start RTDE synchronization')
    return recipe


def _rtde_reply(sock, expected_kind, on_message=None):
    """The reply to a control request; controller text messages go to on_message."""
    while True:
        kind, payload = rtde_read(sock)
        if kind == expected_kind:
            return kind, payload
        if kind == RTDE_TEXT_MESSAGE and on_message is not None:
            on_message(rtde_text(payload))


def rtde_inputs(recipe, velocities, sequence):
    """One data package: six velocity registers and the sequence counter."""
    velocities = np.asarray(velocities, dtype=float)
    if velocities.shape != (UR_ARM_JOINTS,) or not np.all(np.isfinite(velocities)):
        raise ValueError('RTDE command needs six finite joint values')
    return rtde_packet(RTDE_DATA_PACKAGE, struct.pack(
        '>B6di', recipe, *velocities, int(sequence) & 0x7FFFFFFF))


UR_RTDE_PROGRAM = '''def ros_speedj_rtde():
  global last_seq = read_input_integer_register({seq})
  global age = 1000000
  textmsg("{banner}")
  while True:
    seq = read_input_integer_register({seq})
    if seq != last_seq:
      last_seq = seq
      age = 0
    end
    if age > {stale}:
      speedj([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], {deceleration}, {cycle})
    else:
      qd = [{registers}]
      speedj(qd, {acceleration}, {cycle})
    end
    age = age + 1
  end
end
'''


def rtde_program(acceleration, deceleration, command_timeout, cycle=0.008,
                 base=RTDE_REGISTER_BASES[0]):
    """The register-reading streaming program, ready to send to port 30003."""
    for name, value in (('acceleration', acceleration), ('deceleration', deceleration),
                        ('command_timeout', command_timeout), ('cycle', cycle)):
        if not (math.isfinite(value) and value > 0):
            raise ValueError(f'stream {name} must be positive and finite')
    reg = base
    stale = max(1, int(math.ceil(command_timeout / cycle)))
    banner = (f'ros_speedj_rtde: running; speedj every {cycle:.4f} s from '
              f'input_double_register_{reg}..{reg + 5}, zero after {stale} cycles without '
              'a new sequence')
    registers = ', '.join(f'read_input_float_register({reg + i})' for i in range(6))
    return UR_RTDE_PROGRAM.format(
        seq=base, banner=banner, registers=registers,
        cycle=f'{cycle:.4f}', stale=stale,
        acceleration=f'{acceleration:.4f}', deceleration=f'{deceleration:.4f}',
    ).encode('ascii')


def stopj_command(deceleration):
    if not (math.isfinite(deceleration) and deceleration > 0):
        raise ValueError('stopj deceleration must be positive and finite')
    return f'stopj({deceleration:.4f})\n'.encode('ascii')


# --- Parker IPA rail ---------------------------------------------------------

RAIL_COMMAND_PORT = 5002
RAIL_FEEDBACK_PORT = 5003
RAIL_SUBSCRIBE = bytes.fromhex('000000010000000a')   # enable, 10 ms interval
RAIL_UNSUBSCRIBE = bytes(8)
RAIL_STOP = b'AXIS0 JOG OFF'
# Legacy encoder scale used by the Simulink model (Gain 1/26214.4/1000).
RAIL_COUNTS_PER_MM = 26214.4
# JOG VEL is formatted %05.1f, so 0.1 unit/s is the smallest nonzero speed
# the legacy command can express and 999.9 the largest.
RAIL_MIN_SPEED_UNITS = 0.05
RAIL_MAX_SPEED_UNITS = 999.9
RAIL_TERMINATORS = {'cr': b'\r', 'crlf': b'\r\n', 'lfcr': b'\n\r'}


def parse_rail_feedback(data):
    """Encoder counts and raw jog velocity from one fast-status datagram."""
    if not 320 <= len(data) <= 2560:
        raise ValueError(f'rail packet length {len(data)} outside 320..2560')
    counts = struct.unpack_from('>i', data, 0)[0]
    velocity = struct.unpack_from('>f', data, 32)[0]
    if not math.isfinite(velocity):
        raise ValueError('rail packet has a non-finite velocity')
    return counts, float(velocity)


class RailCalibration:
    """Map between controller units and the planner's rail coordinate.

    After homing, the controller's zero is repeatable, but it is not the URDF
    zero: offset_m is the planner coordinate of controller position 0, and
    sign is +1 when increasing counts (JOG FWD) move toward +x in the URDF.
    units_per_mm is the controller's user units per mm for JOG VEL; the rig's
    measurements put it near 1, but it is not yet calibrated.
    """

    def __init__(self, offset_m=0.0, sign=1.0, units_per_mm=1.0,
                 counts_per_mm=RAIL_COUNTS_PER_MM):
        if sign not in (1.0, -1.0):
            raise ValueError('rail sign must be +1 or -1')
        for name, value in (('offset_m', offset_m), ('units_per_mm', units_per_mm),
                            ('counts_per_mm', counts_per_mm)):
            if not math.isfinite(value):
                raise ValueError(f'rail {name} must be finite')
        if units_per_mm <= 0 or counts_per_mm <= 0:
            raise ValueError('rail scales must be positive')
        self.offset_m = float(offset_m)
        self.sign = float(sign)
        self.units_per_mm = float(units_per_mm)
        self.counts_per_mm = float(counts_per_mm)

    def position_m(self, counts):
        return self.offset_m + self.sign * counts / self.counts_per_mm / 1000.0

    def velocity_m_s(self, velocity_units):
        return self.sign * velocity_units / self.units_per_mm / 1000.0

    def controller_velocity(self, velocity_m_s):
        """Signed JOG VEL value, in controller units/s, for a planner velocity."""
        return self.sign * velocity_m_s * 1000.0 * self.units_per_mm


def rail_jog_command(velocity_units):
    """The legacy one-line jog: speed magnitude, then direction.

    Returns RAIL_STOP for speeds the format rounds to zero, and refuses speeds
    it cannot represent rather than truncating them.
    """
    if not math.isfinite(velocity_units):
        raise ValueError('rail velocity must be finite')
    speed = abs(velocity_units)
    if speed < RAIL_MIN_SPEED_UNITS:
        return RAIL_STOP
    if speed > RAIL_MAX_SPEED_UNITS:
        raise ValueError(f'rail speed {speed:.1f} units/s exceeds the command format')
    direction = 'FWD' if velocity_units > 0 else 'REV'
    return f'AXIS0 JOG VEL {speed:05.1f}:AXIS0 JOG {direction}'.encode('ascii')


# Home-found flag: -1 when set, 0 when clear. Read-only; it says the axis has
# been referenced since power-up, not that the carriage is at the switch now.
RAIL_HOMED_QUERY = b'PRINT BIT 16134'
# Jog-active flag: -1 while a jog (including its deceleration after JOG OFF)
# is in progress, 0 once it has ended. Read-only.
RAIL_JOG_ACTIVE_QUERY = b'PRINT BIT 792'
# Jog acceleration and deceleration, controller units/s^2. JOG OFF stops at
# the deceleration: at 0.1 (seen on this controller before) a stop from
# 35 units/s takes almost six minutes.
RAIL_JOG_ACCEL_QUERY = b'PRINT P12349'
RAIL_JOG_DECEL_QUERY = b'PRINT P12350'
RAIL_VERSION_QUERY = b'VER'
RAIL_DRIVE_ON = b'AXIS0 DRIVE ON'
RAIL_DRIVE_OFF = b'AXIS0 DRIVE OFF'
# Fast-status mapping for the UDP feedback: word 0 = actual position counts
# (P12290), word 8 = current jog velocity (P12346). Telemetry only, no motion.
RAIL_FSTAT_SETUP = (b'FSTAT0(48,2)', b'FSTAT1(48,58)', b'FSTAT ON')


def rail_home_command(velocity_units, direction=-1):
    """Set the jog speed, then home: the controller homes at that jog speed."""
    if direction not in (-1, 1):
        raise ValueError('homing direction must be -1 or +1')
    if not RAIL_MIN_SPEED_UNITS <= velocity_units <= RAIL_MAX_SPEED_UNITS:
        raise ValueError(f'homing speed {velocity_units} units/s outside the command format')
    return f'AXIS0 JOG VEL {velocity_units:05.1f}:AXIS0 JOG HOME {direction:d}'.encode('ascii')


def parse_rail_number_reply(reply, query):
    """The number a PRINT query answered, or None if it cannot be read.

    The controller echoes the command, then prints the value on its own line
    and a prompt. Only a standalone number after the echo is accepted, so a
    prompt such as P00> is never mistaken for the value.
    """
    index = reply.find(query)
    rest = reply[index + len(query):] if index >= 0 else reply
    match = re.search(r'^\s*(-?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)\s*$', rest, flags=re.MULTILINE)
    if match is None:
        return None
    value = float(match.group(1))
    return value if math.isfinite(value) else None


def parse_rail_bit_reply(reply, query=RAIL_HOMED_QUERY.decode('ascii')):
    """True/False from a PRINT BIT reply (-1 set, 0 clear), or None if unreadable."""
    value = parse_rail_number_reply(reply, query)
    return {-1.0: True, 0.0: False}.get(value)
