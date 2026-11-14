"""Run the executor with a simulated arm (no ROS), for testing on a laptop.

  python3 -m scate_hw                      # from examples/example_scate_ur10e/hardware
  python3 -m scate_hw --start-offset 0.1   # arm starts away from the start pose

It serves the control API on 127.0.0.1:8765; the SCATE UR10e Base app's dev
server talks to it (and starts it itself if nothing answers there). On the
lab PC the ROS 2 node (ros2/scate_ur10e_hw) runs the same executor against
ur_bridge instead.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time

from . import kinematics
from .executor import Executor
from .robots import SimRobot
from .safety import Settings
from .server import make_server, serve_in_thread

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))


def default_binary():
    for candidate in (os.environ.get('SCATE_HARDWARE_BIN'),
                      os.path.join(REPO, 'build', 'examples', 'example_scate_ur10e', 'scate_hardware')):
        if candidate and os.path.isfile(candidate):
            return candidate
    return os.path.join(REPO, 'build', 'examples', 'example_scate_ur10e', 'scate_hardware')


def default_description():
    return os.path.join(REPO, 'apps', 'scate', 'ur10e-reactive', 'public', 'robots', 'ur10e.json')


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python3 -m scate_hw', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--scate-bin', default=default_binary(), help='scate_hardware executable')
    parser.add_argument('--description', default=default_description(), help='robots/ur10e.json')
    parser.add_argument('--start-offset', type=float, default=0.0,
                        help='rad added to every joint of the simulated arm at power-on')
    parser.add_argument('--lag', type=float, default=0.05, help='s, simulated arm response time')
    parser.add_argument('--noise', type=float, default=0.0, help='rad, simulated encoder noise')
    parser.add_argument('--kp', type=float, default=Settings.kp)
    parser.add_argument('--rate-hz', type=float, default=Settings.rate_hz)
    args = parser.parse_args(argv)

    description = kinematics.load_description(args.description)
    q0 = [x + args.start_offset for x in description['defaults']['q_start']]
    robot = SimRobot(q0, lag=args.lag, noise=args.noise)
    settings = Settings(kp=args.kp, rate_hz=args.rate_hz)
    executor = Executor(robot, args.scate_bin, description, settings)
    server = make_server(executor, args.host, args.port)
    serve_in_thread(server)
    print(f'[scate_hw] simulated arm; control API on http://{args.host}:{args.port} '
          f'(planner {args.scate_bin})', flush=True)

    def shutdown(*_):
        executor.close()
        robot.close()
        server.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    while True:
        time.sleep(1.0)


if __name__ == '__main__':
    main()
