"""The parts of a live-app request (live_scenario.h) the executor needs.

The planner reads the whole request; the executor only needs the start and
goal, the obstacles (to send their true centers as the readings: the
obstacle is simulated until motion capture is connected), and the walls
and floor (for its own clearance check of every plan).
"""

from __future__ import annotations


class Obstacle:

    def __init__(self, start, velocity, radius):
        self.start, self.velocity, self.radius = list(start), list(velocity), float(radius)

    def center(self, t):
        return [p + t * v for p, v in zip(self.start, self.velocity)]

    def as_dict(self):
        return {'start': self.start, 'velocity': self.velocity, 'radius': self.radius}


class Request:

    def __init__(self, text):
        self.text = text if text.endswith('\n') else text + '\n'
        self.robot = 'ur10e'
        self.q_start = self.q_goal = None
        self.obstacles, self.planes = [], []
        self.sensing = 'markers'
        for number, raw in enumerate(text.splitlines(), 1):
            line = raw.split('#', 1)[0].split()
            if not line:
                continue
            key, fields = line[0], line[1:]
            try:
                if key == 'robot':
                    self.robot = fields[0]
                elif key in ('q_start', 'q_goal'):
                    setattr(self, key, [float(x) for x in fields])
                elif key == 'obstacle':
                    x = [float(f) for f in fields]
                    if len(x) != 7:
                        raise ValueError('obstacle needs x y z vx vy vz radius')
                    self.obstacles.append(Obstacle(x[0:3], x[3:6], x[6]))
                elif key == 'plane':
                    x = [float(f) for f in fields[1:]]
                    if len(x) != 4:
                        raise ValueError('plane needs name nx ny nz offset')
                    self.planes.append({'name': fields[0], 'normal': x[0:3], 'offset': x[3]})
                elif key == 'sensing':
                    self.sensing = fields[0]
            except (IndexError, ValueError) as error:
                raise ValueError(f'request line {number}: {error}') from None
        if not self.q_start or not self.q_goal or len(self.q_start) != len(self.q_goal):
            raise ValueError('the request needs q_start and q_goal of the same length')
        if not self.obstacles:
            raise ValueError('the request needs an obstacle')
        if self.sensing != 'encoders':
            raise ValueError("a hardware request needs 'sensing encoders'")

    def obstacle_centers(self, t):
        return [obstacle.center(t) for obstacle in self.obstacles]
