// Browser kinematics of the live robot descriptions against the native link frames.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { armClearance, linkTransforms, matrixPose, poseMatrix, solvePointIK, sphereCenters, transformPoint } from '../../shared/kinematics.js';

const readJson = path => JSON.parse(readFileSync(new URL(path, import.meta.url)));
const robots = { ur10e: readJson('../public/robots/ur10e.json'), ur5: readJson('../public/robots/ur5.json') };
const replay = readJson('../public/replay.json');
const near = (a, b, tolerance, what) => a.forEach((value, i) => assert(Math.abs(value - b[i]) < tolerance, `${what}[${i}]: ${value} vs ${b[i]}`));

test('descriptions name the replay links and carry a default experiment', () => {
  for (const [key, robot] of Object.entries(robots)) {
    assert.equal(robot.format, 'scate-live-robot');
    assert.equal(robot.key, key);
    assert.deepEqual(robot.links.map(link => link.name), replay.links);
    assert.equal(robot.joints.length, 6);
    assert.equal(robot.defaults.q_start.length, 6);
    assert(robot.defaults.obstacles.length >= 1);
  }
});

test('the product of exponentials reproduces the recorded UR10e link poses', () => {
  const run = replay.runs[0];
  for (const frame of [run.frames[0], run.frames[57], run.frames.at(-1)]) {
    const transforms = linkTransforms(robots.ur10e, frame.q);
    frame.links.forEach((pose, l) => {
      const expected = poseMatrix(pose);
      near(transforms[l], expected, 2e-5, `link ${l} at t=${frame.t}`);
    });
  }
});

test('poses round-trip through matrices', () => {
  const pose = [0.1, -0.2, 0.3, ...(() => { const q = [0.2, -0.5, 0.4, 0.7]; const n = Math.hypot(...q); return q.map(v => v / n); })()];
  const back = matrixPose(poseMatrix(pose));
  near(back.slice(0, 3), pose.slice(0, 3), 1e-12, 'position');
  const sign = Math.sign(back[6]) === Math.sign(pose[6]) ? 1 : -1;
  near(back.slice(3).map(v => sign * v), pose.slice(3), 1e-12, 'rotation');
});

test('point IK moves the tool to a reachable target within the limits', () => {
  for (const robot of Object.values(robots)) {
    const tool = robot.spheres[robot.tool_sphere];
    const reachable = transformPoint(linkTransforms(robot, robot.defaults.q_goal)[tool.link], tool.center);
    const q = solvePointIK(robot, robot.defaults.q_start, tool.link, tool.center, reachable, { iterations: 200 });
    const reached = transformPoint(linkTransforms(robot, q)[tool.link], tool.center);
    assert(Math.hypot(...reached.map((v, i) => v - reachable[i])) < 2e-3, `${robot.key} reaches the goal tool point`);
    q.forEach((value, j) => assert(value >= robot.joints[j].lower && value <= robot.joints[j].upper));
  }
});

test('arm clearance matches the recorded clearance at the start', () => {
  const robot = robots.ur10e, frame = replay.runs[0].frames[0];
  const obstacles = frame.obstacle_centers.map((center, j) => ({ center, radius: replay.obstacles[j].radius }));
  assert(Math.abs(armClearance(robot, frame.q, obstacles).gap - frame.clear) < 1e-4);
  assert.equal(sphereCenters(robot, frame.q).length, robot.spheres.length);
});
