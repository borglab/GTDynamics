import loadMujoco from 'mujoco-js';
import { BrowserRobot } from './browser-ik.js';

let runtime;
let robot;

self.addEventListener('message', async ({ data: { id, path, body } }) => {
  try {
    let result;
    if (path === 'api/load') {
      const mujoco = await (runtime ??= loadMujoco());
      const next = BrowserRobot.load(mujoco, body.model, body.urdf);
      robot?.dispose();
      robot = next;
      result = robot.describe();
    } else {
      if (robot?.name !== body.model) throw new Error('The selected robot is still loading.');
      result = path === 'api/fk' ? robot.forward(body.ee, body.q) : robot.solve(body);
    }
    self.postMessage({ id, result });
  } catch (error) {
    self.postMessage({ id, error: error.message || String(error) });
  }
});
