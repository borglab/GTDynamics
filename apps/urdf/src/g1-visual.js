import { loadJsonUrl } from '../../shared/loadJson.js';
import { createRobotVisual } from '../../shared/mujocoRobot.js';
import geometryUrl from '../../contact-labels/public/model.json.gz?url&no-inline';

export const g1Path = 'g1/g1.urdf';

const descriptions = import.meta.glob('../../../models/mujoco/.cache/g1-mjlab/contact-frames.json', {
  eager: true, import: 'bodyNames',
});

/** Use the site's compiled Menagerie meshes with body poses from either solver. */
export async function loadG1Visual(signal) {
  const bodyNames = Object.values(descriptions)[0];
  if (!bodyNames) throw new Error('G1 model data is missing. Run npm run models from apps/.');
  const geometry = await loadJsonUrl(geometryUrl, signal, true);
  signal.throwIfAborted();
  if (bodyNames.length + 1 !== geometry.nbody) {
    throw new Error('G1 body names do not match the compiled display model.');
  }
  const { root, bodies } = createRobotVisual(geometry);
  const byName = new Map(bodyNames.map((name, index) => {
    const body = bodies[index + 1];
    body.userData.ikLink = name;
    return [name, body];
  }));
  return {
    root,
    update({ skeleton }) {
      for (const frame of skeleton) {
        const body = byName.get(frame.link);
        if (!body) continue;
        body.position.fromArray(frame.position);
        body.quaternion.fromArray(frame.quat_xyzw);
      }
    },
  };
}
