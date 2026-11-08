/** Model assets and worker transport for the static site. */
import { g1Path } from './g1-visual.js';

const assetRoot = '../../../models/urdfs/';
const assets = import.meta.glob([
  '../../../models/urdfs/{ur10e,ur5}/*.urdf',
  '../../../models/urdfs/{ur10e,ur5}/visual/*.dae',
  '../../../models/urdfs/g1/g1.urdf',
], { eager: true, query: '?url&no-inline', import: 'default' });

const models = ['ur10e/ur10e.urdf', 'ur5/ur5.urdf', g1Path]
  .filter(path => assets[assetRoot + path]).map(path => ({ path, status: 'ok' }));
const schemes = [{
  id: 'dls',
  label: 'Damped least squares',
  description: 'MuJoCo supplies the tool pose and geometric Jacobian. The browser worker adapts the damping and clamps joint steps to the URDF limits.',
  default_iters: 60,
}];

// Keep the link and joint definitions, including fixed tool frames. The shared
// Three.js viewer loads the original visual geometry separately.
function kinematicsUrdf(source) {
  const document = new DOMParser().parseFromString(source, 'application/xml');
  if (document.documentElement.localName !== 'robot') throw new Error('Invalid robot description.');
  document.querySelectorAll('visual, collision').forEach(element => element.remove());
  const mujoco = document.createElement('mujoco');
  const compiler = document.createElement('compiler');
  compiler.setAttribute('fusestatic', 'false');
  compiler.setAttribute('discardvisual', 'true');
  mujoco.append(compiler);
  document.documentElement.prepend(mujoco);
  return new XMLSerializer().serializeToString(document);
}

export function createBrowserBackend() {
  const worker = new Worker(new URL('./browser-worker.js', import.meta.url), { type: 'module' });
  const pending = new Map();
  let nextId = 0;
  let failure;

  function fail(error) {
    failure = error;
    for (const finish of pending.values()) finish(error);
    worker.terminate();
  }

  worker.addEventListener('error', event => fail(new Error(event.message || 'Browser solver failed to start.')));
  worker.addEventListener('message', ({ data: { id, error, result } }) => {
    pending.get(id)?.(error ? new Error(error) : null, result);
  });

  function call(path, body, signal) {
    signal.throwIfAborted();
    if (failure) throw failure;
    return new Promise((resolve, reject) => {
      const id = nextId++;
      const abort = () => finish(signal.reason);
      const finish = (error, result) => {
        signal.removeEventListener('abort', abort);
        pending.delete(id);
        if (error) reject(error);
        else resolve(result);
      };
      signal.addEventListener('abort', abort, { once: true });
      pending.set(id, finish);
      worker.postMessage({ id, path, body });
    });
  }

  function urdfUrl(model) { return assets[assetRoot + model]; }

  return {
    async request(path, body, signal) {
      if (path === 'api/models') return { models };
      if (path === 'api/schemes') return schemes;
      if (path === 'api/load') {
        const response = await fetch(urdfUrl(body.model), { signal });
        if (!response.ok) throw new Error(`Robot description returned HTTP ${response.status}.`);
        body = { ...body, urdf: kinematicsUrdf(await response.text()) };
      }
      return call(path, body, signal);
    },
    urdfUrl,
    resolveAsset(model, url) {
      if (!url.startsWith('./visual/')) return url;
      const directory = model.slice(0, model.lastIndexOf('/') + 1);
      return assets[assetRoot + directory + url.slice(2)] ?? url;
    },
    dispose() { fail(new DOMException('Browser solver closed.', 'AbortError')); },
  };
}
