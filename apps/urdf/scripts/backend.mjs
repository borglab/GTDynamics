/** Starts the GTDynamics Python backend (server.py) for the URDF IK app. */
import { spawn, spawnSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

const app = fileURLToPath(new URL('../', import.meta.url));
const venvPython = fileURLToPath(new URL('../../../.venv/bin/python', import.meta.url));

/** GTDYNAMICS_PYTHON, else the repository's .venv, else python3. */
export const python = process.env.GTDYNAMICS_PYTHON || (existsSync(venvPython) ? venvPython : 'python3');

/**
 * Spawns scripts/server.py, or returns null (after explaining why) when the
 * interpreter cannot import gtdynamics. The page then shows its offline notice.
 */
export function startBackend({ port = process.env.URDF_BACKEND_PORT || 8767 } = {}) {
  if (spawnSync(python, ['-c', 'import gtdynamics'], { stdio: 'ignore' }).status !== 0) {
    console.log(`${python} cannot import gtdynamics; URDF IK will show its local-setup notice. `
      + 'Set GTDYNAMICS_PYTHON to use another interpreter.');
    return null;
  }
  return spawn(python, ['scripts/server.py', '--host', '127.0.0.1', '--port', String(port)],
    { cwd: app, stdio: 'inherit' });
}

/** Resolves once the backend answers, or rejects after timeoutMs. */
export async function waitForBackend({ port = process.env.URDF_BACKEND_PORT || 8767, timeoutMs = 60_000 } = {}) {
  const deadline = Date.now() + timeoutMs;
  for (;;) {
    try {
      if ((await fetch(`http://127.0.0.1:${port}/api/schemes`)).ok) return;
    } catch { /* not listening yet */ }
    if (Date.now() > deadline) throw new Error(`URDF IK backend did not answer on port ${port}`);
    await new Promise((resolve) => setTimeout(resolve, 250));
  }
}
