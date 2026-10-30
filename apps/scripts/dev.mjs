import { spawn } from 'node:child_process';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';
import { createServer } from 'vite';
import apps from '../registry.json' with { type: 'json' };

const root = fileURLToPath(new URL('../', import.meta.url));
const servers = [];
let bridge;
let stopping = false;
async function stop(code = 0) {
  if (stopping) return;
  stopping = true;
  bridge?.kill('SIGTERM');
  await Promise.all(servers.map(server => server.close()));
  process.exitCode = code;
}
process.once('SIGINT', () => stop());
process.once('SIGTERM', () => stop());

try {
  const executable = process.env.MOTION_PLANNING_BACKEND
    || resolve(root, '../build/apps/motion-planning/backend/motion_planning_optimizer');
  if (existsSync(executable)) {
    bridge = spawn(process.execPath, ['motion-planning/server/index.mjs'], {
      cwd: root, stdio: 'inherit', env: { ...process.env, MOTION_PLANNING_BACKEND: executable },
    });
    bridge.on('error', error => { console.error(error); stop(1); });
    bridge.on('exit', code => { if (!stopping) stop(code || 1); });
  } else {
    console.log('Native planner is not built; Motion Planning will show the recorded solve. See apps/README.md for live planning setup.');
  }
  for (const [index, app] of apps.entries()) {
    const server = await createServer({
      root: resolve(root, app.id), base: `/${app.id}/`,
      server: { host: '127.0.0.1', port: 3001 + index, strictPort: true },
    });
    servers.push(server);
    await server.listen();
  }
  const hub = await createServer({ root });
  servers.push(hub);
  await hub.listen();
  hub.printUrls();
} catch (error) {
  console.error(error);
  await stop(1);
}
