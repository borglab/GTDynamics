import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';
import { dirname, resolve } from 'node:path';

const require = createRequire(import.meta.url);

const cwd = fileURLToPath(new URL('../', import.meta.url));
const children = [
  spawn(process.execPath, ['server/index.mjs'], { cwd, stdio: 'inherit' }),
  spawn(process.execPath, [resolve(dirname(require.resolve('vite/package.json')), 'bin/vite.js'), '--host', '127.0.0.1'], { cwd, stdio: 'inherit' }),
];
let stopping = false;
function stop(code = 0) {
  if (stopping) return;
  stopping = true;
  process.exitCode = code;
  for (const child of children) child.kill('SIGTERM');
}
for (const child of children) {
  child.on('error', error => { console.error(error.message); stop(1); });
  child.on('exit', code => stop(code || 0));
}
process.on('SIGINT', () => stop());
process.on('SIGTERM', () => stop());
