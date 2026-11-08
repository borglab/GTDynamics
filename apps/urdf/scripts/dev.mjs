/** npm run dev: the Python backend plus this app's Vite server (--server-only: backend alone). */
import { spawn } from 'node:child_process';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { startBackend } from './backend.mjs';

const require = createRequire(import.meta.url);
const cwd = fileURLToPath(new URL('../', import.meta.url));
const children = [startBackend()].filter(Boolean);
if (!process.argv.includes('--server-only')) {
  const vite = resolve(dirname(require.resolve('vite/package.json')), 'bin/vite.js');
  children.push(spawn(process.execPath, [vite, '--host', '127.0.0.1', '--port', '3010'], { cwd, stdio: 'inherit' }));
}
if (!children.length) process.exitCode = 1;
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
