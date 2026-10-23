import { test } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtemp, writeFile, chmod, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { SolverSession } from '../server/session.mjs';

async function fixture(t) {
  const dir = await mkdtemp(join(tmpdir(), 'steap-session-'));
  const executable = join(dir, 'optimizer.mjs');
  await writeFile(executable, `#!/usr/bin/env node
import { createInterface } from 'node:readline';
let iteration = 0;
for await (const line of createInterface({ input: process.stdin })) {
  const request = JSON.parse(line);
  const type = { describe: 'description', build: 'built', targets: 'targets', iterate: 'iteration' }[request.command];
  if (request.command === 'iterate') await new Promise(resolve => setTimeout(resolve, 50));
  if (request.command === 'build' || request.command === 'targets') iteration = 0;
  const result = { type, request_id: request.request_id, iteration: request.command === 'iterate' ? ++iteration : 0, converged: iteration >= 3, start: request.start };
  process.stdout.write(JSON.stringify(result) + '\\n');
}
`);
  await chmod(executable, 0o755);
  const messages = [];
  const session = new SolverSession({ executable, cwd: dir, send: message => messages.push(message) });
  t.after(async () => { session.close(); await rm(dir, { recursive: true, force: true }); });
  async function until(predicate) {
    const deadline = Date.now() + 3000;
    while (!predicate()) {
      if (Date.now() > deadline) throw new Error(`Timed out: ${JSON.stringify(messages)}`);
      await new Promise(resolve => setTimeout(resolve, 10));
    }
  }
  await until(() => messages.some(m => m.type === 'description'));
  return { session, messages, until };
}

test('build is explicit; solve streams real iteration responses to convergence', async t => {
  const { session, messages, until } = await fixture(t);
  session.accept({ type: 'solve' });
  assert.match(messages.at(-1).message, /Build the graph/);
  session.accept({ type: 'build', intermediate_states: 5, duration: 4 });
  await until(() => messages.some(m => m.type === 'built'));
  session.accept({ type: 'solve', iteration_delay_ms: 0 });
  await until(() => messages.some(m => m.type === 'status' && m.message === 'Converged.'));
  assert.deepEqual(messages.filter(m => m.type === 'iteration').map(m => m.iteration), [1, 2, 3]);
});

test('retargeting coalesces edits and suppresses an obsolete in-flight iteration', async t => {
  const { session, messages, until } = await fixture(t);
  session.accept({ type: 'build', intermediate_states: 5, duration: 4 });
  await until(() => messages.some(m => m.type === 'built'));
  session.accept({ type: 'solve', iteration_delay_ms: 0 });
  await new Promise(resolve => setTimeout(resolve, 10));
  session.accept({ type: 'targets', start: 'obsolete', solve: true });
  session.accept({ type: 'targets', start: 'latest', solve: true });
  await until(() => messages.some(m => m.type === 'status' && m.message === 'Converged.'));
  assert.deepEqual(messages.filter(m => m.type === 'targets').map(m => m.start), ['latest']);
  const relevant = messages.filter(m => ['targets', 'iteration'].includes(m.type));
  assert.equal(relevant[0].type, 'targets');
  assert.deepEqual(relevant.slice(1).map(m => m.iteration), [1, 2, 3]);
});

test('stop suppresses pending iteration and queued automatic solve', async t => {
  const { session, messages, until } = await fixture(t);
  session.accept({ type: 'build', intermediate_states: 5, duration: 4 });
  await until(() => messages.some(m => m.type === 'built'));
  session.accept({ type: 'solve', iteration_delay_ms: 0 });
  session.accept({ type: 'targets', start: 'new', solve: true });
  session.accept({ type: 'stop' });
  await until(() => session.controls.length === 0 && !session.busy);
  assert.equal(messages.filter(m => m.type === 'iteration').length, 0);
  assert.equal(session.solving, false);
});
