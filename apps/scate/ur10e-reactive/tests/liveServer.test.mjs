// The live-simulation endpoint: request validation and the job lifecycle, with a fake scate_live.
import test from 'node:test';
import assert from 'node:assert/strict';
import { EventEmitter } from 'node:events';
import { readFileSync, writeFileSync } from 'node:fs';
import { createHardwareRelay, createLiveJobs, hardwareMiddleware, liveRequestText } from '../../shared/liveServer.mjs';

const request = {
  robot: 'ur10e', q_start: [0, -1.5708, 1.5708, -1.5708, -1.5708, 0], q_goal: [1.6, -1.2, 1.3, -1.9, -1.3, 0.4],
  obstacles: [{ start: [0.3, 1.1, 0.6], velocity: [0, -0.02, 0], radius: 0.08 }], sigmas: [0, 3], seed: 0,
};

test('a request becomes the text scate_live reads', () => {
  assert.equal(liveRequestText(request), [
    'robot ur10e', 'q_start 0 -1.5708 1.5708 -1.5708 -1.5708 0', 'q_goal 1.6 -1.2 1.3 -1.9 -1.3 0.4',
    'obstacle 0.3 1.1 0.6 0 -0.02 0 0.08', 'sigmas 0 3', 'seed 0', ''].join('\n'));
});

test('malformed requests are refused before anything runs', () => {
  for (const bad of [
    { ...request, robot: 'ur3' }, { ...request, q_start: [0, 0, 0] }, { ...request, q_goal: [0, 0, 0, 0, 0, NaN] },
    { ...request, obstacles: [] }, { ...request, obstacles: [{ ...request.obstacles[0], radius: 0 }] },
    { ...request, sigmas: [-1] }, { ...request, seed: 1.5 }, null,
  ]) assert.throws(() => liveRequestText(bad));
});

test('walls and floor become plane lines', () => {
  const planes = [{ name: 'wall', normal: [0, 0, 1], offset: -0.05 }, { name: 'floor', normal: [0, -1, 0], offset: -0.975 }];
  const text = liveRequestText({ ...request, environment: planes });
  assert.match(text, /\nplane wall 0 0 1 -0\.05\nplane floor 0 -1 0 -0\.975\nseed 0\n$/);
  assert.equal(liveRequestText({ ...request, environment: [] }), liveRequestText(request));
  for (const bad of [
    [{ ...planes[0], normal: [0, 0, 2] }], [{ ...planes[0], normal: [0, 1] }], [{ ...planes[0], offset: NaN }],
    [{ ...planes[0], name: 'a wall' }], [{ ...planes[0], name: '' }], 'wall', Array(7).fill(planes[0]),
  ]) assert.throws(() => liveRequestText({ ...request, environment: bad }));
});

// A child process stand-in that writes the replay and reports progress like scate_live.
function fakeSpawn(exitCode = 0) {
  return (binary, [command, requestPath, replayPath]) => {
    const child = new EventEmitter();
    child.stdout = new EventEmitter(); child.stderr = new EventEmitter();
    child.kill = () => child.emit('close', null);
    setTimeout(() => {
      assert.equal(command, 'solve');
      assert.match(readFileSync(requestPath, 'utf8'), /^robot ur10e/);
      child.stdout.emit('data', 'PROGRESS 1 2 σ = 3\nPRO');
      child.stdout.emit('data', 'GRESS 2 2 writing replay\n');
      if (exitCode === 0) writeFileSync(replayPath, '{"format":"scate-replay"}');
      else child.stderr.emit('data', 'scate_live: the request needs an obstacle\n');
      child.emit('close', exitCode);
    }, 5);
    return child;
  };
}
const settle = () => new Promise(resolve => setTimeout(resolve, 30));

test('a job reports progress, then serves its replay', async () => {
  const jobs = createLiveJobs({ binary: '/fake/scate_live', spawnProcess: fakeSpawn(0) });
  const job = jobs.start(request);
  assert.equal(jobs.status(job.id).state, 'running');
  await settle();
  const status = jobs.status(job.id);
  assert.deepEqual([status.state, status.done, status.total, status.label], ['done', 2, 2, 'writing replay']);
  assert.equal(JSON.parse(jobs.replay(job.id)).format, 'scate-replay');
  jobs.dispose();
});

test('a failed solve reports the solver message and no replay', async () => {
  const jobs = createLiveJobs({ binary: '/fake/scate_live', spawnProcess: fakeSpawn(1) });
  const job = jobs.start(request);
  await settle();
  assert.equal(jobs.status(job.id).state, 'failed');
  assert.match(jobs.status(job.id).error, /needs an obstacle/);
  assert.equal(jobs.replay(job.id), null);
  jobs.dispose();
});

test('without the binary nothing starts', () => {
  assert.throws(() => createLiveJobs({ binary: null }).start(request), /not built/);
});

test('hardware fields become request lines, checked', () => {
  const text = liveRequestText({ ...request, sensing: 'encoders', v_limit: 0.3, u_limit: 0.6, hardware_sigma: 3 });
  assert.match(text, /\nseed 0\nsensing encoders\nv_limit 0\.3\nu_limit 0\.6\nhardware_sigma 3\n$/);
  for (const bad of [{ sensing: 'vicon' }, { v_limit: 0 }, { v_limit: 1.5 }, { u_limit: 2.5 }, { u_limit: 'fast' },
    { hardware_sigma: 0.3 }]) {
    assert.throws(() => liveRequestText({ ...request, ...bad }));
  }
});

// A fake executor behind fetch, recording the calls.
function fakeExecutor({ down = false } = {}) {
  const calls = [];
  const fetchImpl = async (url, options = {}) => {
    calls.push({ url, method: options.method || 'GET', body: options.body });
    if (down) throw new Error('connect ECONNREFUSED');
    const path = new URL(url).pathname;
    const body = path === '/status' ? { state: 'ready', robot: { kind: 'sim', status: 'ready' } }
      : path === '/replay' ? { format: 'scate-replay' } : { ok: true };
    return { status: 200, text: async () => JSON.stringify(body) };
  };
  return { calls, fetchImpl };
}

test('the relay forwards to the executor and converts a prepare request to text', async () => {
  const { calls, fetchImpl } = fakeExecutor();
  const relay = createHardwareRelay({ env: { SCATE_HW_URL: 'http://127.0.0.1:9999' }, fetchImpl });
  const status = await relay.status();
  assert.equal(status.connected, true);
  assert.equal(status.state, 'ready');
  const middleware = hardwareMiddleware(relay);
  const call = (method, url, body) => new Promise(resolve => {
    const request = new EventEmitter();
    Object.assign(request, { method, url });
    const response = { statusCode: 0, headers: {}, setHeader(k, v) { this.headers[k] = v; },
      end(data) { resolve({ code: this.statusCode, body: JSON.parse(data) }); } };
    middleware(request, response, () => resolve({ code: 'next' }));
    setImmediate(() => { if (body !== undefined) request.emit('data', JSON.stringify(body)); request.emit('end'); });
  });
  const prepared = await call('POST', '/api/scate-hw/prepare', { ...request, sensing: 'encoders' });
  assert.deepEqual(prepared, { code: 200, body: { ok: true } });
  const sent = JSON.parse(calls.at(-1).body);
  assert.equal(calls.at(-1).url, 'http://127.0.0.1:9999/prepare');
  assert.match(sent.request, /^robot ur10e\n[\s\S]*sensing encoders\n$/);
  assert.equal((await call('POST', '/api/scate-hw/prepare', { robot: 'nope' })).code, 400);
  assert.equal((await call('POST', '/api/scate-hw/begin', {})).code, 200);
  assert.equal(calls.at(-1).url, 'http://127.0.0.1:9999/begin');
  assert.equal((await call('GET', '/api/scate-hw/replay')).body.format, 'scate-replay');
  assert.equal((await call('POST', '/api/scate-hw/explode', {})).code, 404);
  assert.equal((await call('GET', '/other')).code, 'next');
});

test('with no executor and no SCATE_HW_URL the relay starts the simulated arm', async () => {
  const { fetchImpl } = fakeExecutor({ down: true });
  const started = [];
  const spawnProcess = (command, args, options) => {
    started.push({ command, args, cwd: options.cwd, bin: options.env.SCATE_HARDWARE_BIN });
    const child = new EventEmitter();
    child.stdout = new EventEmitter(); child.stderr = new EventEmitter(); child.kill = () => {};
    return child;
  };
  writeFileSync('/tmp/fake_scate_hardware', '');
  const relay = createHardwareRelay({ env: { SCATE_HARDWARE_BIN: '/tmp/fake_scate_hardware' }, fetchImpl, spawnProcess });
  const status = await relay.status();
  assert.equal(status.connected, false);
  assert.match(status.hint, /simulated arm/);
  assert.deepEqual(started.map(s => [s.command, s.args.join(' '), s.bin]), [['python3', '-m scate_hw', '/tmp/fake_scate_hardware']]);
  assert.match(started[0].cwd, /examples\/example_scate_ur10e\/hardware$/);
  await relay.status();
  assert.equal(started.length, 1, 'started once');
  // A configured URL is never replaced by a simulator.
  const fixed = createHardwareRelay({ env: { SCATE_HW_URL: 'http://lab:8765', SCATE_HARDWARE_BIN: '/tmp/fake_scate_hardware' }, fetchImpl, spawnProcess });
  assert.match((await fixed.status()).hint, /No executor at http:\/\/lab:8765/);
  assert.equal(started.length, 1);
  relay.dispose();
});
