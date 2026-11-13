// The live-simulation endpoint: request validation and the job lifecycle, with a fake scate_live.
import test from 'node:test';
import assert from 'node:assert/strict';
import { EventEmitter } from 'node:events';
import { readFileSync, writeFileSync } from 'node:fs';
import { createLiveJobs, liveRequestText } from '../../shared/liveServer.mjs';

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
