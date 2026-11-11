/**
 * Development-server endpoint of the SCATE live simulation: runs the native
 * `scate_live solve` (examples/example_scate_ur10e) on a start, goal and
 * obstacles chosen in the page, one job at a time, and serves the replay it
 * writes. Static builds have no endpoint, so the page falls back to its
 * recorded experiment.
 *
 *   GET  <base>api/scate-live/status         {available, binary, hint}
 *   POST <base>api/scate-live/solve          request JSON -> {id}
 *   GET  <base>api/scate-live/jobs/<id>      {state, done, total, label, error, elapsed}
 *   GET  <base>api/scate-live/jobs/<id>/replay   the replay JSON
 */
import { spawn } from 'node:child_process';
import { existsSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROBOTS = ['ur5', 'ur10e'];
const MAX_OBSTACLES = 4;
const MAX_RUNS = 5;
const repoRoot = fileURLToPath(new URL('../../../', import.meta.url));

/** The scate_live binary: $SCATE_LIVE_BIN, else <repo>/build/examples/example_scate_ur10e/scate_live. */
export function findScateLive(env = process.env) {
  const candidates = [env.SCATE_LIVE_BIN, resolve(repoRoot, 'build/examples/example_scate_ur10e/scate_live')];
  return candidates.find(path => path && existsSync(path)) || null;
}

const finite = value => typeof value === 'number' && Number.isFinite(value);
function numbers(value, length, what) {
  if (!Array.isArray(value) || value.length !== length || !value.every(finite)) {
    throw new Error(`${what} needs ${length} numbers`);
  }
  return value;
}

/**
 * The request text scate_live reads (live_scenario.h parseLiveRequest), from
 * the page's request: {robot, q_start, q_goal, obstacles: [{start, velocity,
 * radius}], sigmas, seed}. Throws with a message on anything malformed.
 */
export function liveRequestText(request) {
  if (!request || typeof request !== 'object') throw new Error('The request must be a JSON object');
  if (!ROBOTS.includes(request.robot)) throw new Error(`robot must be one of ${ROBOTS.join(', ')}`);
  const lines = [`robot ${request.robot}`,
    `q_start ${numbers(request.q_start, 6, 'q_start').join(' ')}`,
    `q_goal ${numbers(request.q_goal, 6, 'q_goal').join(' ')}`];
  if (!Array.isArray(request.obstacles) || request.obstacles.length < 1 || request.obstacles.length > MAX_OBSTACLES) {
    throw new Error(`Give 1 to ${MAX_OBSTACLES} obstacles`);
  }
  request.obstacles.forEach((obstacle, index) => {
    const start = numbers(obstacle?.start, 3, `obstacle ${index + 1} start`);
    const velocity = numbers(obstacle?.velocity, 3, `obstacle ${index + 1} velocity`);
    if (!finite(obstacle.radius) || obstacle.radius <= 0 || obstacle.radius > 0.5) {
      throw new Error(`obstacle ${index + 1} radius must be in (0, 0.5] m`);
    }
    lines.push(`obstacle ${[...start, ...velocity, obstacle.radius].join(' ')}`);
  });
  const sigmas = request.sigmas ?? [0, 3, 0.3, 0.1];
  if (!Array.isArray(sigmas) || !sigmas.length || sigmas.length > MAX_RUNS || !sigmas.every(s => finite(s) && s >= 0)) {
    throw new Error(`sigmas must be 1 to ${MAX_RUNS} numbers >= 0`);
  }
  lines.push(`sigmas ${sigmas.join(' ')}`);
  const seed = request.seed ?? 0;
  if (!Number.isInteger(seed) || seed < 0) throw new Error('seed must be a whole number >= 0');
  lines.push(`seed ${seed}`);
  return lines.join('\n') + '\n';
}

/** The solve jobs: one runs at a time; starting another stops it. */
export function createLiveJobs({ binary = findScateLive(), spawnProcess = spawn } = {}) {
  const jobs = new Map();
  let next = 1, current = null;

  function start(request) {
    if (!binary) throw new Error('scate_live is not built');
    const text = liveRequestText(request);
    if (current?.state === 'running') { current.child.kill('SIGTERM'); current.state = 'failed'; current.error = 'Replaced by a newer solve'; }
    const folder = mkdtempSync(join(tmpdir(), 'scate-live-'));
    writeFileSync(join(folder, 'request.txt'), text);
    const job = { id: String(next++), state: 'running', done: 0, total: (request.sigmas ?? [0, 3, 0.3, 0.1]).length,
      label: 'starting', error: '', started: Date.now(), elapsed: 0, folder, log: '' };
    job.child = spawnProcess(binary, ['solve', join(folder, 'request.txt'), join(folder, 'replay.json')], { cwd: folder });
    let pending = '';
    job.child.stdout.on('data', chunk => {
      pending += chunk;
      const lines = pending.split('\n');
      pending = lines.pop();
      for (const line of lines) {
        const match = /^PROGRESS (\d+) (\d+) (.*)$/.exec(line);
        if (match) { job.done = Number(match[1]); job.total = Number(match[2]); job.label = match[3]; }
      }
    });
    job.child.stderr.on('data', chunk => { job.log = (job.log + chunk).slice(-2000); });
    job.child.on('error', error => { job.state = 'failed'; job.error = error.message; });
    job.child.on('close', code => {
      job.elapsed = (Date.now() - job.started) / 1000;
      if (job.state !== 'running') return;
      if (code === 0 && existsSync(join(folder, 'replay.json'))) job.state = 'done';
      else { job.state = 'failed'; job.error = job.log.trim().split('\n').pop() || `scate_live exited with code ${code}`; }
    });
    jobs.set(job.id, job);
    current = job;
    // Keep the last few jobs only.
    for (const [id, old] of jobs) {
      if (jobs.size <= 4) break;
      if (old !== current) { rmSync(old.folder, { recursive: true, force: true }); jobs.delete(id); }
    }
    return job;
  }

  function status(id) {
    const job = jobs.get(id);
    if (!job) return null;
    const elapsed = job.state === 'running' ? (Date.now() - job.started) / 1000 : job.elapsed;
    return { id: job.id, state: job.state, done: job.done, total: job.total, label: job.label, error: job.error, elapsed };
  }

  function replay(id) {
    const job = jobs.get(id);
    return job?.state === 'done' ? readFileSync(join(job.folder, 'replay.json')) : null;
  }

  function dispose() {
    for (const job of jobs.values()) {
      if (job.state === 'running') job.child.kill('SIGTERM');
      rmSync(job.folder, { recursive: true, force: true });
    }
    jobs.clear();
  }
  return { start, status, replay, dispose, binary };
}

function send(response, code, body, type = 'application/json') {
  response.statusCode = code;
  response.setHeader('Content-Type', type);
  response.setHeader('Cache-Control', 'no-store');
  response.end(typeof body === 'string' || Buffer.isBuffer(body) ? body : JSON.stringify(body));
}

function readBody(request) {
  return new Promise((resolveBody, reject) => {
    let body = '';
    request.on('data', chunk => { body += chunk; if (body.length > 100000) reject(new Error('Request too large')); });
    request.on('end', () => resolveBody(body));
    request.on('error', reject);
  });
}

/** Connect-style middleware of the endpoint, for Vite's dev and preview servers. */
export function liveMiddleware(jobs) {
  return async (request, response, next) => {
    const path = (request.url || '').split('?')[0];
    const at = path.indexOf('/api/scate-live/');
    if (at < 0) return next();
    const route = path.slice(at + '/api/scate-live/'.length);
    try {
      if (route === 'status' && request.method === 'GET') {
        return send(response, 200, {
          available: !!jobs.binary, binary: jobs.binary,
          hint: jobs.binary ? '' : 'Build scate_live (cmake --build build --target scate_live) or set SCATE_LIVE_BIN, then restart npm run dev.',
        });
      }
      if (route === 'solve' && request.method === 'POST') {
        let body;
        try { body = JSON.parse(await readBody(request)); } catch { return send(response, 400, { error: 'The request is not JSON' }); }
        try { return send(response, 202, { id: jobs.start(body).id }); }
        catch (error) { return send(response, 400, { error: error.message }); }
      }
      const match = /^jobs\/(\d+)(\/replay)?$/.exec(route);
      if (match && request.method === 'GET') {
        if (match[2]) {
          const replay = jobs.replay(match[1]);
          return replay ? send(response, 200, replay) : send(response, 404, { error: 'No replay for this job' });
        }
        const status = jobs.status(match[1]);
        return status ? send(response, 200, status) : send(response, 404, { error: 'Unknown job' });
      }
      return send(response, 404, { error: 'Unknown live-simulation route' });
    } catch (error) {
      return send(response, 500, { error: error.message });
    }
  };
}

/** Vite plugin adding the endpoint to `vite` (dev) and `vite preview`. */
export default function scateLiveServer() {
  let jobs = null;
  const install = server => {
    jobs ??= createLiveJobs();
    server.middlewares.use(liveMiddleware(jobs));
    server.httpServer?.once('close', () => { jobs?.dispose(); jobs = null; });
  };
  return { name: 'scate-live-server', configureServer: install, configurePreviewServer: install };
}
