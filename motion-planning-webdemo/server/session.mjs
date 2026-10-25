import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';

/** One native graph per browser connection; only one C++ command in flight. */
export class SolverSession {
  constructor({ executable, cwd, send, onClose = () => {} }) {
    this.send = send;
    this.onClose = onClose;
    this.closed = false;
    this.built = false;
    this.solving = false;
    this.generation = 0;
    this.nextId = 0;
    this.controls = [];
    this.pending = new Map();
    this.child = spawn(executable, [], { cwd, stdio: ['pipe', 'pipe', 'pipe'] });
    this.child.stdin.on('error', () => {});
    this.child.stderr.on('data', data => process.stderr.write(`[optimizer] ${data}`));
    this.lines = createInterface({ input: this.child.stdout });
    this.lines.on('line', line => {
      try {
        const result = JSON.parse(line);
        const pending = this.pending.get(result.request_id);
        if (!pending) throw new Error('Optimizer returned an unknown request ID.');
        this.pending.delete(result.request_id);
        clearTimeout(pending.timer);
        pending.resolve(result);
      } catch (error) {
        this.fail(`Invalid optimizer response: ${error.message}`);
      }
    });
    this.child.once('error', error => this.fail(`Cannot start C++ optimizer: ${error.message}. Build motion_planning_optimizer first (see README).`));
    this.child.once('exit', (code, signal) => {
      if (!this.closed) this.fail(`C++ optimizer stopped (${signal || code}). Reconnect after rebuilding it.`);
    });
    this.controls.push({ command: 'describe', generation: 0 });
    this.pump();
  }

  emit(message) { if (!this.closed) this.send(message); }

  request(command) {
    return new Promise((resolve, reject) => {
      if (this.closed) return reject(new Error('Session closed.'));
      const request_id = ++this.nextId;
      const timer = setTimeout(() => this.fail('C++ optimizer timed out after 60 seconds.'), 60_000);
      this.pending.set(request_id, { resolve, reject, timer });
      this.child.stdin.write(`${JSON.stringify({ ...command, request_id })}\n`);
    });
  }

  accept(message) {
    if (this.closed) return;
    if (!message || typeof message !== 'object' || Array.isArray(message)) {
      this.emit({ type: 'error', message: 'Expected a JSON command object.' });
      return;
    }
    const { type } = message;
    if (type === 'stop') {
      this.generation++;
      this.solving = false;
      // Pending target replacements must not restart a stopped solver.
      for (const control of this.controls) control.solve = false;
      this.emit({ type: 'status', solving: false, message: 'Solver paused.' });
    } else if (type === 'solve') {
      if (!this.built) return this.emit({ type: 'error', message: 'Build the graph before solving.' });
      this.configureSolve(message);
    } else if (type === 'build' || type === 'targets') {
      if (type === 'targets' && !this.built && !this.controls.some(c => c.command === 'build')) {
        return this.emit({ type: 'error', message: 'Build the graph before changing its priors.' });
      }
      this.generation++;
      this.solving = false;
      const control = {
        command: type, start: message.start, goal: message.goal,
        ...(type === 'build' ? { intermediate_states: message.intermediate_states, duration: message.duration } : {}),
        generation: this.generation, solve: type === 'targets' && message.solve === true,
      };
      if (type === 'build') this.controls = this.controls.filter(c => c.command === 'describe');
      else this.controls = this.controls.filter(c => c.command !== 'targets');
      this.controls.push(control);
      this.emit({ type: 'status', solving: false, message: type === 'build' ? 'Building kinematic slices…' : 'Updating endpoint priors…' });
    } else if (type === 'describe') {
      this.controls.push({ command: 'describe', generation: this.generation });
    } else {
      this.emit({ type: 'error', message: `Unknown command: ${String(type)}` });
    }
    this.wake();
    this.pump();
  }

  configureSolve(message = {}) {
    const clamp = (value, fallback, lo, hi) => Number.isFinite(value) ? Math.max(lo, Math.min(hi, Math.round(value))) : fallback;
    this.remaining = clamp(message.max_iterations, 40, 1, 200);
    this.delay = clamp(message.iteration_delay_ms, 80, 0, 1000);
    this.solving = true;
    this.emit({ type: 'status', solving: true, message: 'Optimizing…' });
  }

  wake() {
    if (this.timer) clearTimeout(this.timer);
    this.timer = null;
    this.resume?.();
    this.resume = null;
  }

  async pump() {
    if (this.busy || this.closed) return;
    this.busy = true;
    try {
      while (!this.closed && (this.controls.length || this.solving)) {
        const control = this.controls.shift();
        const generation = this.generation;
        const result = await this.request(control || { command: 'iterate' });
        if (this.closed) break;
        if (result.type === 'error') {
          this.solving = false;
          this.emit(result);
          this.emit({ type: 'status', solving: false, message: 'Optimizer needs attention.' });
          continue;
        }
        if (control) {
          if (control.command === 'build') this.built = true;
          // Always deliver graph builds; queued targets still need the new topology.
          if (control.command !== 'targets' || control.generation === this.generation) this.emit(result);
          if (control.solve && control.generation === this.generation) this.configureSolve();
        } else if (generation === this.generation && this.solving) {
          this.emit(result);
          this.remaining--;
          if (result.done || result.converged || this.remaining <= 0) {
            this.solving = false;
            const message = result.stop_reason === 'iteration_limit'
              ? 'Native iteration limit reached. Move a target or rebuild to restart.'
              : result.converged
                ? (result.targets_reached === false ? 'Solver stopped with endpoint residuals; inspect the targets.' : 'Converged.')
                : result.done ? 'Solver stopped; inspect the residuals.' : 'Iteration limit reached; solve again to continue.';
            this.emit({ type: 'status', solving: false, message });
          } else if (this.delay > 0 && !this.controls.length) {
            await new Promise(resolve => { this.resume = resolve; this.timer = setTimeout(() => this.wake(), this.delay); });
          }
        }
      }
    } catch (error) {
      if (!this.closed) this.fail(error.message);
    } finally {
      this.busy = false;
    }
  }

  fail(message) {
    if (this.closed) return;
    this.emit({ type: 'error', message });
    this.close();
    this.onClose();
  }

  close() {
    if (this.closed) return;
    this.closed = true;
    this.solving = false;
    this.wake();
    for (const { reject, timer } of this.pending.values()) {
      clearTimeout(timer);
      reject(new Error('Session closed.'));
    }
    this.pending.clear();
    this.lines.close();
    this.child.kill('SIGTERM');
  }
}
