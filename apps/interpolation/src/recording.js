/** The browser replays native FK; it does not load a GP solver or MuJoCo. */
import { decodeStates } from '../../shared/kinematics/recording.js';
export async function readGzip(name, signal) {
  const response = await fetch(`${import.meta.env.BASE_URL}${name}`, { signal });
  if (!response.ok) throw new Error(`Could not load ${name}: HTTP ${response.status}`);
  const bytes = new Uint8Array(await response.arrayBuffer());
  if (bytes[0] !== 0x1f || bytes[1] !== 0x8b) return bytes.buffer;
  return new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
}

export function validateRecording(record, floats) {
  if (record.schemaVersion !== 1 || record.storage.codec !== 'delta-shuffle-int32-v1'
      || floats.length !== record.storage.length || record.times.length < 2
      || !record.times.every((time, i) => Number.isFinite(time) && (!i || time > record.times[i - 1]))) {
    throw new Error('Invalid Interpolation recording');
  }
  for (const [name, spec] of Object.entries(record.storage.arrays)) {
    const count = spec.shape.reduce((a, b) => a * b, 1);
    if (!Number.isInteger(spec.offset) || spec.offset < 0 || spec.offset + count > floats.length
        || spec.shape[0] !== record.times.length) throw new Error(`Invalid ${name} storage`);
  }
  if (record.contacts.length !== record.times.length
      || record.knots.frames.some((frame, i) => !Number.isInteger(frame)
        || frame < 0 || frame >= record.times.length
        || Math.abs(record.times[frame] - record.knots.times[i]) > 1e-8)) {
    throw new Error('Contact or knot timestamps do not match the replay');
  }
}

export async function loadRecording(signal) {
  const record = JSON.parse(new TextDecoder().decode(await readGzip('recording.json.gz', signal)));
  const [buffer, geometry] = await Promise.all([
    readGzip(record.storage.states, signal), readGzip(record.storage.model, signal),
  ]);
  signal.throwIfAborted();
  const floats = decodeStates(buffer, record.storage);
  validateRecording(record, floats);
  const arrays = Object.fromEntries(Object.entries(record.storage.arrays).map(([name, spec]) => [
    name, floats.subarray(spec.offset, spec.offset + spec.shape.reduce((a, b) => a * b, 1)),
  ]));
  return { record, arrays, model: JSON.parse(new TextDecoder().decode(geometry)) };
}

/** First sample at or after time, clamped to the saved recording. */
export function frameAtTime(times, time) {
  let first = 0, end = times.length;
  while (first < end) {
    const middle = (first + end) >>> 1;
    if (times[middle] < time) first = middle + 1;
    else end = middle;
  }
  return Math.min(first, times.length - 1);
}
