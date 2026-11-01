/** Static recording access. Offsets are float counts, poses are body-frame xyzw. */
export const referenceLabel = 'LiDAR-inertial reference';
const recordings = new Map();
let cachedModel;

export function clearRecordingCache() {
  recordings.clear(); cachedModel = undefined;
}

export function historyFor(record, method, stage, frame) {
  if (stage === 'initializer' && record.availableStages && !record.availableStages.includes(stage)) {
    throw new Error('Initializer history has not been loaded');
  }
  const run = record.runs[method];
  return stage === 'smoothing' ? run.smoothing.slice(1)
    : method === 'ik' ? run.ikFrames[frame] : run.initializer;
}

export function bodyOffset(record, state, frame, body = 0) {
  return state.poseOffset + ((state.frames === 1 ? 0 : frame) * record.bodies + body) * 7;
}

export function jointValue(record, floats, state, frame, joint) {
  return floats[state.jointOffset + (state.frames === 1 ? 0 : frame) * record.jointNames.length + joint];
}

/** Decode delta-coded display coordinates; all numerical diagnostics stay in JSON. */
export function decodeStates(buffer, storage) {
  if (storage.codec !== 'delta-shuffle-int32-v1' || storage.scale !== 1000000 || buffer.byteLength % 4) {
    throw new Error('Unsupported state encoding');
  }
  const bytes = new Uint8Array(buffer), count = bytes.length / 4;
  if (!storage.blocks.every(n => Number.isInteger(n) && n > 0)
      || storage.blocks.reduce((a,b) => a+b, 0) !== count) throw new Error('Truncated encoded states');
  const floats = new Float32Array(count), previous = new Map();
  let offset = 0;
  for (const size of storage.blocks) {
    if (!previous.has(size)) previous.set(size, new Int32Array(size));
    const prior = previous.get(size);
    for (let j=0; j<size; j++) {
      const i=offset+j;
      const delta=bytes[i] | (bytes[i+count]<<8) | (bytes[i+2*count]<<16) | (bytes[i+3*count]<<24);
      prior[j] += delta;
      floats[i] = prior[j] / storage.scale;
    }
    offset += size;
  }
  return floats;
}

export function validateRecording(record, floats) {
  if (record.schemaVersion !== 1 || record.samples < 2 || record.bodies < 1 || record.dt <= 0) {
    throw new Error('Unsupported or incomplete kinematics recording. Regenerate it with export_recording.py.');
  }
  const states = [record.reference];
  for (const method of ['contact', 'ik']) {
    const run = record.runs[method];
    if (run.smoothing.length !== run.solver.iterations + 2) throw new Error(`Incomplete ${method} smoothing history`);
    states.push(...run.smoothing, ...run.initializer, ...run.ikFrames.flat());
  }
  const includesInitializers = !record.availableStages || record.availableStages.includes('initializer');
  if (includesInitializers && (record.runs.ik.ikFrames.length !== record.samples || record.runs.ik.ikFrames.some(h => !h.length))) {
    throw new Error('Incomplete per-frame IK history');
  }
  for (const state of states) {
    if (state.velocityOffset !== undefined && (!Number.isInteger(state.velocityOffset)
        || state.velocityOffset < 0 || state.velocityOffset + state.frames*3 > floats.length
        || state.bias?.length !== 6 || !state.bias.every(Number.isFinite))) {
      throw new Error('Invalid IMU velocity or bias storage');
    }
    if (!Number.isInteger(state.poseOffset) || !Number.isInteger(state.jointOffset)
        || state.poseOffset < 0 || state.jointOffset < 0
        || ![1, record.samples].includes(state.frames)
        || state.poseOffset + state.frames * record.bodies * 7 > floats.length
        || state.jointOffset + state.frames * record.jointNames.length > floats.length) {
      throw new Error('Truncated state buffer');
    }
  }
}

async function fetchFile(name, signal, compressed = false) {
  const response = await fetch(`${import.meta.env.BASE_URL}${name}`, { signal });
  if (!response.ok) throw new Error(`Could not load ${name} (${response.status}). Run the offline exporter.`);
  if (!compressed) return response.json();
  const buffer = await response.arrayBuffer();
  const bytes = new Uint8Array(buffer);
  // Some static hosts mark .gz files Content-Encoding:gzip, so fetch already
  // decompresses them. Others serve the archive itself. Support both cases.
  if (bytes[0] !== 0x1f || bytes[1] !== 0x8b) return buffer;
  if (typeof DecompressionStream === 'undefined') throw new Error('This viewer needs a browser with gzip stream support.');
  return new Response(new Blob([buffer]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
}

async function loadModel(record, directory, signal) {
  if (cachedModel && cachedModel.fingerprint === record.modelFingerprint) return cachedModel.model;
  const buffer = await fetchFile(`${directory}${record.storage.model}`, signal, true);
  const model = JSON.parse(new TextDecoder().decode(buffer));
  signal.throwIfAborted();
  cachedModel = {fingerprint: record.modelFingerprint, model};
  return model;
}

export async function loadRecording(signal, directory = '', stage = 'smoothing') {
  signal.throwIfAborted();
  const key = `${directory}|${stage}`;
  const cachedKey = recordings.has(`${directory}|initializer`) ? `${directory}|initializer` : key;
  if (recordings.has(cachedKey)) {
    const cached = recordings.get(cachedKey);
    recordings.delete(cachedKey); recordings.set(cachedKey, cached);
    return cached;
  }
  const name = stage === 'smoothing' ? 'smoothing.json.gz' : 'recording.json.gz';
  const metadata = await fetchFile(`${directory}${name}`, signal, true);
  const record = JSON.parse(new TextDecoder().decode(metadata));
  const [stateBuffer, model] = await Promise.all([
    fetchFile(`${directory}${record.storage.states}`, signal, true), loadModel(record, directory, signal),
  ]);
  signal.throwIfAborted();
  const floats = decodeStates(stateBuffer, record.storage);
  validateRecording(record, floats);
  const data = {record, floats, model};
  if (stage === 'initializer') recordings.delete(`${directory}|smoothing`);
  recordings.set(key, data);
  let retainedBytes = [...recordings.values()].reduce((sum, entry) => sum + entry.floats.byteLength, 0);
  while (recordings.size > 2 || (recordings.size > 1 && retainedBytes > 192*1024*1024)) {
    const oldest = recordings.keys().next().value;
    retainedBytes -= recordings.get(oldest).floats.byteLength;
    recordings.delete(oldest);
  }
  return data;
}
