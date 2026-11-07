/** Load one offline STEAP/CMRG build recording and the cell's display geometry. */
async function asset(name, signal) {
  // Resolve relative to the document, not the production bundle's assets directory.
  const response = await fetch(new URL(name, document.baseURI), { signal });
  if (!response.ok) throw new Error(`${name}: HTTP ${response.status}`);
  const bytes = new Uint8Array(await response.arrayBuffer());
  // Static hosts may set Content-Encoding:gzip, which fetch decodes already.
  const text = bytes[0] === 0x1f && bytes[1] === 0x8b
    ? await new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'))).text()
    : new TextDecoder().decode(bytes);
  return JSON.parse(text);
}

export async function loadPresets(signal) {
  return asset('presets.json', signal);
}

export async function loadRecording(file, signal) {
  const [record, model] = await Promise.all([asset(file, signal), asset('model.json.gz', signal)]);
  validateRecording(record, model);
  return { record, model };
}

function finiteArray(value, length, label) {
  if (!Array.isArray(value) || value.length !== length || !value.every(Number.isFinite)) {
    throw new Error(`Invalid ${label}`);
  }
}

/** Validate dense replay arrays before constructing a Three.js scene. */
export function validateRecording(record, model) {
  const frames = record.frames, count = frames?.time?.length;
  if (record.schemaVersion !== 1 || record.kind !== 'steap-cmrg' || !count || count < 2) {
    throw new Error('Invalid STEAP recording dimensions');
  }
  const bodies = record.bodies?.names?.length, movable = record.bodies?.movable?.length;
  if (!bodies || !movable || (model && model.nbody !== bodies)) throw new Error('Robot model/body count mismatch');
  finiteArray(frames.time, count, 'timestamps');
  if (frames.time.some((time, k) => k && time < frames.time[k - 1])) throw new Error('Timestamps must not decrease');
  const members = record.structure?.members?.length;
  for (const [name, expected] of [['bodyPoses', movable], ['tips', 2], ['cable', 2], ['members', members]]) {
    if (frames[name]?.length !== count) throw new Error(`Invalid ${name} sample count`);
    if (frames[name][0].length !== expected) throw new Error(`Invalid ${name} entry count`);
  }
  frames.bodyPoses[0].forEach(pose => finiteArray(pose, 7, 'body pose'));
  frames.members[0].forEach(pose => finiteArray(pose, 7, 'member pose'));
  if (!Array.isArray(record.segments) || !record.segments.length) throw new Error('Missing maneuver segments');
  for (const segment of record.segments) {
    if (segment.frameStart + segment.frameCount > count) throw new Error(`Segment ${segment.key} exceeds frames`);
    if (!segment.subFrames || !segment.planned?.length || segment.replans?.length !== segment.planned.length) {
      throw new Error(`Segment ${segment.key} lacks replans`);
    }
  }
  if (!Array.isArray(record.nails)) throw new Error('Missing nails');
  for (const nail of record.nails) {
    finiteArray(nail.head, 3, 'nail head');
    finiteArray(nail.direction, 3, 'nail direction');
    if (!(nail.frame >= 0 && nail.frame < count)) throw new Error('Nail frame exceeds frames');
  }
  return record;
}

/** Index of the last frame whose time is not after t. */
export function frameAtTime(times, t) {
  let low = 0, high = times.length - 1;
  while (low < high) {
    const mid = (low + high + 1) >> 1;
    if (times[mid] <= t) low = mid; else high = mid - 1;
  }
  return low;
}

/** Index of the segment containing a frame. */
export function segmentAtFrame(segments, frame) {
  let index = 0;
  segments.forEach((segment, i) => { if (segment.frameStart <= frame) index = i; });
  return index;
}
