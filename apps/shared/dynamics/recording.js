/** Load one offline dynamics result and independent compiled display geometry. */
export async function loadRecording(signal) {
  // Resolve relative to the document, not the production bundle's assets directory.
  async function asset(name) {
    const response = await fetch(new URL(name, document.baseURI), {signal});
    if (!response.ok) throw new Error(`${name}: HTTP ${response.status}`);
    const bytes = new Uint8Array(await response.arrayBuffer());
    // Static hosts may set Content-Encoding:gzip, which fetch decodes already.
    const text = bytes[0] === 0x1f && bytes[1] === 0x8b
      ? await new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'))).text()
      : new TextDecoder().decode(bytes);
    return JSON.parse(text);
  }
  const [record, model] = await Promise.all([asset('recording.json.gz'), asset('model.json.gz')]);
  validateRecording(record, model);
  return {record, model};
}

function finiteArray(value, length, label) {
  if (!Array.isArray(value) || value.length !== length || !value.every(Number.isFinite)) {
    throw new Error(`Invalid ${label}`);
  }
}

/** Validate dense replay arrays before constructing a Three.js scene. */
export function validateRecording(record, model) {
  const n = record.times?.length, b = record.bodyNames?.length, j = record.jointNames?.length;
  if (record.schemaVersion !== 1 || !n || n < 2 || !b || j !== 29) throw new Error('Invalid dynamics recording dimensions');
  finiteArray(record.times, n, 'timestamps');
  if (record.times.some((time, k) => k && time <= record.times[k - 1])) throw new Error('Timestamps must increase');
  if (model && model.nbody !== b) throw new Error('Robot model/body count mismatch');
  if (!Number.isInteger(record.rootBody) || record.rootBody < 0 || record.rootBody >= b) throw new Error('Invalid root body');
  for (const [name, count, size] of [['bodyPoses',b,7], ['contactPoses',2,7], ['wrenches',2,6], ['worldForces',2,3], ['worldMoments',2,3]]) {
    if (record[name]?.length !== n) throw new Error(`Invalid ${name} sample count`);
    record[name].forEach(sample => {
      if (sample.length !== count) throw new Error(`Invalid ${name} count`);
      sample.forEach(value => finiteArray(value, size, name));
    });
  }
  if (record.contact?.length !== n || record.jointTorques?.length !== n) throw new Error('Invalid dynamics sample count');
  record.contact.forEach(sample => {
    if (sample.length !== 2 || !sample.every(value => typeof value === 'boolean')) throw new Error('Invalid contact flags');
  });
  record.jointTorques.forEach(sample => finiteArray(sample, j, 'joint torques'));
  if (record.settings?.accelerationsEstimated) {
    for (const [name, size] of [['baseTwists',6], ['baseTwistAccelerations',6],
      ['baseAccelerationPriors',6], ['jointAngles',j], ['jointVelocities',j],
      ['jointAccelerations',j], ['jointAccelerationPriors',j]]) {
      if (record[name]?.length !== n) throw new Error(`Invalid ${name} sample count`);
      record[name].forEach(sample => finiteArray(sample, size, name));
    }
    if (record.linkTwistAccelerations?.length !== n) throw new Error('Invalid link accelerations');
    record.linkTwistAccelerations.forEach(sample => {
      if (sample.length !== b) throw new Error('Invalid link acceleration count');
      sample.forEach(acceleration => finiteArray(acceleration, 6, 'link acceleration'));
    });
  }
  if (record.jointEffortLimits) finiteArray(record.jointEffortLimits, j, 'joint effort limits');
  return record;
}

/** Nearest retained slice in original dataset seconds; no state interpolation. */
export function frameAtTime(times, time) {
  let low = 0, high = times.length - 1;
  while (low < high) {
    const middle = Math.floor((low + high) / 2);
    if (times[middle] < time) low = middle + 1; else high = middle;
  }
  return low > 0 && Math.abs(times[low - 1] - time) <= Math.abs(times[low] - time) ? low - 1 : low;
}

/** Group every actuator exactly once without depending on model joint order. */
export function jointGroups(names) {
  const groups = [
    ['Left leg', name => name.startsWith('left_') && /hip|knee|ankle/.test(name)],
    ['Right leg', name => name.startsWith('right_') && /hip|knee|ankle/.test(name)],
    ['Waist', name => name.startsWith('waist_')],
    ['Left arm', name => name.startsWith('left_') && /shoulder|elbow|wrist/.test(name)],
    ['Right arm', name => name.startsWith('right_') && /shoulder|elbow|wrist/.test(name)],
  ];
  const used = new Set();
  const result = groups.map(([title, predicate]) => ({title, indices:names.flatMap((name, i) => {
    if (!predicate(name)) return [];
    used.add(i); return [i];
  })})).filter(group => group.indices.length);
  const other = names.flatMap((_, i) => used.has(i) ? [] : [i]);
  if (other.length) result.push({title:'Other joints', indices:other});
  return result;
}
