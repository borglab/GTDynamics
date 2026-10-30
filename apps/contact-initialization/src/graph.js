/**
 * Contact-only event graph. Joint angles have already been consumed by FK;
 * the only variables here are event poses and stationary contact landmarks.
 * Nothing in this module reads a recorded root pose or reference trajectory.
 */

const I = [1, 0, 0, 0, 1, 0, 0, 0, 1];
const add = (a, b) => a.map((v, i) => v + b[i]);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const scale = (a, s) => a.map((v) => v * s);
const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => Math.sqrt(dot(a, a));
const mean = (points) => scale(points.reduce((s, p) => add(s, p), [0, 0, 0]), 1 / points.length);
const mv = (R, p) => [dot(R.slice(0, 3), p), dot(R.slice(3, 6), p), dot(R.slice(6, 9), p)];
const transpose = (R) => [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
const multiply = (A, B) => {
  const BT = transpose(B);
  return Array.from({ length: 9 }, (_, i) => dot(A.slice(3 * Math.floor(i / 3), 3 * Math.floor(i / 3) + 3), BT.slice(3 * (i % 3), 3 * (i % 3) + 3)));
};
const copyPose = (pose) => ({ rotation: [...pose.rotation], translation: [...pose.translation] });
const copyPoses = (poses) => poses.map(copyPose);
const copyPoints = (points) => points.map((p) => [...p]);
export const identityPose = () => ({ rotation: [...I], translation: [0, 0, 0] });

export function quaternionToRotation(q) {
  const length = Math.hypot(...q);
  if (!(length > 0)) throw new Error('A foot quaternion must be nonzero.');
  const [w, x, y, z] = q.map((v) => v / length);
  return [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)];
}

export const transformPoint = (pose, point) => add(mv(pose.rotation, point), pose.translation);
export const composePoses = (a, b) => ({ rotation: multiply(a.rotation, b.rotation), translation: transformPoint(a, b.translation) });
export const inversePose = (pose) => {
  const rotation = transpose(pose.rotation);
  return { rotation, translation: scale(mv(rotation, pose.translation), -1) };
};

/** Largest eigenvector of a symmetric matrix, using Jacobi rotations. */
function largestEigenvector(matrix) {
  const A = matrix.map((row) => [...row]);
  const V = Array.from({ length: 4 }, (_, i) => Array.from({ length: 4 }, (_, j) => Number(i === j)));
  for (let iteration = 0; iteration < 64; iteration++) {
    let p = 0, q = 1;
    for (let i = 0; i < 4; i++) for (let j = i + 1; j < 4; j++) if (Math.abs(A[i][j]) > Math.abs(A[p][q])) [p, q] = [i, j];
    if (Math.abs(A[p][q]) < 1e-14) break;
    const angle = 0.5 * Math.atan2(2 * A[p][q], A[q][q] - A[p][p]);
    const c = Math.cos(angle), s = Math.sin(angle);
    const app = A[p][p], aqq = A[q][q], apq = A[p][q];
    for (let k = 0; k < 4; k++) if (k !== p && k !== q) {
      const akp = A[k][p], akq = A[k][q];
      A[k][p] = A[p][k] = c * akp - s * akq;
      A[k][q] = A[q][k] = s * akp + c * akq;
    }
    A[p][p] = c * c * app - 2 * s * c * apq + s * s * aqq;
    A[q][q] = s * s * app + 2 * s * c * apq + c * c * aqq;
    A[p][q] = A[q][p] = 0;
    for (let k = 0; k < 4; k++) {
      const vkp = V[k][p], vkq = V[k][q];
      V[k][p] = c * vkp - s * vkq;
      V[k][q] = s * vkp + c * vkq;
    }
  }
  let largest = 0;
  for (let i = 1; i < 4; i++) if (A[i][i] > A[largest][largest]) largest = i;
  return V.map((row) => row[largest]);
}

/** Rank of the pose block of point-contact factors, with landmarks fixed. */
export function contactPoseRank(points) {
  if (!points.length) return 0;
  const origin = points[0];
  const first = points.find((p) => norm(sub(p, origin)) > 1e-8);
  if (!first) return 3;
  const direction = sub(first, origin);
  return points.some((p) => norm(cross(direction, sub(p, origin))) > 1e-9) ? 6 : 5;
}

function rotationBetween(a, b) {
  const an = norm(a), bn = norm(b);
  if (an < 1e-12 || bn < 1e-12) return [...I];
  const u = scale(a, 1 / an), v = scale(b, 1 / bn);
  const d = Math.max(-1, Math.min(1, dot(u, v)));
  if (d > 1 - 1e-12) return [...I];
  if (d < -1 + 1e-12) {
    const basis = Math.abs(u[0]) < 0.8 ? [1, 0, 0] : [0, 1, 0];
    const axis = cross(u, basis);
    return quaternionToRotation([0, ...scale(axis, 1 / norm(axis))]);
  }
  return quaternionToRotation([1 + d, ...cross(u, v)]);
}

/** Exact least-squares rigid registration; degenerate data retain free attitude. */
export function fitRigidTransform(local, world, previous = identityPose()) {
  if (!local.length || local.length !== world.length) throw new Error('Rigid registration needs matching nonempty point sets.');
  const a = mean(local), b = mean(world);
  const rank = contactPoseRank(local);
  let rotation;
  if (rank === 3) rotation = [...previous.rotation];
  else if (rank === 5) {
    let index = 1;
    for (let i = 1; i < local.length; i++) if (norm(sub(local[i], local[0])) > norm(sub(local[index], local[0]))) index = i;
    const direction = sub(local[index], local[0]);
    const axis = scale(direction, 1 / norm(direction));
    // For collinear local points, covariance supplies the exact best world axis,
    // including repeated/noisy measurements; rotation around it remains free.
    let worldAxis = [0, 0, 0];
    for (let i = 0; i < local.length; i++) worldAxis = add(worldAxis, scale(sub(world[i], b), dot(sub(local[i], a), axis)));
    rotation = multiply(rotationBetween(mv(previous.rotation, axis), worldAxis), previous.rotation);
  } else {
    const S = Array(9).fill(0);
    for (let k = 0; k < local.length; k++) {
      const x = sub(local[k], a), y = sub(world[k], b);
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) S[3 * i + j] += x[i] * y[j];
    }
    const [xx, xy, xz, yx, yy, yz, zx, zy, zz] = S;
    rotation = quaternionToRotation(largestEigenvector([
      [xx + yy + zz, yz - zy, zx - xz, xy - yx],
      [yz - zy, xx - yy - zz, xy + yx, zx + xz],
      [zx - xz, xy + yx, -xx + yy - zz, yz + zy],
      [xy - yx, zx + xz, yz + zy, -xx - yy + zz],
    ]));
  }
  return { rotation, translation: sub(b, mv(rotation, a)) };
}

function noiseGenerator() {
  let state = 0x71a934bd;
  return () => {
    state = (Math.imul(state, 1664525) + 1013904223) >>> 0;
    return (state / 0x100000000 - 0.5) * Math.sqrt(12);
  };
}

function averageLandmarks(graph, poses) {
  const sums = graph.landmarks.map(() => [0, 0, 0]);
  const counts = graph.landmarks.map(() => 0);
  for (let i = 0; i < graph.frames.length; i++) for (const observation of graph.frames[i].observations) {
    const index = observation.landmarkIndex;
    sums[index] = add(sums[index], transformPoint(poses[i], observation.local));
    counts[index]++;
  }
  return sums.map((sum, i) => {
    const point = scale(sum, 1 / Math.max(1, counts[i]));
    if (graph.groundPlane) point[2] = graph.groundPlane.height;
    return point;
  });
}

function soleLevelRotation(frame) {
  const contact = frame.observations[0]?.contactId;
  const points = frame.observations.filter((o) => o.contactId === contact).map((o) => o.local);
  if (points.length < 3) throw new Error('A level-floor seed requires a sole patch.');
  let normal = cross(sub(points[1], points[0]), sub(points[2], points[0]));
  normal = scale(normal, (normal[2] < 0 ? -1 : 1) / norm(normal));
  return rotationBetween(normal, [0, 0, 1]);
}

function seedLandmark(graph, pose, local) {
  const point = transformPoint(pose, local);
  if (graph.groundPlane) point[2] = graph.groundPlane.height;
  return point;
}

function initializeGraph(graph) {
  const poses = graph.frames.map(() => null);
  const landmarks = graph.landmarks.map(() => null);
  const components = [];
  const anchorFrameIndices = [];
  while (poses.some((p) => p === null)) {
    const anchor = poses.findIndex((p) => p === null);
    const component = [];
    const pose = identityPose();
    // This is only a display gauge for disconnected components, never a link.
    pose.translation[0] = components.length * 1.2;
    if (graph.groundPlane) {
      pose.rotation = soleLevelRotation(graph.frames[anchor]);
      const support = graph.frames[anchor].observations[0]?.contactId;
      const points = graph.frames[anchor].observations.filter((o) => o.contactId === support).map((o) => mv(pose.rotation, o.local));
      pose.translation[2] = graph.groundPlane.height - mean(points)[2];
    }
    poses[anchor] = pose;
    anchorFrameIndices.push(anchor);
    component.push(anchor);
    for (const observation of graph.frames[anchor].observations) landmarks[observation.landmarkIndex] = seedLandmark(graph, pose, observation.local);
    let changed = true;
    while (changed) {
      changed = false;
      for (let i = 0; i < graph.frames.length; i++) if (poses[i] === null) {
        const known = graph.frames[i].observations.filter((o) => landmarks[o.landmarkIndex] !== null);
        if (!known.length) continue;
        poses[i] = fitRigidTransform(known.map((o) => o.local), known.map((o) => landmarks[o.landmarkIndex]));
        component.push(i);
        for (const observation of graph.frames[i].observations) if (landmarks[observation.landmarkIndex] === null) landmarks[observation.landmarkIndex] = seedLandmark(graph, poses[i], observation.local);
        changed = true;
      }
    }
    components.push(component);
  }
  return { poses, landmarks: averageLandmarks(graph, poses), components, anchorFrameIndices };
}

/**
 * Build a graph from a recorded sequence's local FK and force-annotated schedule.
 * `noise` is deterministic synthetic local-point measurement noise, in metres.
 */
export function buildGraph(data, { model = 'patch', noise = 0, groundPlane = false } = {}) {
  if (!['patch', 'point'].includes(model)) throw new Error('Contact model must be patch or point.');
  if (groundPlane && model !== 'patch') throw new Error('The level-floor option requires the sole-patch contact model.');
  if (!Number.isFinite(noise) || noise < 0) throw new Error('Measurement noise must be a nonnegative finite number.');
  if (!Array.isArray(data.frames) || !data.frames.length) throw new Error('The sequence needs at least one event frame.');
  const soleCorners = data.model.soleCorners;
  if (!Array.isArray(soleCorners) || soleCorners.length < 3 || contactPoseRank(soleCorners) !== 6) throw new Error('The sole patch requires three noncollinear FK points.');
  const centers = data.model.soleCenter || mean(soleCorners);
  const localSole = model === 'point' ? [centers] : soleCorners.slice(0, 3);
  const random = noiseGenerator();
  const contactById = new Map();
  const landmarks = [];
  const activeIds = new Set(data.frames.flatMap((frame) => frame.contacts));
  const contacts = data.contacts.filter((c) => activeIds.has(c.id)).map((contact) => {
    const copy = { ...contact, foot: contact.foot ?? contact.side, landmarkIndices: [] };
    for (let i = 0; i < localSole.length; i++) {
      copy.landmarkIndices.push(landmarks.length);
      landmarks.push({ id: `${contact.id}:${i}`, contactId: contact.id, foot: copy.foot, pointIndex: i });
    }
    contactById.set(copy.id, copy);
    return copy;
  });
  const frames = data.frames.map((frame, i) => {
    const sampleIndex = frame.sampleIndex ?? frame.index;
    const sample = data.samples[sampleIndex];
    if (!sample) throw new Error(`Event frame ${i} has no source sample.`);
    const observations = [];
    for (const id of frame.contacts) {
      const contact = contactById.get(id);
      if (!contact) throw new Error(`Unknown contact episode: ${id}`);
      const foot = sample.feet[contact.foot];
      if (!foot) throw new Error(`Missing local FK for ${contact.foot} foot.`);
      const footPose = { rotation: foot.rotation ?? quaternionToRotation(foot.q), translation: foot.p };
      for (let j = 0; j < localSole.length; j++) {
        const local = transformPoint(footPose, localSole[j]).map((v) => v + noise * random());
        const landmarkIndex = contact.landmarkIndices[j];
        observations.push({ landmarkIndex, landmarkId: landmarks[landmarkIndex].id, contactId: id, foot: contact.foot, local });
      }
    }
    return { id: `frame-${i}`, index: i, sampleIndex, time: frame.t ?? sample.t, contacts: [...frame.contacts], starts: [...(frame.starts ?? [])], ends: [...(frame.ends ?? [])], observations };
  });
  const graph = { model, noise, frames, contacts, landmarks, groundPlane: null };
  if (groundPlane) {
    const rotation = soleLevelRotation(frames[0]);
    const support = frames[0].observations[0]?.contactId;
    const points = frames[0].observations.filter((o) => o.contactId === support).map((o) => mv(rotation, o.local));
    graph.groundPlane = { enabled: true, height: mean(points)[2], normal: [0, 0, 1], derivedFromFrame: 0 };
  }
  const seed = initializeGraph(graph);
  graph.initialPoses = copyPoses(seed.poses);
  graph.seedPoses = copyPoses(seed.poses);
  graph.initialLandmarks = copyPoints(seed.landmarks);
  graph.anchorFrameIndices = seed.anchorFrameIndices;
  graph.components = seed.components;
  const frameRanks = frames.map((frame) => contactPoseRank(frame.observations.map((o) => o.local)));
  const anchors = new Set(seed.anchorFrameIndices);
  const locallyUnobservableDofs = frameRanks.reduce((sum, rank, i) => sum + (anchors.has(i) ? 0 : 6 - rank), 0);
  const observations = frames.reduce((sum, frame) => sum + frame.observations.length, 0);
  graph.dimensions = { poseVariables: 6 * frames.length, landmarkVariables: 3 * landmarks.length, jointVariables: 0, totalVariables: 6 * frames.length + 3 * landmarks.length, residuals: 3 * observations, observations, floorConstraints: groundPlane ? landmarks.length : 0, effectiveLandmarkVariables: (groundPlane ? 2 : 3) * landmarks.length };
  const gaugeDofs = (groundPlane ? 3 : 6) * seed.components.length;
  const freeDofsLowerBound = Math.max(locallyUnobservableDofs, graph.dimensions.totalVariables - graph.dimensions.floorConstraints - gaugeDofs - graph.dimensions.residuals, 0);
  graph.observability = {
    componentCount: seed.components.length,
    components: seed.components,
    gaugeDofs,
    frameRanks,
    locallyUnobservableDofs,
    freeDofsLowerBound,
    fullyConstrained: model === 'patch' && frameRanks.every((rank) => rank === 6),
    explanation: model === 'patch'
      ? groundPlane
        ? 'The known level floor constrains each contact landmark to one horizontal plane. First-sole FK sets the initial tilt and floor height; no recorded root information is used. Horizontal position and heading remain gauge choices in each support component. Rigid soles must still not slip or rock.'
        : 'Three noncollinear stationary sole points determine relative pose. Each disconnected support component retains a separate SE(3) gauge; its first frame is anchored for display. Flat feet must not slip or rock.'
      : 'A point fixes three pose directions; two distinct points fix five. Unobserved rotations remain after anchoring. The displayed solution is one choice among many, not a uniquely recovered attitude.',
  };
  return graph;
}

export function graphObjective(graph, poses, landmarks) {
  let objective = 0;
  for (let i = 0; i < graph.frames.length; i++) for (const observation of graph.frames[i].observations) {
    const residual = sub(transformPoint(poses[i], observation.local), landmarks[observation.landmarkIndex]);
    objective += dot(residual, residual);
  }
  return objective;
}

/**
 * Block coordinate descent on Σ ||R_i p_ia + t_i - l_a||². Both blocks are
 * solved exactly: centroid landmark updates and Horn rigid pose registration.
 * Component anchors remain fixed throughout. There is no trajectory prior.
 */
export function optimizeGraph(graph, { iterations = 120, initialPoses = graph.initialPoses, tolerance = 1e-11, snapshots = true } = {}) {
  const start = performance.now();
  if (initialPoses.length !== graph.frames.length) throw new Error('Initial pose count must match event frames.');
  let poses = copyPoses(initialPoses);
  const anchors = new Set(graph.anchorFrameIndices);
  // Canonical gauge is always defined by graph construction, not user perturbations.
  for (const index of anchors) poses[index] = copyPose(graph.seedPoses[index]);
  let landmarks = averageLandmarks(graph, poses);
  const count = graph.dimensions.observations;
  const history = [];
  const record = (iteration, objective) => {
    const entry = { iteration, objective, rms: Math.sqrt(objective / Math.max(1, count)) };
    if (snapshots) Object.assign(entry, { poses: copyPoses(poses), landmarks: copyPoints(landmarks) });
    history.push(entry);
  };
  let objective = graphObjective(graph, poses, landmarks);
  let converged = false;
  record(0, objective);
  for (let iteration = 1; iteration <= iterations; iteration++) {
    const previous = objective;
    for (let i = 0; i < graph.frames.length; i++) if (!anchors.has(i) && graph.frames[i].observations.length) {
      const observations = graph.frames[i].observations;
      poses[i] = fitRigidTransform(observations.map((o) => o.local), observations.map((o) => landmarks[o.landmarkIndex]), poses[i]);
    }
    landmarks = averageLandmarks(graph, poses);
    objective = graphObjective(graph, poses, landmarks);
    // Exact block updates must not materially increase the objective.
    if (!Number.isFinite(objective)) throw new Error('Contact optimization produced a non-finite objective.');
    if (objective > previous + 1e-12 * Math.max(1, previous)) throw new Error('Contact objective increased during an exact block update.');
    record(iteration, objective);
    if (previous - objective <= tolerance * Math.max(1, previous)) {
      converged = true;
      break;
    }
  }
  return { poses, landmarks, history, residualRms: Math.sqrt(objective / Math.max(1, count)), objectiveInitial: history[0].objective, objectiveFinal: objective, iterations: history.length - 1, timeMs: performance.now() - start, observability: graph.observability, dimensions: graph.dimensions, converged };
}
