function pickFirstDefined(...values) {
  for (const value of values) {
    if (value !== undefined && value !== null) {
      return value;
    }
  }
  return null;
}

function asArray(value) {
  return Array.isArray(value) ? value : [];
}

// GTD edit from the upstream policy viewer:
// interpolate clip samples to reduce stair-step target jitter.
function lerpScalar(a, b, t) {
  return a + ((b - a) * t);
}

function lerpArray(frame0, frame1, alpha) {
  if (!Array.isArray(frame0)) {
    return frame1;
  }
  if (!Array.isArray(frame1) || alpha <= 0) {
    return frame0;
  }
  if (alpha >= 1) {
    return frame1;
  }

  const size = Math.min(frame0.length, frame1.length);
  const out = new Array(size);
  for (let i = 0; i < size; i += 1) {
    const a = Number(frame0[i]);
    const b = Number(frame1[i]);
    out[i] = (Number.isFinite(a) && Number.isFinite(b)) ? lerpScalar(a, b, alpha) : (frame0[i] ?? frame1[i]);
  }
  return out;
}

function nlerpQuat(quat0, quat1, alpha) {
  const q = lerpArray(quat0, quat1, alpha);
  if (!Array.isArray(q) || q.length < 4) {
    return q;
  }

  const w = Number(q[0]);
  const x = Number(q[1]);
  const y = Number(q[2]);
  const z = Number(q[3]);
  const norm = Math.hypot(w, x, y, z);
  if (!Number.isFinite(norm) || norm <= 1e-12) {
    return q;
  }
  return [w / norm, x / norm, y / norm, z / norm];
}

export class ClipPlayer {
  constructor(defaultFps = 60) {
    this.defaultFps = defaultFps;
    this.reset();
  }

  reset() {
    this.clipName = 'default';
    this.jointFrames = [];
    this.rootPosFrames = [];
    this.rootQuatFrames = [];
    this.frameCount = 0;
    this.fps = this.defaultFps;
    this.dt = 1 / this.defaultFps;
    this.duration = 0;
    this.currentFrameIndex = 0;
    this.jointNames = [];
  }

  loadClip(payload, options = {}) {
    if (!payload || typeof payload !== 'object') {
      throw new Error('Clip payload must be a JSON object');
    }

    const clipName = options.clipName ?? 'default';
    const clips = payload.clips && typeof payload.clips === 'object' ? payload.clips : null;
    const clipPayload = clips ? (clips[clipName] ?? clips.default ?? Object.values(clips)[0]) : payload;
    if (!clipPayload || typeof clipPayload !== 'object') {
      throw new Error(`Clip '${clipName}' was not found in payload`);
    }

    const rawJointFrames = pickFirstDefined(clipPayload.joint_pos, clipPayload.jointPos);
    const rawRootPosFrames = pickFirstDefined(clipPayload.root_pos, clipPayload.rootPos);
    const rawRootQuatFrames = pickFirstDefined(clipPayload.root_quat, clipPayload.rootQuat);

    const jointFrames = asArray(rawJointFrames);
    if (jointFrames.length === 0) {
      throw new Error('Motion clip must include non-empty joint_pos (or jointPos) frames');
    }

    const fps = Number(
      pickFirstDefined(options.fps, clipPayload.fps, payload.fps)
    );
    const dt = Number(
      pickFirstDefined(options.dt, clipPayload.dt, payload.dt)
    );

    this.fps = Number.isFinite(fps) && fps > 0
      ? fps
      : (Number.isFinite(dt) && dt > 0 ? 1 / dt : this.defaultFps);
    this.dt = Number.isFinite(dt) && dt > 0 ? dt : 1 / this.fps;

    this.clipName = clipName;
    this.jointFrames = jointFrames;
    this.rootPosFrames = asArray(rawRootPosFrames);
    this.rootQuatFrames = asArray(rawRootQuatFrames);
    this.frameCount = this.jointFrames.length;
    this.duration = Math.max(0, (this.frameCount - 1) * this.dt);
    this.currentFrameIndex = 0;
    this.jointNames = asArray(payload.joint_names ?? payload.jointNames).slice();
  }

  getTargets(timeSec = 0) {
    if (this.frameCount === 0) {
      return null;
    }

    // GTD edit: sample by continuous time with interpolation rather than
    // nearest-neighbor frame snapping.
    const safeTime = Number.isFinite(timeSec) ? Math.max(0, timeSec) : 0;
    const frameFloat = safeTime / this.dt;
    let frame0 = Math.floor(frameFloat);
    if (frame0 < 0) {
      frame0 = 0;
    }
    if (frame0 >= this.frameCount) {
      frame0 = this.frameCount - 1;
    }

    const frame1 = Math.min(frame0 + 1, this.frameCount - 1);
    const alpha = frame1 > frame0 ? Math.max(0, Math.min(1, frameFloat - frame0)) : 0;

    this.currentFrameIndex = frame0;

    const joint0 = this.jointFrames[frame0] ?? this.jointFrames[this.jointFrames.length - 1];
    const joint1 = this.jointFrames[frame1] ?? joint0;
    const jointPos = lerpArray(joint0, joint1, alpha);

    let rootPos = null;
    if (this.rootPosFrames.length > 0) {
      const rootPos0 = this.rootPosFrames[frame0] ?? this.rootPosFrames[this.rootPosFrames.length - 1];
      const rootPos1 = this.rootPosFrames[frame1] ?? rootPos0;
      rootPos = lerpArray(rootPos0, rootPos1, alpha);
    }

    let rootQuat = null;
    if (this.rootQuatFrames.length > 0) {
      const rootQuat0 = this.rootQuatFrames[frame0] ?? this.rootQuatFrames[this.rootQuatFrames.length - 1];
      const rootQuat1 = this.rootQuatFrames[frame1] ?? rootQuat0;
      rootQuat = nlerpQuat(rootQuat0, rootQuat1, alpha);
    }

    return {
      frameIndex: frame0,
      jointPos,
      rootPos,
      rootQuat
    };
  }
}
