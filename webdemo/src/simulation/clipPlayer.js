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

    const safeTime = Number.isFinite(timeSec) ? Math.max(0, timeSec) : 0;
    let frameIndex = Math.round(safeTime / this.dt);
    if (frameIndex < 0) {
      frameIndex = 0;
    }
    if (frameIndex >= this.frameCount) {
      frameIndex = this.frameCount - 1;
    }

    this.currentFrameIndex = frameIndex;
    const jointPos = this.jointFrames[frameIndex] ?? this.jointFrames[this.jointFrames.length - 1];
    const rootPos = this.rootPosFrames.length > 0
      ? (this.rootPosFrames[frameIndex] ?? this.rootPosFrames[this.rootPosFrames.length - 1])
      : null;
    const rootQuat = this.rootQuatFrames.length > 0
      ? (this.rootQuatFrames[frameIndex] ?? this.rootQuatFrames[this.rootQuatFrames.length - 1])
      : null;

    return {
      frameIndex,
      jointPos,
      rootPos,
      rootQuat
    };
  }
}
