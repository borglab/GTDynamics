export const stateNames = {
  3: "Support (FG)",
  4: "Swing (FG)",
  2: "Likely contact",
  1: "Stationary",
  0: "Moving",
  "-1": "Uncertain",
};
export const stateColors = {
  3: "#087f71",
  4: "#c45c39",
  2: "#326ac4",
  1: "#087f71",
  0: "#c45c39",
  "-1": "#a69053",
};
export { loadJson } from "../../shared/loadJson.js";
export function validateRecord(record) {
  const n = record.times?.length;
  if (
    record.schemaVersion !== 7 ||
    n !== 1000 ||
    record.poses.length !== n ||
    record.poses.some(
      (p) => p.length !== record.bodyCount * 7 || !p.every(Number.isFinite),
    ) ||
    record.times.some(
      (t, i) => !Number.isFinite(t) || (i && t <= record.times[i - 1]),
    )
  )
    throw new Error("Invalid contact-label recording");
  const graph = record.supportGraph;
  if (
    graph?.states?.length !== n ||
    graph.withoutWalkingConstraints?.states?.length !== n ||
    graph.withoutWalkingConstraints?.withoutVelocity?.states?.length !== n ||
    graph.states.some((s) => ![0, 1, 2, 3].includes(s)) ||
    graph.withoutVelocity?.states?.length !== n ||
    graph.withoutVelocity.states.some((s) => ![0, 1, 2, 3].includes(s)) ||
    !["left", "right"].every(
      (side) =>
        graph.bodyVelocities[side]?.length === n &&
        graph.bodyVelocities[side].every(
          (v) => v.length === 3 && v.every(Number.isFinite),
        ),
    ) ||
    !["left", "right"].every(
      (side) =>
        graph.displacements[side]?.length === n &&
        graph.displacements[side].every((d) => Number.isFinite(d) && d >= 0),
    )
  )
    throw new Error("Invalid support graph recording");
  for (const enforced of [false, true])
    for (const velocity of [false, true]) {
      const states = supportGraphResult(record, velocity, enforced).states;
      const invalid = states.some((state, k) => {
        if (![0, 1, 2, 3].includes(state)) return true;
        if (!enforced) return false;
        const previous = states[k - 1];
        return state === 3 ||
          (state === 0 && previous === 1) ||
          (state === 1 && previous === 0);
      });
      if (invalid)
        throw new Error("Invalid walking constraint result");
    }
  for (const side of ["left", "right"]) {
    const foot = record.feet[side];
    if (
      foot.worldContactPoses.length !== n ||
      foot.motion.maxSpeed.length !== n ||
      foot.motion.maxExcursion.length !== n
    )
      throw new Error("Incomplete foot motion");
    for (const preset of Object.keys(record.presets)) {
      const target = foot.targets[preset];
      if (
        target.walkingLabels?.length !== n ||
        target.walkingLabels.some((l, i) => l !== target.labels[i] && l !== 2)
      )
        throw new Error("Invalid walking targets");
      if (
        target.labels.length !== n ||
        target.scores.length !== n ||
        target.labels.some((l) => ![-1, 0, 1].includes(l)) ||
        target.scores.some((s) => !Number.isFinite(s) || s < 0 || s > 1)
      )
        throw new Error("Invalid contact targets");
    }
  }
  return record;
}
export function intervals(labels) {
  const result = [];
  for (let first = 0; first < labels.length; ) {
    let end = first + 1;
    while (end < labels.length && labels[end] === labels[first]) end++;
    result.push({ start: first, end, label: labels[first] });
    first = end;
  }
  return result;
}

export const supportNames = {
  0: "L",
  1: "R",
  2: "Double stance",
  3: "Neither",
  "-1": "Unresolved",
};
export const supportColors = {
  0: "#d64b4b",
  1: "#2c9b59",
  2: "#dfbc36",
  3: "#5277b8",
  "-1": "#999999",
};
export const markerColors = {
  0: "#ffffff",
  1: "#000000",
  2: "#000000",
  3: "#000000",
  4: "#ffffff",
  "-1": "#999999",
};
export function supportGraphResult(
  record,
  velocityAiding = true,
  enforceWalkingSequence = true,
) {
  const graph = enforceWalkingSequence
    ? record.supportGraph
    : record.supportGraph.withoutWalkingConstraints;
  return velocityAiding ? graph : graph.withoutVelocity;
}
export function footLabels(
  record,
  side,
  preset,
  walkingPrior,
  method,
  velocityAiding = true,
  enforceWalkingSequence = true,
) {
  if (method === "graph")
    return supportGraphResult(record, velocityAiding, enforceWalkingSequence).states.map((state) =>
      state === 3 || state === (side === "left" ? 1 : 0) ? 4 : 3,
    );
  return record.feet[side].targets[preset][
    walkingPrior ? "walkingLabels" : "labels"
  ];
}
export function windowSupportStates(record, preset, walkingPrior) {
  const left = footLabels(record, "left", preset, walkingPrior, "window");
  const right = footLabels(record, "right", preset, walkingPrior, "window");
  return left.map((l, i) => {
    const r = right[i],
      ls = l === 1 || l === 2,
      rs = r === 1 || r === 2;
    return ls && rs ? 2 : ls && r === 0 ? 0 : rs && l === 0 ? 1 : -1;
  });
}
