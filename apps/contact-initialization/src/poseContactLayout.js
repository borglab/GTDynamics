// Positions use the native SVG viewport shared by the pose timeline and its overlays.
export const frameX = (frame) => 32 + 58 * frame;

const observationY = (variable) => variable.y + (variable.frame === undefined ? -13 : 16);

/** Each stance spans its first and last observing frames, even during double support. */
function contactVariables({ frames, contacts }) {
  const spans = new Map();
  frames.forEach(({ observations }, frame) => {
    for (const { contactId } of observations) {
      const first = spans.get(contactId)?.[0] ?? frame;
      spans.set(contactId, [first, frame]);
    }
  });
  return new Map(contacts.map(({ id, foot }) => {
    const [first, last] = spans.get(id);
    return [id, {
      id: `C${id}`, label: `${foot[0].toUpperCase()}${id}`, known: false, foot,
      x: frameX((first + last) / 2), y: foot === "left" ? 175 : 265,
    }];
  }));
}

/** Convert one recorded variant to fgz's positioned, bipartite graph. */
export function poseContactLayout(graph) {
  const poses = graph.frames.map((_, frame) => ({
    id: `X${frame}`, label: `X_{${frame}}`, known: false, frame, x: frameX(frame), y: 64,
  }));
  const contacts = contactVariables(graph);
  const factors = [];
  function add(kind, key, variables, x, y, title, foot) {
    factors.push({ id: `${kind}:${key}`, kind, variables, x, y, title, foot, shape: "square", label: "" });
  }

  for (const pose of poses) {
    const { frame, id, x, y } = pose;
    if (frame > 0 && graph.between) {
      add("motion", frame, [poses[frame - 1].id, id], frameX(frame - 0.5), y,
        `Weak motion ${frame - 1} → ${frame}`);
    }
    if (frame > 0 && graph.upright) {
      add("upright", frame, [id], x, y - 36, `Pelvis upright ${frame}`);
    }
    for (const { contactId } of graph.frames[frame].observations) {
      const contact = contacts.get(contactId);
      add("observation", `${frame}:${contactId}`, [id, contact.id],
        (x + contact.x) / 2, (observationY(pose) + observationY(contact)) / 2,
        `Event ${frame} observes ${contact.label}`, contact.foot);
    }
  }
  for (const [id, contact] of contacts) {
    const { x, y } = contact;
    if (graph.ground) add("height", id, [contact.id], x, y + 35, `Contact height ${id}`);
    if (graph.normal) add("normal", id, [contact.id], x + 28, y, `Sole normal ${id}`);
  }

  return { variables: [...poses, ...contacts.values()], factors };
}

/** Route observation and unary-factor edges to the appropriate side of each glyph. */
export function edgeEndpoint({ kind }, variable) {
  const { x, y } = variable;
  switch (kind) {
    case "observation": return { x, y: observationY(variable) };
    case "upright": return { x, y: y - 16 };
    case "height": return { x, y: y + 15 };
    case "normal": return { x: x + 15, y };
    default: return undefined; // fgz connects motion edges to the pose center.
  }
}
