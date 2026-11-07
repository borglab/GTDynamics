import assert from "node:assert/strict";
import test from "node:test";
import { edgeEndpoint, poseContactLayout } from "../src/poseContactLayout.js";

const recording = {
  frames: [[0], [0, 1], [1]].map(ids => ({
    observations: ids.map(contactId => ({ contactId })),
  })),
  contacts: [{ id: 0, foot: "left" }, { id: 1, foot: "right" }],
};

test("stance markers span their observations and factor markers bisect the attachment points", () => {
  const { variables, factors } = poseContactLayout(recording);
  assert.deepEqual(variables.map(({ id, x, y }) => [id, x, y]), [
    ["X0", 32, 64], ["X1", 90, 64], ["X2", 148, 64],
    ["C0", 61, 175], ["C1", 119, 265],
  ]);
  const [left, , right] = factors;
  assert.deepEqual(factors.map(f => f.variables), [
    ["X0", "C0"], ["X1", "C0"], ["X1", "C1"], ["X2", "C1"],
  ]);
  assert.deepEqual([left.x, left.y, left.foot], [46.5, 121, "left"]);
  assert.deepEqual([right.x, right.y, right.foot], [104.5, 166, "right"]);
  assert.deepEqual(edgeEndpoint(right, variables[1]), { x: 90, y: 80 });
  assert.deepEqual(edgeEndpoint(right, variables[4]), { x: 119, y: 252 });
});

test("optional factors use the recorded variant and skip the fixed initial pose", () => {
  const { factors, variables } = poseContactLayout({
    ...recording, between: true, upright: true, ground: true, normal: true,
  });
  for (const [kind, endpoints] of [
    ["motion", [["X0", "X1"], ["X1", "X2"]]],
    ["upright", [["X1"], ["X2"]]],
    ["height", [["C0"], ["C1"]]],
    ["normal", [["C0"], ["C1"]]],
  ]) {
    assert.deepEqual(factors.filter(f => f.kind === kind).map(f => f.variables), endpoints);
  }
  const position = (kind) => {
    const f = factors.find(factor => factor.kind === kind);
    return [f.x, f.y, edgeEndpoint(f, variables.find(v => v.id === f.variables[0]))];
  };
  assert.deepEqual(position("motion"), [61, 64, undefined]);
  assert.deepEqual(position("upright"), [90, 28, { x: 90, y: 48 }]);
  assert.deepEqual(position("height"), [61, 210, { x: 61, y: 190 }]);
  assert.deepEqual(position("normal"), [89, 175, { x: 76, y: 175 }]);
  assert.equal(new Set(factors.map(f => f.id)).size, factors.length);
});
