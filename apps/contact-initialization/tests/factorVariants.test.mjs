import assert from "node:assert/strict";
import test from "node:test";
import { factorVariants } from "../src/factorVariants.js";
test("all eight independent combinations of the shared priors are selectable", () => {
  assert.equal(factorVariants.length, 8);
  assert.equal(new Set(factorVariants.map((v) => v.id)).size, 8);
  assert.equal(
    new Set(factorVariants.map((v) => [v.between, v.ground, v.upright].join()))
      .size,
    8,
  );
});
