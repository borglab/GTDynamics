import test from "node:test";
import assert from "node:assert/strict";
import { playbackFrame } from "../src/playback.js";

test("an animation timestamp before Play never selects a negative sample", () => {
  // requestAnimationFrame timestamps mark the start of the rendering frame.
  // The first callback can precede the performance.now() captured by Play.
  for (const elapsed of [-16, -0.1, 0, 19.99]) {
    assert.equal(playbackFrame(0, elapsed, 1000), 0);
  }
  assert.equal(playbackFrame(0, 20, 1000), 1);
  assert.equal(playbackFrame(0, 700, 1000), 35);
});

test("resuming playback does not move backwards on its first callback", () => {
  assert.equal(playbackFrame(205, -0.1, 1000), 205);
  assert.equal(playbackFrame(205, 20, 1000), 206);
});

test("playback stops at the last sample, including after a delayed callback", () => {
  assert.equal(playbackFrame(0, 19980, 1000), 999);
  assert.equal(playbackFrame(998, 1000, 1000), 999);
  assert.equal(playbackFrame(0, 60000, 1000), 999);
});
