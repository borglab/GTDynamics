import { recordedSourceBytes } from '../../shared/tests/recordedSources.mjs';
import { recordingSourceMatches } from '../../shared/recordingProvenance.mjs';
import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { gunzipSync } from "node:zlib";
import { createHash } from "node:crypto";
import { Quaternion, Vector3 } from "three";
import {
  validateRecord,
  intervals,
  footLabels,
  supportGraphResult,
  windowSupportStates,
} from "../src/recording.js";
const root = new URL("../../../", import.meta.url),
  publicRoot = new URL("../public/", import.meta.url);
const index = JSON.parse(readFileSync(new URL("snippets.json", publicRoot)));
const records = index.map((s) =>
  validateRecord(
    JSON.parse(
      gunzipSync(readFileSync(new URL(`snippets/${s.id}.json.gz`, publicRoot))),
    ),
  ),
);
test("three distinct 20-second SSIO windows, attribution, and registered app", () => {
  assert.equal(records.length, 3);
  assert.deepEqual(
    records.map((r) => r.times[0]),
    [24, 78, 198],
  );
  assert(index.every((s) => s.label.includes(" s")));
  assert.match(
    readFileSync(new URL("SSIO_ATTRIBUTION.txt", publicRoot), "utf8"),
    /Creative Commons Attribution 4.0/,
  );
  const apps = JSON.parse(readFileSync(new URL("apps/registry.json", root)));
  assert(apps.some((a) => a.id === "contact-labels"));
  const kinematics = apps.findIndex((app) => app.title === "Kinematics");
  assert.equal(apps[kinematics + 1].title, "Interpolation");
  assert.deepEqual(apps.slice(-3).map(app => app.title), [
    "Kinodynamics", "Motion Planning", "SCATE UR5",
  ]);
});
for (const r of records) {
  test(`${r.id}: labels, windows, contact frames, and privileged provenance`, () => {
    assert.equal(r.durationSeconds, 20);
    assert.equal(r.sampleRateHz, 50);
    assert.equal(r.times.length, 1000);
    assert(Math.abs(r.times.at(-1) - r.times[0] - 19.98) < 1e-10);
    assert.match(r.source.targetInputs[0], /LiDAR/);
    assert(
      !r.source.futureClassifierInputs.some((s) =>
        /pose|LiDAR|velocityWorld/.test(s),
      ),
    );
    for (const [path, hash] of Object.entries(r.provenance.sources))
      assert(recordingSourceMatches(path, recordedSourceBytes(path, hash), hash), `Historical native labels: ${path}`);
    for (const side of ["left", "right"]) {
      const foot = r.feet[side];
      assert(foot.motion.completeWindow.every(Boolean));
      for (const name of Object.keys(r.presets)) {
        const p = r.presets[name],
          target = foot.targets[name];
        assert.equal(target.labels.length, 1000);
        assert.equal(
          intervals(target.labels).reduce((sum, x) => sum + x.end - x.start, 0),
          1000,
        );
        for (let k = 0; k < 1000; k++) {
          const metrics = [
            ["maxSpeed", "Speed"],
            ["maxExcursion", "Excursion"],
            ["angularSpeed", "AngularSpeed"],
            ["angularExcursion", "AngularExcursion"],
          ];
          if (target.labels[k] === 1)
            for (const [field, suffix] of metrics)
              assert(foot.motion[field][k] <= p["stationary" + suffix]);
          if (target.labels[k] === 0)
            assert(
              metrics.some(
                ([field, suffix]) =>
                  foot.motion[field][k] >= p["moving" + suffix],
              ),
            );
          // Check the saved contact frame against the body's rigid transform.
          const pose = r.poses[k],
            offset = foot.body * 7,
            position = new Vector3().fromArray(pose, offset),
            rotation = new Quaternion().fromArray(pose, offset + 3).normalize();
          const local = foot.bodyTcontact,
            point = new Vector3(local[0][3], local[1][3], local[2][3])
              .applyQuaternion(rotation)
              .add(position);
          assert(
            point.distanceTo(
              new Vector3(...foot.worldContactPoses[k].slice(0, 3)),
            ) < 3e-6,
          );
          assert(
            rotation.angleTo(
              new Quaternion(...foot.worldContactPoses[k].slice(3)).normalize(),
            ) < 5e-6,
          );
        }
      }
      const count = (name) =>
        foot.targets[name].labels.filter((l) => l === 1).length;
      assert(count("strict") <= count("default"));
      assert(count("default") <= count("tolerant"));
    }
  });
}
test("the known first turn is not labeled a fully stationary right sole", () => {
  const r = records.find((r) => r.id === "onset"),
    k = r.times.findIndex((t) => Math.abs(t - 28.5) < 1e-9);
  assert(k >= 0);
  assert.equal(r.feet.right.targets.default.labels[k], 0);
  assert(r.feet.right.motion.angularSpeed[k] > 0.5);
});

test("walking support is separate from raw stationarity in every snippet and preset", () => {
  for (const r of records) {
    assert.equal(r.walkingPrior.inferredLabel, 2);
    for (const name of Object.keys(r.presets)) {
      const p = r.presets[name];
      let inferred = 0;
      for (const [side, other] of [
        ["left", "right"],
        ["right", "left"],
      ]) {
        const a = r.feet[side],
          b = r.feet[other],
          target = a.targets[name];
        for (let k = 0; k < r.times.length; k++) {
          const translates = (foot) =>
            foot.motion.maxSpeed[k] >= p.movingSpeed ||
            foot.motion.maxExcursion[k] >= p.movingExcursion;
          const promote =
            a.motion.completeWindow[k] &&
            b.motion.completeWindow[k] &&
            target.labels[k] !== 1 &&
            !translates(a) &&
            translates(b);
          assert.equal(target.walkingLabels[k], promote ? 2 : target.labels[k]);
          if (promote) {
            inferred++;
            assert.notEqual(b.targets[name].walkingLabels[k], 2);
          }
        }
      }
      assert(inferred > 0, `${r.id}/${name} should exercise the prior`);
    }
  }
});

test("support graph uses consecutive translations and preserves comparison inputs", () => {
  for (const record of records) {
    const g = record.supportGraph;
    assert.match(g.inference, /max-product/);
    assert.equal(
      g.factorCount,
      10 * g.sliceCount - 7,
    );
    assert.equal(
      g.sliceCount,
      record.times.length + 2 * g.contextSamplesPerEnd,
    );
    assert.equal(g.variableCount, 2 * g.sliceCount);
    assert.equal(g.withoutVelocity.factorCount, 8 * g.sliceCount - 7);
    assert.equal(g.sampleIntervalSeconds, 0.02);
    assert.equal(g.singleSliceWeight, 0.1);
    assert.equal(g.enforceWalkingSequence, true);
    assert.equal(g.walkingConstraintFactorCount, 2*g.sliceCount-1);
    for (const velocity of [true, false]) {
      const strict = supportGraphResult(record, velocity, true);
      const relaxed = supportGraphResult(record, velocity, false);
      assert.equal(relaxed.factorCount, (velocity ? 8 : 6)*g.sliceCount-6);
      assert.equal(strict.factorCount-relaxed.factorCount, g.walkingConstraintFactorCount);
      assert(strict.states.every((s,k) => s !== 3 &&
        !(k && ((s===0 && strict.states[k-1]===1) || (s===1 && strict.states[k-1]===0)))));
      for (const enforce of [true, false]) {
        const selected = supportGraphResult(record, velocity, enforce);
        for (const side of ["left", "right"])
          assert.deepEqual(footLabels(record, side, "default", true, "graph", velocity, enforce),
            selected.states.map((s) => s===3 || s===(side==="left" ? 1 : 0) ? 4 : 3));
      }
    }
    assert(Number.isFinite(g.solveSeconds) && g.solveSeconds > 0);
    for (const row of g.transition)
      assert(Math.abs(row.reduce((a, b) => a + b, 0) - 1) < 1e-12);
    for (const side of ["left", "right"]) {
      const positions = record.feet[side].worldContactPoses;
      for (let k = 1; k < positions.length; k++) {
        const distance = Math.hypot(
          ...positions[k].slice(0, 3).map((v, j) => v - positions[k - 1][j]),
        );
        assert(Math.abs(distance - g.displacements[side][k]) < 2e-6);
        assert(Math.abs(record.times[k] - record.times[k - 1] - 0.02) < 1e-10);
      }
      const labels = footLabels(record, side, "default", true, "graph");
      assert.deepEqual(
        labels,
        footLabels(record, side, "strict", false, "graph"),
      );
      assert(labels.every((l) => l === 3 || l === 4));
    }
    const left = footLabels(record, "left", "default", true, "graph");
    const right = footLabels(record, "right", "default", true, "graph");
    assert(left.every((l, i) => (l === 4 && right[i] === 4) === (g.states[i] === 3)));
    assert(g.states.every((state) => [0, 1, 2, 3].includes(state)));
    for (const prior of [false, true]) {
      const states = windowSupportStates(record, "default", prior);
      assert.equal(states.length, 1000);
      assert(states.every((s) => [-1, 0, 1, 2].includes(s)));
    }
  }
});

test("report removal preserves all numerical recording content", () => {
  for (const record of records) {
    const { provenance, ...content } = record;
    // Fresh numerical exports have no maintenance entry. Historical archives
    // retain a content hash when only presentation provenance was updated.
    const migration = provenance.maintenance?.at(-1);
    if (!migration) continue;
    assert.equal(migration.kind, "presentation-only");
    assert.equal(createHash("sha256").update(JSON.stringify(content)).digest("hex"), migration.retainedContentSha256);
    assert.match(migration.previousExporterSha256, /^[a-f0-9]{64}$/);
    assert.equal(Object.keys(migration.retiredReportSources).length, 2);
    assert(!Object.keys(provenance.sources).some(path => /factor_report|typeset_report/.test(path)));
  }
});
