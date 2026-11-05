import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { gunzipSync } from "node:zlib";
import { recordedSourceBytes } from '../../shared/tests/recordedSources.mjs';
import { recordingSourceMatches } from '../../shared/recordingProvenance.mjs';
import {
  composePoses,
  inversePose,
  quaternionToRotation,
} from "../src/poseMath.js";
const root = new URL("../public/", import.meta.url),
  repo = new URL("../../../", import.meta.url);
const read = async (file) => {
  const bytes = await readFile(new URL(file, root));
  return JSON.parse(file.endsWith(".gz") ? gunzipSync(bytes) : bytes);
};
const catalogue = await read("sequences.json");
const sha = (bytes) => createHash("sha256").update(bytes).digest("hex");
const squaredNorm = (v) => v.reduce((s, x) => s + x * x, 0);
const rotationAngle = (pose) =>
  Math.acos(
    Math.max(
      -1,
      Math.min(
        1,
        (pose.rotation[0] + pose.rotation[4] + pose.rotation[8] - 1) / 2,
      ),
    ),
  );
const matrixPose = (matrix) => ({
  rotation: matrix.slice(0, 3).flatMap((row) => row.slice(0, 3)),
  translation: matrix.slice(0, 3).map((row) => row[3]),
});
const close = (a, b, tol = 1e-8) =>
  assert(Math.abs(a - b) < tol, `${a} != ${b}`);
for (const entry of catalogue.sequences) {
  const data = await read(entry.data),
    recording = await read(entry.recording);
  test(`${entry.id}: canonical frames, preserved schedule and pose-only provenance`, async () => {
    assert.equal(recording.schemaVersion, 3);
    assert.equal(
      recording.solver,
      "Initializer.InitializeSolutionContactPoses",
    );
    assert.equal(
      recording.source.referenceExcludedFromOptimization,
      !entry.id.startsWith("ssio-"),
    );
    assert.equal(
      recording.source.datasetSha256,
      sha(await readFile(new URL(entry.data, root))),
    );
    for (const [path, hash] of Object.entries(recording.source.sources)) {
      assert.match(hash, /^[a-f0-9]{64}$/);
      // Label archives also contain report/export provenance. Check the actual
      // label inputs below so documentation-only changes do not stale a solve.
      if (
        path !== recording.source.labelSource
      )
        assert(recordingSourceMatches(path, recordedSourceBytes(path, hash), hash), path);
    }
    assert.equal(data.model.soleCorners, undefined);
    assert.equal(data.model.soleOutline, undefined);
    assert.equal(data.metadata.kinematicModel.entry, "g1_mjlab");
    assert.deepEqual(Object.keys(data.model.contactFrames), ["left", "right"]);
    assert(recording.runs[recording.defaultVariant]);
    assert.equal(recording.models, undefined);
    if (entry.id === "g1-steptouch-30s") {
      assert.deepEqual(
        data.samples
          .filter((s) => !s.support.left && !s.support.right)
          .map((s) => s.index),
        Array.from({ length: 11 }, (_, i) => 1357 + i),
      );
      assert.deepEqual(Object.keys(recording.runs), [
        "between",
        "between-upright",
        "between-ground",
        "between-ground-upright",
      ]);
      assert.match(recording.unavailable.none, /Disconnected/);
    } else if (entry.id.startsWith("ssio-")) {
      assert.equal(Object.keys(recording.runs).length, 8);
      const labels = JSON.parse(
        gunzipSync(await readFile(new URL(recording.source.labelSource, repo))),
      );
      assert.deepEqual(
        recording.source.labelStates,
        labels.supportGraph.states,
      );
      assert.deepEqual(
        recording.source.sourceIndices,
        labels.times.map((time) => Math.round(time * 50)),
      );
      assert.deepEqual(
        data.samples.map((sample) => sample.support),
        labels.supportGraph.states.map((state) => ({
          left: state === 0 || state === 2,
          right: state === 1 || state === 2,
        })),
      );
      assert.equal(data.samples.length, 1000);
      assert.equal(recording.defaultVariant, "between-ground-upright");
      for (const run of Object.values(recording.runs)) {
        assert.equal(run.graph.noise.contactRotationDegrees, 90);
        assert.equal(run.graph.groundHeight, 0);
        assert.equal(run.graph.normal, false);
        assert.equal(run.graph.initializeUpright, true);
        const first = run.graph.frames[0];
        const pelvis = run.evaluation.alignedReferencePoses[0];
        const heights = first.observations.map(
          (o) => composePoses(pelvis, o.localPose).translation[2],
        );
        close(heights.reduce((a, b) => a + b, 0) / heights.length, 0, 1e-8);
        const start = run.result.history[0];
        // Relative yaw changes preserve the anchor's world-up tilt.
        for (const pose of start.poses)
          close(pose.rotation[8], start.poses[0].rotation[8], 1e-9);
        for (const frame of run.graph.frames)
          assert.equal(labels.supportGraph.states[frame.sampleIndex], 2);
      }
    } else assert.equal(Object.keys(recording.runs).length, 8);
    assert.equal(recording.defaultVariant, "between-ground-upright");
    assert.equal(recording.source.sampling.mode, "double-support");
    assert.equal(recording.source.sampling.maxSliceIntervalMs, 200);
    for (const run of Object.values(recording.runs)) {
      const selected = run.graph.frames.map(frame=>frame.sampleIndex);
      for(const frame of run.graph.frames)assert(data.samples[frame.sampleIndex].support.left&&data.samples[frame.sampleIndex].support.right);
      let begin=0;
      while(begin<data.samples.length){
        const mask=sample=>Number(sample.support.left)*2+Number(sample.support.right);
        let end=begin;
        while(end+1<data.samples.length&&mask(data.samples[end+1])===mask(data.samples[begin]))end++;
        const retained=selected.filter(k=>k>=begin&&k<=end),dt=1/entry.sampleRateHz;
        if(mask(data.samples[begin])===3){
          if((end-begin+1)*dt<=.2+1e-12)assert.deepEqual(retained,[begin+Math.floor((end-begin)/2)]);
          else{
            assert.equal(retained[0],begin+1);assert.equal(retained.at(-1),end-1);
            const gaps=retained.slice(1).map((k,i)=>k-retained[i]);
            assert(Math.max(...gaps)*dt<=.2+1e-12);assert(Math.max(...gaps)-Math.min(...gaps)<=1);
          }
        }else assert.equal(retained.length,0);
        begin=end+1;
      }
      assert.equal(run.graph.initializeUpright, true);
      assert.equal(run.graph.normal, false);
      close(run.graph.noise.contactTranslation, 0.01);
      close(run.graph.noise.contactRotationDegrees, 90);
      close(run.graph.noise.height, 0.1);
      close(run.graph.noise.uprightDegrees, 15);
      close(run.graph.noise.betweenTranslation, 10);
      close(run.graph.noise.betweenRotationDegrees, 180);
    }
  });
  for (const [variant, run] of Object.entries(recording.runs))
    test(`${entry.id}/${variant}: native pose observations, gauges, history, residuals and timings`, () => {
      const { graph: g, result: r, benchmark: b } = run;
      assert.equal(
        g.freeDimensions,
        (g.frames.length - 1 + g.contacts.length) * 6,
      );
      assert.equal(r.history.length, r.iterations + 1);
      if (r.stalled) {
        assert.equal(r.converged, false);
        close(r.history.at(-1).objective, r.history.at(-2).objective, 1e-8);
      }
      const initial = r.history[0],
        last = r.history.at(-1);
      assert.equal(initial.contactPoses.length, g.contacts.length);
      assert(last.objective <= initial.objective + 1e-8);
      assert.equal(
        g.factorGroups.find((x) => x.id === "normal").count,
        g.normal ? g.contacts.length : 0,
      );
      assert.equal(
        g.factorGroups.find((x) => x.id === "height").count,
        g.ground ? g.contacts.length : 0,
      );
      assert.equal(
        g.factorGroups.find((x) => x.id === "motion").count,
        g.between ? g.frames.length - 1 : 0,
      );
      assert.equal(
        g.factorGroups.find((x) => x.id === "upright").count,
        g.upright ? g.frames.length - 1 : 0,
      );
      for (const s of r.history) {
        assert.deepEqual(
          s.poses[0],
          initial.poses[0],
          "Exact fixed root gauge",
        );
        assert.equal(s.contactPoses.length, g.contacts.length);
        close(
          Object.values(s.factorErrors).reduce((a, b) => a + b, 0),
          s.objective / 2,
          1e-7,
        );
        let translation = 0,
          angle = 0,
          count = 0;
        g.frames.forEach((f, i) =>
          f.observations.forEach((o) => {
            const predicted = composePoses(s.poses[i], o.localPose),
              actual = s.contactPoses[o.contactId];
            translation += squaredNorm(
              predicted.translation.map((v, a) => v - actual.translation[a]),
            );
            angle +=
              rotationAngle(composePoses(inversePose(predicted), actual)) ** 2;
            count++;
          }),
        );
        close(Math.sqrt(translation / count), s.rms);
        close(Math.sqrt(angle / count), s.rotationRms, 1e-7);
      }
      const seen = new Map();
      g.frames.forEach((f) => {
        close(f.time, data.samples[f.sampleIndex].t, 1e-6);
        const active = data.contacts
          .filter((c) => c.start <= f.sampleIndex && f.sampleIndex <= c.end)
          .map((c) => c.foot)
          .sort();
        assert.deepEqual(
          f.observations.map((o) => g.contacts[o.contactId].foot).sort(),
          active,
        );
        f.observations.forEach((o) => {
          const foot = g.contacts[o.contactId].foot,
            sample = data.samples[f.sampleIndex].feet[foot];
          const bodyTfoot = {
            rotation: quaternionToRotation(sample.q),
            translation: sample.p,
          };
          const expected = composePoses(
            composePoses(inversePose(g.rootBodyTcom), bodyTfoot),
            matrixPose(data.model.contactFrames[foot]),
          );
          expected.rotation.forEach((v, i) =>
            close(v, o.localPose.rotation[i], 1e-8),
          );
          expected.translation.forEach((v, i) =>
            close(v, o.localPose.translation[i], 1e-8),
          );
          const previous = seen.get(foot);
          if (previous) {
            const uninterrupted = data.samples
              .slice(previous.k, f.sampleIndex + 1)
              .every((s) => s.support[foot]);
            assert.equal(
              previous.id === o.contactId,
              uninterrupted,
              "Lift-off breaks native landmark continuity",
            );
          }
          seen.set(foot, { id: o.contactId, k: f.sampleIndex });
        });
      });
      assert.equal(b.runs, 11);
      assert.equal(b.warmup, 2);
      for (const stats of Object.values(b.phases)) {
        assert.equal(stats.samplesMs.length, 11);
        const sorted = stats.samplesMs.slice().sort((a, b) => a - b);
        close(stats.medianMs, sorted[5]);
        close(stats.minMs, sorted[0]);
        close(stats.p95Ms, sorted.at(-1));
        assert(stats.minMs > 0);
      }
    });
}
