import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { recordingSourceMatches } from '../recordingProvenance.mjs';

test('additive interpolation bindings preserve exact historical estimator provenance', () => {
  const source = readFileSync(new URL('../../../gtdynamics.i', import.meta.url));
  const publicRoot = new URL('../../contact-initialization/public/', import.meta.url);
  const catalogue = JSON.parse(readFileSync(new URL('sequences.json', publicRoot)));
  const path = catalogue.sequences[0].recording;
  const bytes = readFileSync(new URL(path, publicRoot));
  const record = JSON.parse(path.endsWith('.gz') ? gunzipSync(bytes) : bytes);
  const expected = record.source.sources['gtdynamics.i'];
  assert(recordingSourceMatches('gtdynamics.i', source, expected));
  const changed = Buffer.from(source.toString().replace('InitializeSolutionContactPoses', 'ChangedInitializer'));
  assert(!recordingSourceMatches('gtdynamics.i', changed, expected));
  assert(!recordingSourceMatches('unrelated.cpp', source, expected));
});
