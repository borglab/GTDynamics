import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { createHash } from 'node:crypto';
import { recordedSourceBytes } from './recordedSources.mjs';
import { withoutAdditiveWrapperSection, recordingSourceMatches } from '../recordingProvenance.mjs';

test('additive interpolation bindings preserve exact historical estimator provenance', () => {
  const publicRoot = new URL('../../contact-initialization/public/', import.meta.url);
  const catalogue = JSON.parse(readFileSync(new URL('sequences.json', publicRoot)));
  const path = catalogue.sequences[0].recording;
  const bytes = readFileSync(new URL(path, publicRoot));
  const record = JSON.parse(path.endsWith('.gz') ? gunzipSync(bytes) : bytes);
  const expected = record.source.sources['gtdynamics.i'];
  const source = recordedSourceBytes('gtdynamics.i', expected);
  assert(recordingSourceMatches('gtdynamics.i', source, expected));
  const changed = Buffer.from(source.toString().replace('InitializeSolutionContactPoses', 'ChangedInitializer'));
  assert(!recordingSourceMatches('gtdynamics.i', changed, expected));
  assert(!recordingSourceMatches('unrelated.cpp', source, expected));
});

const sha = source => createHash('sha256').update(source).digest('hex');

test('interpolation provenance retains its own API while omitting additive dynamics bindings', () => {
  const record = JSON.parse(gunzipSync(readFileSync(new URL('../../interpolation/public/recording.json.gz', import.meta.url))));
  const expected = record.provenance.sources['gtdynamics.i'];
  const source = recordedSourceBytes('gtdynamics.i', expected);
  assert(recordingSourceMatches('gtdynamics.i', source, expected));
  const changed = Buffer.from(source.toString().replace('FloatingBaseInterpolation solve() const;', 'FloatingBaseInterpolation changed() const;'));
  assert.notEqual(changed.toString(), source.toString());
  assert(!recordingSourceMatches('gtdynamics.i', changed, expected));
});

const historical = 'class Initializer { void initialize(); };\n\n#include <old/Dynamics.h>\nclass Dynamics {};\n';
const additive = '#include <new/Wrench.h>\nclass Wrench {};\n\n';
const current = historical.replace('#include <old/Dynamics.h>', additive + '#include <old/Dynamics.h>');
const scoped = source => withoutAdditiveWrapperSection(source, 'new/Wrench.h', 'old/Dynamics.h');

test('additive bindings preserve the original recording fingerprint exactly', () => {
  assert.equal(sha(scoped(current)), sha(historical));
  assert.equal(scoped(historical), historical);
  assert.equal(sha(scoped(current.replace('class Wrench {};', 'class Wrench { void extra(); };'))), sha(historical));
});

test('changes to initializer APIs or preexisting dynamics APIs still invalidate provenance', () => {
  assert.notEqual(sha(scoped(current.replace('void initialize();', 'void initialize(double sigma);'))), sha(historical));
  assert.notEqual(sha(scoped(current.replace('class Dynamics {};', 'class Dynamics { void changed(); };'))), sha(historical));
});

test('malformed additive-section boundaries fail instead of hiding source changes', () => {
  assert.throws(() => scoped(current.replace('#include <old/Dynamics.h>\n', '')), /boundaries/);
  assert.throws(() => scoped(additive + current), /boundaries/);
});


test('historical replay resolution rejects unknown source fingerprints', () => {
  assert.throws(() => recordedSourceBytes('gtdynamics.i', '0'.repeat(64)), /matches neither/);
});
