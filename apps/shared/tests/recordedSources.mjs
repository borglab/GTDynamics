/* GTDynamics Copyright 2026, Georgia Tech Research Corporation.
 * See LICENSE for license information.
 * @file recordedSources.mjs
 * @brief Resolve historical replay fingerprints against their archived source.
 * @author GTDynamics contributors
 */
import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { recordingSourceMatches } from '../recordingProvenance.mjs';

const root = fileURLToPath(new URL('../../../', import.meta.url));
const revision = readFileSync(new URL('../recordingSourceRevision.txt', import.meta.url), 'utf8').trim();
const archived = new Map();
if (!/^[a-f0-9]{40}$/.test(revision)) throw new Error('Invalid archived recording source revision');

/** Read matching current source or the immutable historical source checkpoint.
 * Historical replay checks validate that checkpoint, not outputs of today's
 * estimator. The caller must still verify the recorded hash, including the
 * narrowly defined additive-wrapper allowances in recordingSourceMatches.
 */
export function recordedSourceBytes(path, expected) {
  const current = readFileSync(resolve(root, path));
  if (recordingSourceMatches(path, current, expected)) return current;
  if (!archived.has(path)) {
    archived.set(path, execFileSync('git', ['show', `${revision}:${path}`], {
      cwd: root, maxBuffer: 16 * 1024 * 1024,
    }));
  }
  const historical = archived.get(path);
  if (!recordingSourceMatches(path, historical, expected)) {
    throw new Error(`Recording fingerprint for ${path} matches neither current source nor checkpoint ${revision}`);
  }
  return historical;
}
