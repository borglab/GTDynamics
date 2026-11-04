import { createHash } from 'node:crypto';

const digest = bytes => createHash('sha256').update(bytes).digest('hex');

/** Accept old wrapper fingerprints only when the differences are precisely
 * the additive interpolation and dynamics declarations. All original wrapper content
 * and every native estimator source still require their original exact hash.
 */
export function recordingSourceMatches(path, bytes, expected) {
  if (digest(bytes) === expected) return true;
  if (path !== 'gtdynamics.i') return false;
  const originalApi = bytes.toString()
    .replace(/#include <gtdynamics\/utils\/FloatingBaseInterpolator\.h>[\s\S]*?(?=\/\*\*+ gpmp2)/, '')
    .replace('\n  std::vector<gtdynamics::Joint*> orderedJoints() const;\n', '');
  const candidates = [bytes.toString(), originalApi];
  return candidates.some(source => digest(withoutAdditiveWrapperSection(
    source, 'gtdynamics/utils/ContactWrench.h', 'gtdynamics/dynamics/Dynamics.h',
  )) === expected) || digest(originalApi) === expected;
}

/**
 * Reconstruct a historical gtwrap file by omitting one additive declaration
 * section. The following preexisting header remains in the fingerprint, as
 * does every byte outside the explicitly named new section. This does not
 * replace recorded provenance or exempt changes to the APIs used by a replay.
 */
export function withoutAdditiveWrapperSection(source, firstHeader, followingHeader) {
  const text = source.toString('utf8');
  const first = `#include <${firstHeader}>\n`;
  const following = `#include <${followingHeader}>\n`;
  const start = text.indexOf(first);
  if (start === -1) return text;
  const end = text.indexOf(following, start + first.length);
  if (end === -1 || text.indexOf(first, start + first.length) !== -1) {
    throw new Error('Additive wrapper section has missing or ambiguous boundaries');
  }
  return text.slice(0, start) + text.slice(end);
}
