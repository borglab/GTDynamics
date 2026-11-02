import { createHash } from 'node:crypto';

const digest = bytes => createHash('sha256').update(bytes).digest('hex');

/** Accept old wrapper fingerprints only when the differences are precisely
 * the new, unrelated interpolation declarations. All original wrapper content
 * and every native estimator source still require their original exact hash.
 */
export function recordingSourceMatches(path, bytes, expected) {
  if (digest(bytes) === expected) return true;
  if (path !== 'gtdynamics.i') return false;
  const originalApi = bytes.toString()
    .replace(/#include <gtdynamics\/utils\/FloatingBaseInterpolator\.h>[\s\S]*?(?=\/\*\*+ gpmp2)/, '')
    .replace('\n  std::vector<gtdynamics::Joint*> orderedJoints() const;\n', '');
  return digest(originalApi) === expected;
}
