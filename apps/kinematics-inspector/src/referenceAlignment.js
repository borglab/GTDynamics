/** Apply an offline estimate_T_reference fit to world position curves only.
 * Foot-in-pelvis positions are invariant to a common rigid world transform.
 */
export function alignedPositionPlots(plots, alignment) {
  if (!alignment) return plots;
  const {rotation: r, translation: t} = alignment;
  const reference = plots.pelvisWorld.reference.map(p => r.map((row, i) =>
    row.reduce((sum, value, j) => sum + value * p[j], t[i])));
  return {...plots, pelvisWorld: {...plots.pelvisWorld, reference}};
}
