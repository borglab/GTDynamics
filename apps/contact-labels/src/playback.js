/** Map elapsed playback time to a valid sample in a 50 Hz recording. */
export function playbackFrame(first, elapsedMs, frameCount) {
  // A requestAnimationFrame timestamp can slightly precede the performance.now()
  // captured by Play. Flooring that negative delta would select sample -1.
  const elapsed = Math.max(0, elapsedMs);
  return Math.min(frameCount - 1, first + Math.floor(elapsed / 20));
}
