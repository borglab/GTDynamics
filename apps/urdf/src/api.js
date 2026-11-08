/** Requests to the GTDynamics Python backend (scripts/server.py). */

/** Paths are relative to the page, so they work at / and under /urdf/. */
export async function api(path, body, fetchImpl = fetch) {
  const res = await fetchImpl(path, body === undefined ? {} : {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({ error: `HTTP ${res.status}` }));
  if (!res.ok || data.error) throw new Error(data.error || `HTTP ${res.status}`);
  return data;
}

/**
 * Runs at most one request at a time; while one is in flight only the most
 * recent pending request is kept, so dragging never builds up a backlog.
 */
export function latestOnly(send) {
  let inFlight = false;
  let pending = null;
  async function pump() {
    const job = pending;
    pending = null;
    inFlight = true;
    try {
      await send(job);
    } finally {
      inFlight = false;
      if (pending) pump();
    }
  }
  return (job) => {
    pending = job;
    if (!inFlight) pump();
  };
}
